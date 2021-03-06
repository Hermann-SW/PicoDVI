#include <stdio.h>
#include <stdlib.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/ssi.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/stdlib.h"

#include "tmds_encode.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"

// TMDS bit clock 252 MHz
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VREG_VSEL VREG_VOLTAGE_1_10
#define DVI_TIMING dvi_timing_640x480p_60hz

#define LED_PIN 25

#define N_IMAGES 3
#define FRAMES_PER_IMAGE 300

uint8_t mandel[FRAME_WIDTH * (FRAME_HEIGHT / 2)];

#define PALETTE_SIZE 64
uint16_t palette[PALETTE_SIZE];

uint32_t tmds_palette_blue[PALETTE_SIZE * 2];
uint32_t tmds_palette_green[PALETTE_SIZE * 2];
uint32_t tmds_palette_red[PALETTE_SIZE * 2];

struct dvi_inst dvi0;
struct semaphore dvi_start_sem;

// Just dumped out register values from running the code in tmds_encode.c:
// channel_msb: 4, lshift_lower: 3, i0 ctrl: 02801c60, 00002117, i1 ctrl: 02801c6d, 00002117
// channel_msb: 10, lshift_lower: 0, i0 ctrl: 02801c43, 00002117, i1 ctrl: 02801c53, 00002117
// channel_msb: 15, lshift_lower: 0, i0 ctrl: 02801c68, 00002117, i1 ctrl: 00001c78, 00002117
//
// This saves a few cycles vs doing the setup dynamically (just enough to tip
// us over the edge as it happens, when proper DC balancing is enabled.)

extern uint32_t tmds_table_fullres_x[];
extern uint32_t tmds_table_fullres_y[];

static inline void init_interp_for_encode() {
	interp0_hw->ctrl[1] = 0x00002117u;
	interp1_hw->ctrl[1] = 0x00002117u;
	uint32_t lutbase = (uint32_t)(get_core_num() ? tmds_table_fullres_x : tmds_table_fullres_y);
	interp0_hw->base[2] = (uint32_t)lutbase;
	interp1_hw->base[2] = (uint32_t)lutbase;
}

static inline void prepare_scanline_core1(const uint32_t *colourbuf, uint32_t *tmdsbuf) {
	const uint pixwidth = 640;
#if 0
	// Blue
	interp0_hw->ctrl[0] = 0x02801c60u;
	interp1_hw->ctrl[0] = 0x02801c6du;
	tmds_fullres_encode_loop_16bpp_leftshift_x(colourbuf, tmdsbuf, pixwidth, 3);
	// Green
	interp0_hw->ctrl[0] = 0x02801c43u;
	interp1_hw->ctrl[0] = 0x02801c53u;
	tmds_fullres_encode_loop_16bpp_x(colourbuf, tmdsbuf + pixwidth, pixwidth);
	// Red
	interp0_hw->ctrl[0] = 0x02801c68u;
	interp1_hw->ctrl[0] = 0x00001c78u;
	tmds_fullres_encode_loop_16bpp_x(colourbuf, tmdsbuf + 2 * pixwidth, pixwidth);
#else
  interp0_hw->base[2] = (uint32_t)tmds_palette_blue;
  interp1_hw->base[2] = (uint32_t)tmds_palette_blue;
  interp0_hw->ctrl[0] = 0x02801c40u;
  interp1_hw->ctrl[0] = 0x02801c50u;
	tmds_fullres_encode_loop_16bpp_x(colourbuf, tmdsbuf, pixwidth);

  interp0_hw->base[2] = (uint32_t)tmds_palette_green;
  interp1_hw->base[2] = (uint32_t)tmds_palette_green;
  interp0_hw->ctrl[0] = 0x02801c40u;
  interp1_hw->ctrl[0] = 0x02801c50u;
	tmds_fullres_encode_loop_16bpp_x(colourbuf, tmdsbuf + pixwidth, pixwidth);

  interp0_hw->base[2] = (uint32_t)tmds_palette_red;
  interp1_hw->base[2] = (uint32_t)tmds_palette_red;
  interp0_hw->ctrl[0] = 0x02801c40u;
  interp1_hw->ctrl[0] = 0x02801c50u;
	tmds_fullres_encode_loop_16bpp_x(colourbuf, tmdsbuf + pixwidth * 2, pixwidth);
#endif

}

static inline void prepare_scanline_core0(const uint32_t *colourbuf, uint32_t *tmdsbuf) {
	const uint pixwidth = 640;
  interp0_hw->base[2] = (uint32_t)tmds_palette_blue;
  interp1_hw->base[2] = (uint32_t)tmds_palette_blue;
  interp0_hw->ctrl[0] = 0x02801c40u;
  interp1_hw->ctrl[0] = 0x02801c50u;
	tmds_fullres_encode_loop_16bpp_y(colourbuf, tmdsbuf, pixwidth);

  interp0_hw->base[2] = (uint32_t)tmds_palette_green;
  interp1_hw->base[2] = (uint32_t)tmds_palette_green;
  interp0_hw->ctrl[0] = 0x02801c40u;
  interp1_hw->ctrl[0] = 0x02801c50u;
	tmds_fullres_encode_loop_16bpp_y(colourbuf, tmdsbuf + pixwidth, pixwidth);

  interp0_hw->base[2] = (uint32_t)tmds_palette_red;
  interp1_hw->base[2] = (uint32_t)tmds_palette_red;
  interp0_hw->ctrl[0] = 0x02801c40u;
  interp1_hw->ctrl[0] = 0x02801c50u;
	tmds_fullres_encode_loop_16bpp_y(colourbuf, tmdsbuf + pixwidth * 2, pixwidth);
}

void init_palette() {
  for (int i = 0; i < PALETTE_SIZE; ++i) {
    if (i < 0x20) palette[i] = i;
    else if (i < 0x60) palette[i] = (i - 0x20) << 5;
    else if (i < 0xc0) palette[i] = ((i - 0x60) >> 2) << 11;
    else palette[i] = ((i - 0xc0) >> 1) * 0x0840;
  }

  for (int i = 0; i < PALETTE_SIZE; ++i) {
    uint16_t blue = (palette[i] << 1) & 0x3e;
    uint16_t green = (palette[i] >> 5) & 0x3f;
    uint16_t red = (palette[i] >> 10) & 0x3e;
    tmds_palette_blue[i] = tmds_table_fullres_x[blue];
    tmds_palette_blue[i + PALETTE_SIZE] = tmds_table_fullres_x[64 + blue];
    tmds_palette_green[i] = tmds_table_fullres_x[green];
    tmds_palette_green[i + PALETTE_SIZE] = tmds_table_fullres_x[64 + green];
    tmds_palette_red[i] = tmds_table_fullres_x[red];
    tmds_palette_red[i + PALETTE_SIZE] = tmds_table_fullres_x[64 + red];
  }
}

void __time_critical_func(fill_colour_buf)(uint16_t* colour_buf, uint32_t y) {
  if (y == 0) {
    for (int i = 0; i < FRAME_WIDTH; ++i) {
      colour_buf[i] = ((i + y) & 0x3f) << 2;
    }
    for (int i = 0; i < FRAME_WIDTH; ++i) {
      colour_buf[i+FRAME_WIDTH] = ((i + y + 1) & 0x3f) << 2;
    }
  }
}

// Core 1 handles DMA IRQs and runs TMDS encode on scanline buffers it
// receives through the mailbox FIFO
void __not_in_flash("core1_main") core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	sem_acquire_blocking(&dvi_start_sem);
	dvi_start(&dvi0);

	init_interp_for_encode();

	while (1) {
		const uint32_t *colourbuf = (const uint32_t*)multicore_fifo_pop_blocking();
		uint32_t *tmdsbuf = (uint32_t*)multicore_fifo_pop_blocking();
		prepare_scanline_core1(colourbuf, tmdsbuf);
		multicore_fifo_push_blocking(0);
	}
	__builtin_unreachable();
}

uint16_t img_buf[2 * FRAME_WIDTH];

int __not_in_flash("main") main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

	setup_default_uart();

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	// gpio_put(LED_PIN, 1);
  
  init_palette();

	printf("Configuring DVI\n");

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DEFAULT_DVI_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	printf("Core 1 start\n");
	sem_init(&dvi_start_sem, 0, 1);
	hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
	multicore_launch_core1(core1_main);

	uint heartbeat = 0;

	init_interp_for_encode();

	sem_release(&dvi_start_sem);
	while (1) {
		if (++heartbeat >= 30) {
			heartbeat = 0;
			gpio_xor_mask(1u << LED_PIN);
		}
		for (int y = 0; y < FRAME_HEIGHT; y += 2) {
      fill_colour_buf(img_buf, y);

			uint32_t *our_tmds_buf, *their_tmds_buf;
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &their_tmds_buf);
			multicore_fifo_push_blocking((uint32_t)(img_buf));
			multicore_fifo_push_blocking((uint32_t)their_tmds_buf);
	
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &our_tmds_buf);
			prepare_scanline_core0((const uint32_t*)(img_buf + FRAME_WIDTH), our_tmds_buf);
			
			multicore_fifo_pop_blocking();
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &their_tmds_buf);
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &our_tmds_buf);
		}
	}
	__builtin_unreachable();
}
	
