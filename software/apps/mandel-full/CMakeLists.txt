# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(mandel-full
	main.c
  mandelbrot.c
)

pico_define_boot_stage2(mandel-full_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(mandel-full_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(mandel-full mandel-full_boot2)

target_compile_definitions(mandel-full PRIVATE
	DVI_VERTICAL_REPEAT=1
	DVI_N_TMDS_BUFFERS=0
	DVI_SYMBOLS_PER_WORD=2
	PICO_FLASH_SPI_CLKDIV=4
	)

target_compile_definitions(mandel-full PRIVATE PICO_STACK_SIZE=0x200)


target_link_libraries(mandel-full
	pico_stdlib
	pico_multicore
	libdvi
)

# create map/bin/hex file etc.
pico_add_extra_outputs(mandel-full)

pico_set_binary_type(mandel-full default)
