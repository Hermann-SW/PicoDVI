# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(sprite_bounce
	main.c
)

pico_set_boot_stage2(sprite_bounce vista-palette_boot2)

target_compile_options(sprite_bounce PRIVATE -Wall)

target_link_libraries(sprite_bounce
	pico_stdlib
	pico_multicore
	pico_util
	libdvi
	libsprite
)

# create map/bin/hex file etc.
pico_add_extra_outputs(sprite_bounce)
