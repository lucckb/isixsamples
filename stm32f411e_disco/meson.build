# Board-level meson.build for stm32f411e_disco
# Projects list
_prjs = [
	'e0_led_blink',
	'e1_mutex',
	'e2_task_suspend',
	'e3_semaphore',
	'e4_fifo_from_irq',
	'e5_events',
	'e6_spi_mems',
	'e7_i2c_mems'
]

# Build board library first
subdir('board')

# Build all examples
foreach prj : _prjs
	subdir(prj)
endforeach

