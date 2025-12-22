# Board-level meson.build for stm32f469i_disco
# This board uses BGR8 pixel format, so we need to rebuild isixrtos subproject
# with CONFIG_GFX_PIXEL_FORMAT=4 (CONFIG_GFX_PIXEL_FORMAT_BGR8)
# Note: This requires isixrtos to support gfx_pixel_format external property

# Projects list
_prjs = [
	'e0_led_blink',
	'e1_graphics'
]

# Build board library first
subdir('board')

# Build all examples
foreach prj : _prjs
	subdir(prj)
endforeach

