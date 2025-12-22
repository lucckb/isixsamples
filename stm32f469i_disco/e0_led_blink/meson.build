# Example: e0_led_blink for stm32f469i_disco

_srcs = files(
	'src/blink.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('led_blink',
	sources : [ _srcs, platform_srcs ],
	include_directories : [ _include, platform_includes ],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
		libgfx,
        _config,
	],
	cpp_args : [
		'-DHAVE_ISIX_CONFIG_H',
		'-DCONFIG_GFX_PIXEL_FORMAT=CONFIG_GFX_PIXEL_FORMAT_BGR8',
	]
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('led_blink.binary',
	input: _elf,
	output: 'led_blink.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

