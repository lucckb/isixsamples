# Example: e0_led_blink for stm32f411e_disco

_srcs = files(
	'src/blink.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('411disco_blink',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

# Objcopy to binary
_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('411disco_blink.binary',
	input: _elf,
	output: '411disco_blink.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

