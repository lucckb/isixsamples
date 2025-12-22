# Example: blink for zl41arm

_srcs = files(
	'blinkf4.cpp'
)

_include = include_directories('.')

_elf = executable('blink',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('blink.binary',
	input: _elf,
	output: 'blink.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

