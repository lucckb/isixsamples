# Example: e7_i2c_mems for stm32f411e_disco

_srcs = files(
	'src/i2c_main.cpp',
	'src/lms303.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('i2c_mems',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('i2c_mems.binary',
	input: _elf,
	output: 'i2c_mems.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

