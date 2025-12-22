# Example: e6_spi_mems for stm32f411e_disco

_srcs = files(
	'src/gyro_main.cpp',
	'src/l3gd20.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('spi_mems',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('spi_mems.binary',
	input: _elf,
	output: 'spi_mems.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

