# Example: e4_fifo_from_irq for stm32f411e_disco

_srcs = files(
	'src/fifoirq.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('fifoirq',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('fifoirq.binary',
	input: _elf,
	output: 'fifoirq.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

