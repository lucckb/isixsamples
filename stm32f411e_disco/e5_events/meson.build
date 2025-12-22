# Example: e5_events for stm32f411e_disco

_srcs = files(
	'src/events.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('events',
	sources : [_srcs, platform_srcs],
	include_directories : [_include,  platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('events.binary',
	input: _elf,
	output: 'events.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

