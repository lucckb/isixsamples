# Example: e1_mutex for stm32f411e_disco

_srcs = files(
	'src/mutexes.cpp',
	'src/dts.cpp'
)

_include = include_directories('.', '../board/include')

_elf = executable('mutexes',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('mutexes.binary',
	input: _elf,
	output: 'mutexes.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

