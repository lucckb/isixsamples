# Example: e3_semaphore for stm32f411e_disco

_srcs = files(
	'src/semaphore.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('semaphore',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('semaphore.binary',
	input: _elf,
	output: 'semaphore.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

