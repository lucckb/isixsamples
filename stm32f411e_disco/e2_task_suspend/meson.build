# Example: e2_task_suspend for stm32f411e_disco

_srcs = files(
	'src/task_suspend.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

_elf = executable('task_suspend',
	sources : [_srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('task_suspend.binary',
	input: _elf,
	output: 'task_suspend.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

