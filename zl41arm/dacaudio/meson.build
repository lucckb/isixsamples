# Example: dacaudio for zl41arm

_srcs = files(
	'dac_audio.cpp',
	'dac_test.cpp'
)

_include = include_directories('include')

_elf = executable('dacaudio',
	sources : [ _srcs, platform_srcs],
	include_directories : [_include, platform_includes],
	c_args : ['-DUSE_FULL_LL_DRIVER=1'],
	cpp_args : ['-DUSE_FULL_LL_DRIVER=1', '-DHAVE_ISIX_CONFIG_H'],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('dacaudio.binary',
	input: _elf,
	output: 'dacaudio.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

