# Example: e1_graphics for stm32f469i_disco

_srcs = files(
	'src/displaymain.cpp',
	'src/tft_livedemo.cpp',
	'src/dts.cpp'
)

_include = include_directories('.')

# Create libgfx dependency with board include directory
# This allows libgfx to find isix_config.h during LTO linking
_libgfx_with_config = declare_dependency(
	dependencies : libgfx,
	include_directories : platform_includes,
)

_elf = executable('tft_driver',
	sources : [ _srcs, platform_srcs ],
	include_directories : [_include, platform_includes],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
		_libgfx_with_config,
	],
	cpp_args : ['-DHAVE_ISIX_CONFIG_H'],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('tft_driver.binary',
	input: _elf,
	output: 'tft_driver.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

