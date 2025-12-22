project('isix-samples', 'c', 'cpp',
	version : '1.0.0',
	meson_version: '>=1.2',
	default_options : [
		'default_library=static',
		'c_std=gnu17',
		'cpp_std=gnu++23',
		'cpp_rtti=false',
		'cpp_eh=none',
		'werror=false',
		'warning_level=2',
	]
)

# Add link arguments from isixrtos subproject
# These are already set in isixrtos, but we need them in main project too
# Linker script path - same as in isixrtos but relative to main project
_linker_script = meson.project_source_root() / 'isixrtos' / 'libperiph' / 'scripts' / host_machine.cpu_family() / meson.get_external_property('subarch') / 'flash.ld'
add_project_link_arguments(
	'-nostdlib', '-nostartfiles', '--specs=nosys.specs',
	'-Wl,--defsym=_sys_stack_size=512',
	'-T' + _linker_script,
	'-lstdc++', '-lc', '-lm', '-lg', '-lgcc', '-flto=auto',
	'-Wl,--print-memory-usage',
	not get_option('debug') ? ['-Wl,--gc-sections'] : [],
	language : ['c', 'cpp']
)


# Add project argument for debug version
add_project_arguments(
	'-Wno-variadic-macros', '-Wno-long-long', '-pipe',
	'-Wa,-mapcs-32',
	get_option('debug')
		? '-DPDEBUG'
		: ['-fomit-frame-pointer', '-ffunction-sections', '-fdata-sections'],
	language : ['c', 'cpp']
)


# Get isixrtos as subproject (it's a git submodule in root directory)
# Pass gfx_pixel_format for stm32f469i_disco board (BGR8 = 4)
# This allows libgfx to be compiled with correct pixel format
isixrtos_dep = subproject('isixrtos',
	default_options: [
		'crystal_hz=' + get_option('crystal_hz').to_string(),
        'shutdown_api=true',
        'task_stack_check=true',
        'fifo_event_notify=true',
        'cpu_usage_api=true',
        'mem_protect=lite',
        'log_level=crit',
	],
	# Pass gfx_pixel_format as external property for boards that need it
	# stm32f469i_disco uses BGR8 (4), other boards use default RGB565
)

# Export libraries from isixrtos subproject
libisix = isixrtos_dep.get_variable('libisix')
libfoundation = isixrtos_dep.get_variable('libfoundation')
libperiph = isixrtos_dep.get_variable('libperiph')
libgfx = isixrtos_dep.get_variable('libgfx')
libtcpip = isixrtos_dep.get_variable('libtcpip')
libtls = isixrtos_dep.get_variable('libtls')
libgsm = isixrtos_dep.get_variable('libgsm')
libenergymeter = isixrtos_dep.get_variable('libenergymeter')
_config = isixrtos_dep.get_variable('_config')

# Board selection logic
_selected_board = get_option('board')
subdir(_selected_board)
