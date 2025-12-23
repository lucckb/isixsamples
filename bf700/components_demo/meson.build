# Example: components_demo for bf700

_srcs = files(
	'app/app_env.cpp',
	'app/appmain.cpp',
	'app/codec_test.cpp',
	'app/tcpmain.cpp',
	'app/tftdemo.cpp',
	'app/tcpecho/tcpecho.c',
	'board/dts.cpp',
	'board/ili_fsmc_bus.cpp',
	'board/ili_gpio_bus.cpp',
	'board/keypad.cpp',
	'board/platform_startup.cpp',
	'board/si5351.cpp',
	'board/ethernet/ethernetif_f4.c',
	'board/ethernet/ethernetif.c',
	'board/ethernet/phy_dp83848.c',
	'board/ethernet/phy_ksz8081.c',
)

_include = include_directories('.', 'board/include', 'app')

_elf = executable('brefboard',
	sources : _srcs,
	include_directories : _include,
	c_args : [
		'-DETH_DRV_USE_RMII=1',
		'-DETH_DRV_PHY_ADDR=0',
		'-DETH_PHY_DRIVER_NAME=_ethernetif_phy_ksz8081_drv_',
		'-DCONFIG_GFX_PIXEL_FORMAT=1',
	],
	cpp_args : [
		'-DETH_DRV_USE_RMII=1',
		'-DETH_DRV_PHY_ADDR=0',
		'-DETH_PHY_DRIVER_NAME=_ethernetif_phy_ksz8081_drv_',
		'-DCONFIG_GFX_PIXEL_FORMAT=1',
		'-DHAVE_ISIX_CONFIG_H',
	],
	dependencies : [
		libisix,
		libperiph,
		libfoundation,
		libgfx,
		libtcpip,
	],
)

_objcopy = find_program(meson.get_external_property('objcopy'))
custom_target('brefboard.binary',
	input: _elf,
	output: 'brefboard.binary',
	command: [_objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@'],
	build_by_default: true
)

