# Board-level meson.build for zl41arm
# Projects list
_prjs = [
	'blink',
	'dacaudio'
]

# Additional extra libraries
_libs = [
	'zl41arm/board'
]

# Build board library first
subdir('board')

# Build all examples
foreach prj : _prjs
	subdir(prj)
endforeach

