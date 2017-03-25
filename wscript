#!/usr/bin/env python

import sys
sys.path.insert(0, 'isixrtos/isixwaf')


top = '.'
out = 'build'

bld_subdirs = [
        'isixrtos',
        'isixrtos/libgfx',
        'isixrtos/libenergymeter',
        'isixrtos/libgsm',
        #'isixrtos/libtcpip',
        #'isixrtos/libtls',
        #'isixrtos/libfsfat',
        #'isixrtos/libisixdrvstm32',
        #'isixrtos/libisixdrvstm32',
        #'isix_c_examples',
        #'isix_cpp_examples',
        'simple/gfxdemo'
]
'''
bld_subdirs = [
        'isixrtos',
        'isixrtos/libisixdrvstm32',
        'isixrtos/libgfx',
        'isixrtos/libfsfat',
        'isixrtos/libtcpip',
        'refboard'
]
'''

def options(opt):
        opt.recurse( bld_subdirs )

def build(bld):
        bld.recurse( bld_subdirs )

def configure(config):
        config.recurse( bld_subdirs )

def program(ctx):
    ctx.recurse( 'isixrtos' )
