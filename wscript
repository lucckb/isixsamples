#!/usr/bin/env python

import sys
sys.path.insert(0, 'isixrtos/isixwaf')


top = '.'
out = 'build'

bld_subdirs = [
        'isixrtos',
        'isix_c_examples',
]

def options(opt):
        opt.recurse( bld_subdirs )

def build(bld):
        bld.recurse( bld_subdirs )

def configure(config):
        config.recurse( bld_subdirs )



