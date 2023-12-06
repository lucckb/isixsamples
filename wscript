#!/usr/bin/env python

import sys
sys.path.insert(0, 'isixrtos/isixwaf')

# Current build directory
top = '.'

# Optput directory
out = 'build'

#Default board for samples
_def_board = 'stm32f469i_disco'


# Boards list
_boards = [
    'bf700',
    'zl41arm',
    'stm32f411e_disco',
    'stm32f469i_disco'
]

# Board to cpu map
_board_cpu = {
    'bf700' : 'stm32f407zet6',
    'zl41arm'   : 'stm32f417vgt6',
    'stm32f411e_disco' : 'stm32f411vet6',
    'stm32f469i_disco' : 'stm32f469nih6'
}


#On options selection
def options(opt):
    opt.recurse( 'isixrtos' )
    opt.add_option( '--board', action='store',
            choices=_boards, default=_def_board,
            help='Select target board for samples' )

#On configure
def configure(cfg):
    if not cfg.options.cpu:
        brd =  _board_cpu[cfg.options.board]
        cfg.msg('Cpu type not explicitly defined',
                'selected by board type %s'%brd )
        cfg.options.cpu = brd
    cfg.recurse( 'isixrtos' )
    cfg.env.BOARD_TYPE = cfg.options.board
    cfg.recurse( cfg.env.BOARD_TYPE )
    cfg.load('clang_compilation_database')

#On  build
def build(bld):
    bld.recurse( 'isixrtos' )
    bld.recurse( bld.env.BOARD_TYPE )

#Program target
def program(ctx):
    ctx.recurse( 'isixrtos' )

#Debug target
def ocddebug(ctx):
    ctx.recurse( 'isixrtos' )
