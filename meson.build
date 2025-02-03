project('dm', 'c')
cc = meson.get_compiler('c')

png_dep = dependency('libpng')
math_dep = cc.find_library('m')

include_dir = include_directories('.')
executable('base',
  ['dm-base.c', 'tasks.c', 'tasks.h'],
  include_directories: include_dir,
  dependencies: [png_dep, math_dep]
)
