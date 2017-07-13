project('SSM Logger', 'cpp', default_options: ['cpp_std=c++14'])

ssm_logger = executable('ssm_logger',
                        sources: 'src/main.cpp',
                        dependencies: [dependency('boost', modules: ['system', 'program_options']),
                                       dependency('ncurses')])
