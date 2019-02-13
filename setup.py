#!/usr/bin/env python
from __future__ import print_function
from distutils.command.build import build
from subprocess import call
import os
import shutil
import setuptools
import numpy as np

# NOTE: If _mesher.cpp does not exist, you must run
# cython --cplus -I./ext/third_party/zi_lib/ ./ext/third_party/mc/_mesher.pyx

# NOTE: If _mesher.cpp does not exist, you must run
# cython --cplus -I./ext/third_party/zi_lib/ -I./ext/third_party/draco/src/ -I./ext/third_party/draco_build/ ./ext/third_party/mc/_mesher.pyx

# NOTE: If fastremap.cpp does not exist, you must run
# cython --cplus ./ext/remap/fastremap.pyx

third_party_dir = './ext/third_party'

setuptools.setup(
    setup_requires=['pbr', 'numpy'],
    extras_require={
      ':python_version == "2.7"': ['futures'],
      ':python_version == "2.6"': ['futures'],
    },
    pbr=False,
    ext_modules=[
        setuptools.Extension(
            'igneous._mesher',
            sources=[ os.path.join(third_party_dir, name) for name in ('mc/_mesher.cpp','mc/cMesher.cpp') ],
            depends=[ os.path.join(third_party_dir, 'mc/cMesher.h')],
            language='c++',
            include_dirs=[ os.path.join(third_party_dir, name) for name in ('zi_lib/', 'mc/', 'draco/src/', 'draco_build/') ],
            # extra_objects=[
            #   os.path.join(third_party_dir, 'draco_build/lib/', name) for name in ('libdracoenc.a', 'libdraco.a', 'libdracodec.a')
            # ],
            extra_compile_args=[
              '-std=c++11','-O3'
            ], #don't use  '-fvisibility=hidden', python can't see init module
            extra_link_args=[
              '-l:./ext/third_party/draco_build/new_lib/libdracoenc.a', '-l:./ext/third_party/draco_build/new_lib/libdraco.a', '-l:./ext/third_party/draco_build/new_lib/libdracodec.a'
            ]),
        setuptools.Extension(
            'fastremap',
            sources=[ './ext/remap/fastremap.cpp' ],
            depends=[],
            language='c++',
            include_dirs=[np.get_include()],
            extra_compile_args=[
              '-std=c++11', '-O3'
            ])#, #don't use  '-fvisibility=hidden', python can't see init module
        # setuptools.Extension(
        #     'igneous._draco',
        #     sources=[ os.path.join(third_party_dir, name) for name in ('draco_cython/_draco.cpp','draco_helper/draco.cpp') ],
        #     depends=[ os.path.join(third_party_dir, 'draco_cython/draco.h') ],
        #     language='c++',
        #     include_dirs=[ os.path.join(third_party_dir, name) for name in ('draco/', 'draco_cython/') ],
        #     extra_compile_args=[
        #       '-std=c++11', '-O3'
        #     ]) #don't use  '-fvisibility=hidden', python can't see init module
    ],
)

