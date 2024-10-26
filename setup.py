from setuptools import setup, find_packages, Extension
from Cython.Build import cythonize
import numpy as np

def readme():
    with open('README.md', 'r', encoding='utf-8') as f:
        return f.read()

# Cython-модуль, который нужно скомпилировать
extensions = [
    Extension(
        "pointsim.cython_pid",
        ["pointsim/cython_pid.pyx"],
        include_dirs=[np.get_include()]
    ),
]

setup(
    name='pointsim',
    version='0.0.1',
    author='OnisOris',
    author_email='onisoris@yandex.ru',
    description='A module for simulating a point and controlling points.',
    long_description=readme(),
    long_description_content_type='text/markdown',
    url='https://github.com/OnisOris/pointsim',
    packages=find_packages(),
    ext_modules=cythonize(extensions),  # Компиляция Cython
    install_requires=['numpy', 'matplotlib', 'PyQt5', 'cython'],
    classifiers=[
        'Programming Language :: Python :: 3.12',
        'License :: OSI Approved :: GNU General Public License v2 (GPLv2)',
        'Operating System :: OS Independent'
    ],
    keywords='PID control point simulation',
    project_urls={
        'GitHub': 'https://github.com/OnisOris/pointsim'
    },
    python_requires='>=3.9'
)
