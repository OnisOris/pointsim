from setuptools import setup, find_packages


def readme():
    with open('README.md', 'r', encoding='utf-8') as f:
        return f.read()


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
    install_requires=['numpy', 'matplotlib', 'PyQt5'],
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
