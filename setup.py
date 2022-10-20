from setuptools import setup, find_packages
import platform

setup(
    name='pyHerkuleX',
    packages=['pyherkulex'],
    #package_dir={'': 'src'},
    install_requires=['pyserial'],
    author='Guenhael LE QUILLIEC',
    author_email='contact@guenhael.com',
    url='https://framagit.org/guenhael/pyherkulex',
    description='HerkuleX Python package',
    long_description=open('README.rst').read(),
    keywords="herkulex serial hardware RS232 communication protocol servo motor",
    license='GNU General Public License - 3.0-or-later',
    version='1.1.0',
)
