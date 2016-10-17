#!/usr/bin/env python

'''
setup.py file for SWIG eHealth module
'''
from __future__ import print_function

import os
import shutil
from subprocess import check_call, CalledProcessError, STDOUT
import sys
import time
import urllib
import tarfile

import pip
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext as _build_ext


def _execute_process(args, cwd=None):
    if cwd is None:
        check_call(args, stderr=STDOUT)
    else:
        check_call(args, cwd=cwd, stderr=STDOUT)


def _download(url, file_prefix, file_sufix, compression_method='gz'):
    print('Downloading %s%s from %s' % (file_prefix, file_sufix, url))
    urllib.urlretrieve(url=url, filename='%s%s' % (file_prefix, file_sufix))
    print('Unpacking %s%s to %s' % (file_prefix, file_sufix, file_prefix))
    tarfile.open(name='%s%s' % (file_prefix, file_sufix),
                 mode='r:%s' % compression_method).extractall()
                              


def _configure_make_install(file_prefix):
    print('Configuring %s' % file_prefix)
    _execute_process(['./configure', '--prefix=/usr'],
                         cwd='%s' % file_prefix)
    print('Making %s' % file_prefix)
    _execute_process(['make'], cwd='%s' % file_prefix)
    print('Installing %s' % file_prefix)
    _execute_process(['make', 'install'], cwd='%s' % file_prefix)


def _download_configure_make_install(url, file_prefix, file_sufix,
                                     compression_method='gz'):
    try:
        _download(url, file_prefix, file_sufix, compression_method)
        _configure_make_install(file_prefix)
    finally:
        try:
            os.remove('%s%s' % (file_prefix, file_sufix))
            shutil.rmtree('%s' % file_prefix)
        except:
            pass


def dphys_swapfile(command):
    assert command in ('setup', 'swapon', 'swapoff')
    try:
        _execute_process(['dphys-swapfile', command])
    except CalledProcessError:# dphys-swapfile returncode is always != 0
        pass


def change_swapfile(increase=True):
    dphys_swapfile(command='swapoff')
    if increase:
        shutil.copy2('/etc/dphys-swapfile', '/etc/dphys-swapfile.bak')
        with open('/etc/dphys-swapfile', 'w') as swapfile_handle:
            swapfile_handle.write('CONF_SWAPFILE=/var/swap\n')
            swapfile_handle.write('CONF_SWAPSIZE=4096\n')
            swapfile_handle.write('CONF_MAXSWAP=4096\n')
    else:
        shutil.copy2('/etc/dphys-swapfile.bak', '/etc/dphys-swapfile')
    dphys_swapfile(command='setup')
    dphys_swapfile(command='swapon')


def register_tango_server():
    import PyTango

    dev_info = PyTango.DbDevInfo()
    instance_id = ''
    while instance_id == '':
        instance_id = raw_input("Provide this server's instance id: ").strip()
        
    dev_info.server = 'aouda/{}'.format(instance_id)
    print('Device server: {}'.format(dev_info.server))
    dev_info._class = "AoudaServer"
    system_id = raw_input("Provide this server's system id[C3]: ").strip()
    if system_id == '':
        system_id = 'C3'
    subsystem_id = raw_input("Provide this server's sub system id[aouda]: ").strip()
    if subsystem_id == '':
        subsystem_id = 'aouda'
    device_id = raw_input("Provide this server's device id[1]: ").strip()
    if device_id == '':
        device_id = '1'
    dev_info.name = ('{system_id}/{subsystem_id}/{device_id}'
                     .format(system_id=system_id, subsystem_id=subsystem_id,
                             device_id=device_id))
    print('Device name: {}'.format(dev_info.name))

    db = PyTango.Database()
    db.add_device(dev_info)


def _pre_build():
    # 1. install SWIG, libboost, libboost-dev, python2.7-dev, MDP, numpy and scipy
    print('Installing SWIG, libboost, libboost-dev, python2.7-dev, MMDP, '
          'numpy and scipy')
    _execute_process(['apt-get', 'install', '-y', 'swig',
                      'libboost-python1.49.0', 'libboost-python1.49-dev',
                      'python2.7-dev', 'python-numpy', 'python-scipy',
                      'python-mdp'])

    # 2. download, compile and install ZMQ
    if not os.path.isfile('/usr/lib/libzmq.so.3'):
        _download_configure_make_install(
                url='http://download.zeromq.org/zeromq-3.2.4.tar.gz',
                file_prefix='zeromq-3.2.4',
                file_sufix='.tar.gz'
                                        )
    else:
        print('ZMQ is already installed.')

    # 3. download, compile and install omniORB
    if not os.path.isfile('/usr/lib/libomniORB4.so.1.6'):
        _download_configure_make_install(
            url='http://sourceforge.net/projects/omniorb/files/omniORB/'
                'omniORB-4.1.6/omniORB-4.1.6.tar.bz2',
            file_prefix='omniORB-4.1.6',
            file_sufix='.tar.bz2',
            compression_method='bz2'
                                    )
    else:
        print('omniORB4 is already installed.')

    # 4. download, patch, compile and install omniNotify
    if False and not os.path.isfile('/usr/lib/libCOSNotify4.so.0.1'):
        try:
            _download_configure_make_install(
                    url='http://sourceforge.net/projects/omninotify/files/'
                        'omninotify/omniNotify%202.1/omniNotify-2.1.tar.gz',
                    file_prefix='omniNotify',
                    file_sufix='.tar.gz'
                                            )
            _download(url='https://bitbucket.org/italianmarssociety/eras/'
                          'downloads/omniNotify.patch',
                     file_prefix='omniNotify',
                     file_sufix='.patch')
            handle = Popen(['patch', '-p1'], cwd='omniNotify', stdin=PIPE)
            with open('omniNotify/omniNotify.patch') as patch_handle:
                handle.communicate(''.join(patch_handle.readlines()))
            handle.wait()
            if handle.returncode != 0:
                raise CalledProcessError(returncode=handle.returncode,
                                         cmd=str(args), output="")
            _configure_make_install(file_prefix='omniNotify')
        finally:
            try:
                os.remove('omniNotify.tar.gz')
                shutil.rmtree('omniNotify')
            except:
                pass
    else:
        print('omniNotify is already installed.')

    # 5. download, compile and install Tango8.1.2
    if not os.path.isfile('/usr/lib/libtango.so.8.1.2'):
        _download_configure_make_install(
                url='http://sourceforge.net/projects/tango-cs/files/'
                    'tango-8.1.2c.tar.gz',
                file_prefix='tango-8.1.2',
                file_sufix='.tar.gz'
                                        )
    else:
        print('Tango8.1.2 is already installed.')

    # 6. install ipython and pandas
    print('Installing ipython and pandas')
    pip.main(['install', '--upgrade', 'ipython[all]', 'pandas'])

    # 7. download, compile and install PyTango8.1.2
    try:
        import PyTango.utils
        pytango_exists = 'PyTango 8.1.2' in PyTango.utils.info().split('\n')[0]
    except ImportError:
        pytango_exists = False
    if not pytango_exists:
        # 7.1. increase swap size, otherwise the compilation will fail
        print('Increasing swap size')
        change_swapfile(increase=True)
        # 7.2. _download PyTango tarball
        try:
            _download(url='https://pypi.python.org/packages/source/P/PyTango/'
                          'PyTango-8.1.2.tar.gz',
                     file_prefix='PyTango-8.1.2',
                     file_sufix='.tar.gz')
            # 7.3. install PyTango, this takes a very long time
            print('Installing PyTango')
            _execute_process([sys.executable, 'setup.py', 'install'])
        finally:
            # 7.4. return swap configuration to original state
            print('Returning swap configuration to original state')
            try:
                change_swapfile(increase=False)
            except:
                pass
            try:
                os.remove('PyTango-8.1.2.tar.gz')
                shutil.rmtree('PyTango-8.1.2')
            except:
                pass
    else:
        print('PyTango 8.1.2 is already installed.')

    # 8. register tango server
    try:
        register_tango_server()
    except Exception as e:
        print('There was an error registering the Tango Server: {}'.format(e))

    # 9. install Oger
    try:
        __import__('Oger')
        oger_exists = True
    except ImportError:
        oger_exists = False
    if not oger_exists:
        try:
            _download(url='http://organic.elis.ugent.be/sites/'
                          'organic.elis.ugent.be/files/Oger-1.1.3.tar.gz',
                     file_prefix='Oger-1.1.3',
                     file_sufix='.tar.gz')
            print('Installing Oger')
            _execute_process([sys.executable, 'setup.py', 'install'],
                              cwd='Oger-1.1.3')
        finally:
            try:
                os.remove('Oger-1.1.3.tar.gz')
                shutil.rmtree('Oger-1.1.3')
            except:
                pass
    else:
        print('Oger is already installed.')


def _run_swig():
    _execute_process(['swig', '-python', 'aoudadaemon/eHealth.i'])


orig_run = _build_ext.run
class build_ext(_build_ext):
    def run(self):
        global i, orig_run
        _pre_build()
        _run_swig()
        orig_run(self)
        shutil.move('_ehealth.so', 'aoudadaemon/_ehealth.so')


eHealth_module = Extension('_ehealth',
                           sources=['aoudadaemon/eHealth.i',
                                    'aoudadaemon/arduPi.cpp',
                                    'aoudadaemon/eHealth.cpp'],
                           swig_opts=['-c++'],
                           include_dirs=['aoudadaemon'],
                           libraries=['pthread', 'rt']
                           )

setup(name='ehealth',
      version='0.1',
      author='Coocking Hacks & Mario Tambos',
      cmdclass={'build_ext': build_ext},
      ext_modules=[eHealth_module],
      package_data={'': ['*.cfg']},
      packages=find_packages(exclude='test'),
      zip_safe=False
      )
