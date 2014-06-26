'''Wrapper for OVR_CAPI.h

First version generated with ctypesgen. Further edited for Python 3 compability and custom libOVR path.
Needs libovr.so in same directory as pyOVR file.
In case you want to switch where to look for the libovr.so library (like /usr/lib64), simply change LIBOVR_PATH accordingly

'''

__docformat__ =  'restructuredtext'

# Begin preamble

import ctypes, os, sys
from ctypes import *

# Set libovr path, modifiy for own needs
# Is set to look for libovr.so in same dir, as pyOVR.py wrapper
LIBOVR_PATH = os.path.dirname(os.path.abspath(__file__)) + '/libovr.so'
# Look in /usr/lib64
#LIBOVR_PATH = '/usr/lib64/libovr.so'

_int_types = (c_int16, c_int32)
if hasattr(ctypes, 'c_int64'):
    # Some builds of ctypes apparently do not have c_int64
    # defined; it's a pretty good bet that these builds do not
    # have 64-bit pointers.
    _int_types += (c_int64,)
for t in _int_types:
    if sizeof(t) == sizeof(c_size_t):
        c_ptrdiff_t = t
del t
del _int_types

class c_void(Structure):
    # c_void_p is a buggy return type, converting to int, so
    # POINTER(None) == c_void_p is actually written as
    # POINTER(c_void), so it can be treated as a real pointer.
    _fields_ = [('dummy', c_int)]

def POINTER(obj):
    p = ctypes.POINTER(obj)

    # Convert None to a real NULL pointer to work around bugs
    # in how ctypes handles None on 64-bit platforms
    if not isinstance(p.from_param, classmethod):
        def from_param(cls, x):
            if x is None:
                return cls()
            else:
                return x
        p.from_param = classmethod(from_param)

    return p

class UserString:
    def __init__(self, seq):
        if isinstance(seq, str):
            self.data = seq
        elif isinstance(seq, UserString):
            self.data = seq.data[:]
        else:
            self.data = str(seq)
    def __str__(self): return str(self.data)
    def __repr__(self): return repr(self.data)
    def __int__(self): return int(self.data)
    def __long__(self): return int(self.data)
    def __float__(self): return float(self.data)
    def __complex__(self): return complex(self.data)
    def __hash__(self): return hash(self.data)

    def __cmp__(self, string):
        if isinstance(string, UserString):
            return cmp(self.data, string.data)
        else:
            return cmp(self.data, string)
    def __contains__(self, char):
        return char in self.data

    def __len__(self): return len(self.data)
    def __getitem__(self, index): return self.__class__(self.data[index])
    def __getslice__(self, start, end):
        start = max(start, 0); end = max(end, 0)
        return self.__class__(self.data[start:end])

    def __add__(self, other):
        if isinstance(other, UserString):
            return self.__class__(self.data + other.data)
        elif isinstance(other, str):
            return self.__class__(self.data + other)
        else:
            return self.__class__(self.data + str(other))
    def __radd__(self, other):
        if isinstance(other, str):
            return self.__class__(other + self.data)
        else:
            return self.__class__(str(other) + self.data)
    def __mul__(self, n):
        return self.__class__(self.data*n)
    __rmul__ = __mul__
    def __mod__(self, args):
        return self.__class__(self.data % args)

    # the following methods are defined in alphabetical order:
    def capitalize(self): return self.__class__(self.data.capitalize())
    def center(self, width, *args):
        return self.__class__(self.data.center(width, *args))
    def count(self, sub, start=0, end=sys.maxsize):
        return self.data.count(sub, start, end)
    def decode(self, encoding=None, errors=None): # XXX improve this?
        if encoding:
            if errors:
                return self.__class__(self.data.decode(encoding, errors))
            else:
                return self.__class__(self.data.decode(encoding))
        else:
            return self.__class__(self.data.decode())
    def encode(self, encoding=None, errors=None): # XXX improve this?
        if encoding:
            if errors:
                return self.__class__(self.data.encode(encoding, errors))
            else:
                return self.__class__(self.data.encode(encoding))
        else:
            return self.__class__(self.data.encode())
    def endswith(self, suffix, start=0, end=sys.maxsize):
        return self.data.endswith(suffix, start, end)
    def expandtabs(self, tabsize=8):
        return self.__class__(self.data.expandtabs(tabsize))
    def find(self, sub, start=0, end=sys.maxsize):
        return self.data.find(sub, start, end)
    def index(self, sub, start=0, end=sys.maxsize):
        return self.data.index(sub, start, end)
    def isalpha(self): return self.data.isalpha()
    def isalnum(self): return self.data.isalnum()
    def isdecimal(self): return self.data.isdecimal()
    def isdigit(self): return self.data.isdigit()
    def islower(self): return self.data.islower()
    def isnumeric(self): return self.data.isnumeric()
    def isspace(self): return self.data.isspace()
    def istitle(self): return self.data.istitle()
    def isupper(self): return self.data.isupper()
    def join(self, seq): return self.data.join(seq)
    def ljust(self, width, *args):
        return self.__class__(self.data.ljust(width, *args))
    def lower(self): return self.__class__(self.data.lower())
    def lstrip(self, chars=None): return self.__class__(self.data.lstrip(chars))
    def partition(self, sep):
        return self.data.partition(sep)
    def replace(self, old, new, maxsplit=-1):
        return self.__class__(self.data.replace(old, new, maxsplit))
    def rfind(self, sub, start=0, end=sys.maxsize):
        return self.data.rfind(sub, start, end)
    def rindex(self, sub, start=0, end=sys.maxsize):
        return self.data.rindex(sub, start, end)
    def rjust(self, width, *args):
        return self.__class__(self.data.rjust(width, *args))
    def rpartition(self, sep):
        return self.data.rpartition(sep)
    def rstrip(self, chars=None): return self.__class__(self.data.rstrip(chars))
    def split(self, sep=None, maxsplit=-1):
        return self.data.split(sep, maxsplit)
    def rsplit(self, sep=None, maxsplit=-1):
        return self.data.rsplit(sep, maxsplit)
    def splitlines(self, keepends=0): return self.data.splitlines(keepends)
    def startswith(self, prefix, start=0, end=sys.maxsize):
        return self.data.startswith(prefix, start, end)
    def strip(self, chars=None): return self.__class__(self.data.strip(chars))
    def swapcase(self): return self.__class__(self.data.swapcase())
    def title(self): return self.__class__(self.data.title())
    def translate(self, *args):
        return self.__class__(self.data.translate(*args))
    def upper(self): return self.__class__(self.data.upper())
    def zfill(self, width): return self.__class__(self.data.zfill(width))

class MutableString(UserString):
    """mutable string objects

    Python strings are immutable objects.  This has the advantage, that
    strings may be used as dictionary keys.  If this property isn't needed
    and you insist on changing string values in place instead, you may cheat
    and use MutableString.

    But the purpose of this class is an educational one: to prevent
    people from inventing their own mutable string class derived
    from UserString and than forget thereby to remove (override) the
    __hash__ method inherited from UserString.  This would lead to
    errors that would be very hard to track down.

    A faster and better solution is to rewrite your program using lists."""
    def __init__(self, string=""):
        self.data = string
    def __hash__(self):
        raise TypeError("unhashable type (it is mutable)")
    def __setitem__(self, index, sub):
        if index < 0:
            index += len(self.data)
        if index < 0 or index >= len(self.data): raise IndexError
        self.data = self.data[:index] + sub + self.data[index+1:]
    def __delitem__(self, index):
        if index < 0:
            index += len(self.data)
        if index < 0 or index >= len(self.data): raise IndexError
        self.data = self.data[:index] + self.data[index+1:]
    def __setslice__(self, start, end, sub):
        start = max(start, 0); end = max(end, 0)
        if isinstance(sub, UserString):
            self.data = self.data[:start]+sub.data+self.data[end:]
        elif isinstance(sub, str):
            self.data = self.data[:start]+sub+self.data[end:]
        else:
            self.data =  self.data[:start]+str(sub)+self.data[end:]
    def __delslice__(self, start, end):
        start = max(start, 0); end = max(end, 0)
        self.data = self.data[:start] + self.data[end:]
    def immutable(self):
        return UserString(self.data)
    def __iadd__(self, other):
        if isinstance(other, UserString):
            self.data += other.data
        elif isinstance(other, str):
            self.data += other
        else:
            self.data += str(other)
        return self
    def __imul__(self, n):
        self.data *= n
        return self

class String(MutableString, Union):

    _fields_ = [('raw', POINTER(c_char)),
                ('data', c_char_p)]

    def __init__(self, obj=""):
        if isinstance(obj, (str, UserString)):
            self.data = str(obj)
        else:
            self.raw = obj

    def __len__(self):
        return self.data and len(self.data) or 0

    def from_param(cls, obj):
        # Convert None or 0
        if obj is None or obj == 0:
            return cls(POINTER(c_char)())

        # Convert from String
        elif isinstance(obj, String):
            return obj

        # Convert from str
        elif isinstance(obj, str):
            return cls(obj)

        # Convert from c_char_p
        elif isinstance(obj, c_char_p):
            return obj

        # Convert from POINTER(c_char)
        elif isinstance(obj, POINTER(c_char)):
            return obj

        # Convert from raw pointer
        elif isinstance(obj, int):
            return cls(cast(obj, POINTER(c_char)))

        # Convert from object
        else:
            return String.from_param(obj._as_parameter_)
    from_param = classmethod(from_param)

def ReturnString(obj, func=None, arguments=None):
    return String.from_param(obj)

# As of ctypes 1.0, ctypes does not support custom error-checking
# functions on callbacks, nor does it support custom datatypes on
# callbacks, so we must ensure that all callbacks return
# primitive datatypes.
#
# Non-primitive return values wrapped with UNCHECKED won't be
# typechecked, and will be converted to c_void_p.
def UNCHECKED(type):
    if (hasattr(type, "_type_") and isinstance(type._type_, str)
        and type._type_ != "P"):
        return type
    else:
        return c_void_p

# ctypes doesn't have direct support for variadic functions, so we have to write
# our own wrapper class
class _variadic_function(object):
    def __init__(self,func,restype,argtypes):
        self.func=func
        self.func.restype=restype
        self.argtypes=argtypes
    def _as_parameter_(self):
        # So we can pass this variadic function as a function pointer
        return self.func
    def __call__(self,*args):
        fixed_args=[]
        i=0
        for argtype in self.argtypes:
            # Typecheck what we can
            fixed_args.append(argtype.from_param(args[i]))
            i+=1
        return self.func(*fixed_args+list(args[i:]))

# End preamble

_libs = {}
_libdirs = []

# Begin loader

# ----------------------------------------------------------------------------
# Copyright (c) 2008 David James
# Copyright (c) 2006-2008 Alex Holkner
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of pyglet nor the names of its
#    contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

import os.path, re, sys, glob
import platform
import ctypes
import ctypes.util

def _environ_path(name):
    if name in os.environ:
        return os.environ[name].split(":")
    else:
        return []

class LibraryLoader(object):
    def __init__(self):
        self.other_dirs=[]

    def load_library(self,libname):
        """Given the name of a library, load it."""
        paths = self.getpaths(libname)

        for path in paths:
            if os.path.exists(path):
                return self.load(path)

        raise ImportError("%s not found." % libname)

    def load(self,path):
        """Given a path to a library, load it."""
        try:
            # Darwin requires dlopen to be called with mode RTLD_GLOBAL instead
            # of the default RTLD_LOCAL.  Without this, you end up with
            # libraries not being loadable, resulting in "Symbol not found"
            # errors
            if sys.platform == 'darwin':
                return ctypes.CDLL(path, ctypes.RTLD_GLOBAL)
            else:
                return ctypes.cdll.LoadLibrary(path)
        except OSError as e:
            raise ImportError(e)

    def getpaths(self,libname):
        """Return a list of paths where the library might be found."""
        if os.path.isabs(libname):
            yield libname
        else:
            # FIXME / TODO return '.' and os.path.dirname(__file__)
            for path in self.getplatformpaths(libname):
                yield path

            path = ctypes.util.find_library(libname)
            if path: yield path

    def getplatformpaths(self, libname):
        return []

# Darwin (Mac OS X)

class DarwinLibraryLoader(LibraryLoader):
    name_formats = ["lib%s.dylib", "lib%s.so", "lib%s.bundle", "%s.dylib",
                "%s.so", "%s.bundle", "%s"]

    def getplatformpaths(self,libname):
        if os.path.pathsep in libname:
            names = [libname]
        else:
            names = [format % libname for format in self.name_formats]

        for dir in self.getdirs(libname):
            for name in names:
                yield os.path.join(dir,name)

    def getdirs(self,libname):
        '''Implements the dylib search as specified in Apple documentation:

        http://developer.apple.com/documentation/DeveloperTools/Conceptual/
            DynamicLibraries/Articles/DynamicLibraryUsageGuidelines.html

        Before commencing the standard search, the method first checks
        the bundle's ``Frameworks`` directory if the application is running
        within a bundle (OS X .app).
        '''

        dyld_fallback_library_path = _environ_path("DYLD_FALLBACK_LIBRARY_PATH")
        if not dyld_fallback_library_path:
            dyld_fallback_library_path = [os.path.expanduser('~/lib'),
                                          '/usr/local/lib', '/usr/lib']

        dirs = []

        if '/' in libname:
            dirs.extend(_environ_path("DYLD_LIBRARY_PATH"))
        else:
            dirs.extend(_environ_path("LD_LIBRARY_PATH"))
            dirs.extend(_environ_path("DYLD_LIBRARY_PATH"))

        dirs.extend(self.other_dirs)
        dirs.append(".")
        dirs.append(os.path.dirname(__file__))

        if hasattr(sys, 'frozen') and sys.frozen == 'macosx_app':
            dirs.append(os.path.join(
                os.environ['RESOURCEPATH'],
                '..',
                'Frameworks'))

        dirs.extend(dyld_fallback_library_path)

        return dirs

# Posix

class PosixLibraryLoader(LibraryLoader):
    _ld_so_cache = None

    def _create_ld_so_cache(self):
        # Recreate search path followed by ld.so.  This is going to be
        # slow to build, and incorrect (ld.so uses ld.so.cache, which may
        # not be up-to-date).  Used only as fallback for distros without
        # /sbin/ldconfig.
        #
        # We assume the DT_RPATH and DT_RUNPATH binary sections are omitted.

        directories = []
        for name in ("LD_LIBRARY_PATH",
                     "SHLIB_PATH", # HPUX
                     "LIBPATH", # OS/2, AIX
                     "LIBRARY_PATH", # BE/OS
                    ):
            if name in os.environ:
                directories.extend(os.environ[name].split(os.pathsep))
        directories.extend(self.other_dirs)
        directories.append(".")
        directories.append(os.path.dirname(__file__))

        try: directories.extend([dir.strip() for dir in open('/etc/ld.so.conf')])
        except IOError: pass

        unix_lib_dirs_list = ['/lib', '/usr/lib', '/lib64', '/usr/lib64']
        if sys.platform.startswith('linux'):
            # Try and support multiarch work in Ubuntu
            # https://wiki.ubuntu.com/MultiarchSpec
            bitage = platform.architecture()[0]
            if bitage.startswith('32'):
                # Assume Intel/AMD x86 compat
                unix_lib_dirs_list += ['/lib/i386-linux-gnu', '/usr/lib/i386-linux-gnu']
            elif bitage.startswith('64'):
                # Assume Intel/AMD x86 compat
                unix_lib_dirs_list += ['/lib/x86_64-linux-gnu', '/usr/lib/x86_64-linux-gnu']
            else:
                # guess...
                unix_lib_dirs_list += glob.glob('/lib/*linux-gnu')
        directories.extend(unix_lib_dirs_list)

        cache = {}
        lib_re = re.compile(r'lib(.*)\.s[ol]')
        ext_re = re.compile(r'\.s[ol]$')
        for dir in directories:
            try:
                for path in glob.glob("%s/*.s[ol]*" % dir):
                    file = os.path.basename(path)

                    # Index by filename
                    if file not in cache:
                        cache[file] = path

                    # Index by library name
                    match = lib_re.match(file)
                    if match:
                        library = match.group(1)
                        if library not in cache:
                            cache[library] = path
            except OSError:
                pass

        self._ld_so_cache = cache

    def getplatformpaths(self, libname):
        if self._ld_so_cache is None:
            self._create_ld_so_cache()

        result = self._ld_so_cache.get(libname)
        if result: yield result

        path = ctypes.util.find_library(libname)
        if path: yield os.path.join("/lib",path)

# Windows

class _WindowsLibrary(object):
    def __init__(self, path):
        self.cdll = ctypes.cdll.LoadLibrary(path)
        self.windll = ctypes.windll.LoadLibrary(path)

    def __getattr__(self, name):
        try: return getattr(self.cdll,name)
        except AttributeError:
            try: return getattr(self.windll,name)
            except AttributeError:
                raise

class WindowsLibraryLoader(LibraryLoader):
    name_formats = ["%s.dll", "lib%s.dll", "%slib.dll"]

    def load_library(self, libname):
        try:
            result = LibraryLoader.load_library(self, libname)
        except ImportError:
            result = None
            if os.path.sep not in libname:
                for name in self.name_formats:
                    try:
                        result = getattr(ctypes.cdll, name % libname)
                        if result:
                            break
                    except WindowsError:
                        result = None
            if result is None:
                try:
                    result = getattr(ctypes.cdll, libname)
                except WindowsError:
                    result = None
            if result is None:
                raise ImportError("%s not found." % libname)
        return result

    def load(self, path):
        return _WindowsLibrary(path)

    def getplatformpaths(self, libname):
        if os.path.sep not in libname:
            for name in self.name_formats:
                dll_in_current_dir = os.path.abspath(name % libname)
                if os.path.exists(dll_in_current_dir):
                    yield dll_in_current_dir
                path = ctypes.util.find_library(name % libname)
                if path:
                    yield path

# Platform switching

# If your value of sys.platform does not appear in this dict, please contact
# the Ctypesgen maintainers.

loaderclass = {
    "darwin":   DarwinLibraryLoader,
    "cygwin":   WindowsLibraryLoader,
    "win32":    WindowsLibraryLoader
}

loader = loaderclass.get(sys.platform, PosixLibraryLoader)()

def add_library_search_dirs(other_dirs):
    loader.other_dirs = other_dirs

load_library = loader.load_library

del loaderclass

# End loader

add_library_search_dirs([])

# Begin libraries

_libs["libovr.so"] = load_library(LIBOVR_PATH)

# 1 libraries
# End libraries

# No modules

int8_t = c_char # /usr/include/stdint.h: 36

int16_t = c_int # /usr/include/stdint.h: 37

int32_t = c_int # /usr/include/stdint.h: 38

int64_t = c_long # /usr/include/stdint.h: 40

uint8_t = c_ubyte # /usr/include/stdint.h: 48

uint16_t = c_uint # /usr/include/stdint.h: 49

uint32_t = c_uint # /usr/include/stdint.h: 51

uint64_t = c_ulong # /usr/include/stdint.h: 55

int_least8_t = c_char # /usr/include/stdint.h: 65

int_least16_t = c_int # /usr/include/stdint.h: 66

int_least32_t = c_int # /usr/include/stdint.h: 67

int_least64_t = c_long # /usr/include/stdint.h: 69

uint_least8_t = c_ubyte # /usr/include/stdint.h: 76

uint_least16_t = c_uint # /usr/include/stdint.h: 77

uint_least32_t = c_uint # /usr/include/stdint.h: 78

uint_least64_t = c_ulong # /usr/include/stdint.h: 80

int_fast8_t = c_char # /usr/include/stdint.h: 90

int_fast16_t = c_long # /usr/include/stdint.h: 92

int_fast32_t = c_long # /usr/include/stdint.h: 93

int_fast64_t = c_long # /usr/include/stdint.h: 94

uint_fast8_t = c_ubyte # /usr/include/stdint.h: 103

uint_fast16_t = c_ulong # /usr/include/stdint.h: 105

uint_fast32_t = c_ulong # /usr/include/stdint.h: 106

uint_fast64_t = c_ulong # /usr/include/stdint.h: 107

intptr_t = c_long # /usr/include/stdint.h: 119

uintptr_t = c_ulong # /usr/include/stdint.h: 122

intmax_t = c_long # /usr/include/stdint.h: 134

uintmax_t = c_ulong # /usr/include/stdint.h: 135

ovrBool = c_char # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 31

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 51
class struct_ovrVector2i_(Structure):
    pass

struct_ovrVector2i_.__slots__ = [
    'x',
    'y',
]
struct_ovrVector2i_._fields_ = [
    ('x', c_int),
    ('y', c_int),
]

ovrVector2i = struct_ovrVector2i_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 51

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 55
class struct_ovrSizei_(Structure):
    pass

struct_ovrSizei_.__slots__ = [
    'w',
    'h',
]
struct_ovrSizei_._fields_ = [
    ('w', c_int),
    ('h', c_int),
]

ovrSizei = struct_ovrSizei_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 55

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 60
class struct_ovrRecti_(Structure):
    pass

struct_ovrRecti_.__slots__ = [
    'Pos',
    'Size',
]
struct_ovrRecti_._fields_ = [
    ('Pos', ovrVector2i),
    ('Size', ovrSizei),
]

ovrRecti = struct_ovrRecti_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 60

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 66
class struct_ovrQuatf_(Structure):
    pass

struct_ovrQuatf_.__slots__ = [
    'x',
    'y',
    'z',
    'w',
]
struct_ovrQuatf_._fields_ = [
    ('x', c_float),
    ('y', c_float),
    ('z', c_float),
    ('w', c_float),
]

ovrQuatf = struct_ovrQuatf_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 66

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 70
class struct_ovrVector2f_(Structure):
    pass

struct_ovrVector2f_.__slots__ = [
    'x',
    'y',
]
struct_ovrVector2f_._fields_ = [
    ('x', c_float),
    ('y', c_float),
]

ovrVector2f = struct_ovrVector2f_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 70

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 74
class struct_ovrVector3f_(Structure):
    pass

struct_ovrVector3f_.__slots__ = [
    'x',
    'y',
    'z',
]
struct_ovrVector3f_._fields_ = [
    ('x', c_float),
    ('y', c_float),
    ('z', c_float),
]

ovrVector3f = struct_ovrVector3f_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 74

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 78
class struct_ovrMatrix4f_(Structure):
    pass

struct_ovrMatrix4f_.__slots__ = [
    'M',
]
struct_ovrMatrix4f_._fields_ = [
    ('M', (c_float * 4) * 4),
]

ovrMatrix4f = struct_ovrMatrix4f_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 78

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 84
class struct_ovrPosef_(Structure):
    pass

struct_ovrPosef_.__slots__ = [
    'Orientation',
    'Position',
]
struct_ovrPosef_._fields_ = [
    ('Orientation', ovrQuatf),
    ('Position', ovrVector3f),
]

ovrPosef = struct_ovrPosef_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 84

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 95
class struct_ovrPoseStatef_(Structure):
    pass

struct_ovrPoseStatef_.__slots__ = [
    'Pose',
    'AngularVelocity',
    'LinearVelocity',
    'AngularAcceleration',
    'LinearAcceleration',
    'TimeInSeconds',
]
struct_ovrPoseStatef_._fields_ = [
    ('Pose', ovrPosef),
    ('AngularVelocity', ovrVector3f),
    ('LinearVelocity', ovrVector3f),
    ('AngularAcceleration', ovrVector3f),
    ('LinearAcceleration', ovrVector3f),
    ('TimeInSeconds', c_double),
]

ovrPoseStatef = struct_ovrPoseStatef_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 95

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 106
class struct_ovrFovPort_(Structure):
    pass

struct_ovrFovPort_.__slots__ = [
    'UpTan',
    'DownTan',
    'LeftTan',
    'RightTan',
]
struct_ovrFovPort_._fields_ = [
    ('UpTan', c_float),
    ('DownTan', c_float),
    ('LeftTan', c_float),
    ('RightTan', c_float),
]

ovrFovPort = struct_ovrFovPort_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 106

enum_anon_1 = c_int # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

ovrHmd_None = 0 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

ovrHmd_DK1 = 3 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

ovrHmd_DKHD = 4 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

ovrHmd_CrystalCoveProto = 5 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

ovrHmd_DK2 = 6 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

ovrHmd_Other = (ovrHmd_DK2 + 1) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

ovrHmdType = enum_anon_1 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 121

enum_anon_2 = c_int # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_Present = 1 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_Available = 2 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_LowPersistence = 128 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_LatencyTest = 256 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_DynamicPrediction = 512 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_NoVSync = 4096 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_NoRestore = 16384 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCap_Writable_Mask = 4992 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

ovrHmdCaps = enum_anon_2 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 151

enum_anon_3 = c_int # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 162

ovrSensorCap_Orientation = 16 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 162

ovrSensorCap_YawCorrection = 32 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 162

ovrSensorCap_Position = 64 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 162

ovrSensorCaps = enum_anon_3 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 162

enum_anon_4 = c_int # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 171

ovrDistortionCap_Chromatic = 1 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 171

ovrDistortionCap_TimeWarp = 2 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 171

ovrDistortionCap_Vignette = 8 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 171

ovrDistortionCaps = enum_anon_4 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 171

enum_anon_5 = c_int # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 182

ovrEye_Left = 0 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 182

ovrEye_Right = 1 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 182

ovrEye_Count = 2 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 182

ovrEyeType = enum_anon_5 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 182

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 186
class struct_ovrHmdStruct(Structure):
    pass

ovrHmd = POINTER(struct_ovrHmdStruct) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 186

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 228
class struct_ovrHmdDesc_(Structure):
    pass

struct_ovrHmdDesc_.__slots__ = [
    'Handle',
    'Type',
    'ProductName',
    'Manufacturer',
    'HmdCaps',
    'SensorCaps',
    'DistortionCaps',
    'Resolution',
    'WindowsPos',
    'DefaultEyeFov',
    'MaxEyeFov',
    'EyeRenderOrder',
    'DisplayDeviceName',
    'DisplayId',
]
struct_ovrHmdDesc_._fields_ = [
    ('Handle', ovrHmd),
    ('Type', ovrHmdType),
    ('ProductName', String),
    ('Manufacturer', String),
    ('HmdCaps', c_uint),
    ('SensorCaps', c_uint),
    ('DistortionCaps', c_uint),
    ('Resolution', ovrSizei),
    ('WindowsPos', ovrVector2i),
    ('DefaultEyeFov', ovrFovPort * ovrEye_Count),
    ('MaxEyeFov', ovrFovPort * ovrEye_Count),
    ('EyeRenderOrder', ovrEyeType * ovrEye_Count),
    ('DisplayDeviceName', String),
    ('DisplayId', c_int),
]

ovrHmdDesc = struct_ovrHmdDesc_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 228

enum_anon_6 = c_int # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 248

ovrStatus_OrientationTracked = 1 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 248

ovrStatus_PositionTracked = 2 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 248

ovrStatus_PositionConnected = 32 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 248

ovrStatus_HmdConnected = 128 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 248

ovrStatusBits = enum_anon_6 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 248

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 266
class struct_ovrSensorState_(Structure):
    pass

struct_ovrSensorState_.__slots__ = [
    'Predicted',
    'Recorded',
    'Temperature',
    'StatusFlags',
]
struct_ovrSensorState_._fields_ = [
    ('Predicted', ovrPoseStatef),
    ('Recorded', ovrPoseStatef),
    ('Temperature', c_float),
    ('StatusFlags', c_uint),
]

ovrSensorState = struct_ovrSensorState_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 266

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 277
class struct_ovrSensorDesc_(Structure):
    pass

struct_ovrSensorDesc_.__slots__ = [
    'VendorId',
    'ProductId',
    'SerialNumber',
]
struct_ovrSensorDesc_._fields_ = [
    ('VendorId', c_short),
    ('ProductId', c_short),
    ('SerialNumber', c_char * 24),
]

ovrSensorDesc = struct_ovrSensorDesc_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 277

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 309
class struct_ovrFrameTiming_(Structure):
    pass

struct_ovrFrameTiming_.__slots__ = [
    'DeltaSeconds',
    'ThisFrameSeconds',
    'TimewarpPointSeconds',
    'NextFrameSeconds',
    'ScanoutMidpointSeconds',
    'EyeScanoutSeconds',
]
struct_ovrFrameTiming_._fields_ = [
    ('DeltaSeconds', c_float),
    ('ThisFrameSeconds', c_double),
    ('TimewarpPointSeconds', c_double),
    ('NextFrameSeconds', c_double),
    ('ScanoutMidpointSeconds', c_double),
    ('EyeScanoutSeconds', c_double * 2),
]

ovrFrameTiming = struct_ovrFrameTiming_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 309

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 326
class struct_ovrEyeRenderDesc_(Structure):
    pass

struct_ovrEyeRenderDesc_.__slots__ = [
    'Eye',
    'Fov',
    'DistortedViewport',
    'PixelsPerTanAngleAtCenter',
    'ViewAdjust',
]
struct_ovrEyeRenderDesc_._fields_ = [
    ('Eye', ovrEyeType),
    ('Fov', ovrFovPort),
    ('DistortedViewport', ovrRecti),
    ('PixelsPerTanAngleAtCenter', ovrVector2f),
    ('ViewAdjust', ovrVector3f),
]

ovrEyeRenderDesc = struct_ovrEyeRenderDesc_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 326

enum_anon_7 = c_int # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPI_None = 0 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPI_OpenGL = (ovrRenderAPI_None + 1) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPI_Android_GLES = (ovrRenderAPI_OpenGL + 1) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPI_D3D9 = (ovrRenderAPI_Android_GLES + 1) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPI_D3D10 = (ovrRenderAPI_D3D9 + 1) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPI_D3D11 = (ovrRenderAPI_D3D10 + 1) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPI_Count = (ovrRenderAPI_D3D11 + 1) # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

ovrRenderAPIType = enum_anon_7 # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 350

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 359
class struct_ovrRenderAPIConfigHeader_(Structure):
    pass

struct_ovrRenderAPIConfigHeader_.__slots__ = [
    'API',
    'RTSize',
    'Multisample',
]
struct_ovrRenderAPIConfigHeader_._fields_ = [
    ('API', ovrRenderAPIType),
    ('RTSize', ovrSizei),
    ('Multisample', c_int),
]

ovrRenderAPIConfigHeader = struct_ovrRenderAPIConfigHeader_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 359

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 365
class struct_ovrRenderAPIConfig_(Structure):
    pass

struct_ovrRenderAPIConfig_.__slots__ = [
    'Header',
    'PlatformData',
]
struct_ovrRenderAPIConfig_._fields_ = [
    ('Header', ovrRenderAPIConfigHeader),
    ('PlatformData', uintptr_t * 8),
]

ovrRenderAPIConfig = struct_ovrRenderAPIConfig_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 365

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 375
class struct_ovrTextureHeader_(Structure):
    pass

struct_ovrTextureHeader_.__slots__ = [
    'API',
    'TextureSize',
    'RenderViewport',
]
struct_ovrTextureHeader_._fields_ = [
    ('API', ovrRenderAPIType),
    ('TextureSize', ovrSizei),
    ('RenderViewport', ovrRecti),
]

ovrTextureHeader = struct_ovrTextureHeader_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 375

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 381
class struct_ovrTexture_(Structure):
    pass

struct_ovrTexture_.__slots__ = [
    'Header',
    'PlatformData',
]
struct_ovrTexture_._fields_ = [
    ('Header', ovrTextureHeader),
    ('PlatformData', uintptr_t * 8),
]

ovrTexture = struct_ovrTexture_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 381

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 417
if hasattr(_libs['libovr.so'], 'ovr_Initialize'):
    ovr_Initialize = _libs['libovr.so'].ovr_Initialize
    ovr_Initialize.argtypes = []
    ovr_Initialize.restype = ovrBool

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 418
if hasattr(_libs['libovr.so'], 'ovr_Shutdown'):
    ovr_Shutdown = _libs['libovr.so'].ovr_Shutdown
    ovr_Shutdown.argtypes = []
    ovr_Shutdown.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 423
if hasattr(_libs['libovr.so'], 'ovrHmd_Detect'):
    ovrHmd_Detect = _libs['libovr.so'].ovrHmd_Detect
    ovrHmd_Detect.argtypes = []
    ovrHmd_Detect.restype = c_int

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 429
if hasattr(_libs['libovr.so'], 'ovrHmd_Create'):
    ovrHmd_Create = _libs['libovr.so'].ovrHmd_Create
    ovrHmd_Create.argtypes = [c_int]
    ovrHmd_Create.restype = ovrHmd

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 430
if hasattr(_libs['libovr.so'], 'ovrHmd_Destroy'):
    ovrHmd_Destroy = _libs['libovr.so'].ovrHmd_Destroy
    ovrHmd_Destroy.argtypes = [ovrHmd]
    ovrHmd_Destroy.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 434
if hasattr(_libs['libovr.so'], 'ovrHmd_CreateDebug'):
    ovrHmd_CreateDebug = _libs['libovr.so'].ovrHmd_CreateDebug
    ovrHmd_CreateDebug.argtypes = [ovrHmdType]
    ovrHmd_CreateDebug.restype = ovrHmd

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 440
if hasattr(_libs['libovr.so'], 'ovrHmd_GetLastError'):
    ovrHmd_GetLastError = _libs['libovr.so'].ovrHmd_GetLastError
    ovrHmd_GetLastError.argtypes = [ovrHmd]
    if sizeof(c_int) == sizeof(c_void_p):
        ovrHmd_GetLastError.restype = ReturnString
    else:
        ovrHmd_GetLastError.restype = String
        ovrHmd_GetLastError.errcheck = ReturnString

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 448
if hasattr(_libs['libovr.so'], 'ovrHmd_GetEnabledCaps'):
    ovrHmd_GetEnabledCaps = _libs['libovr.so'].ovrHmd_GetEnabledCaps
    ovrHmd_GetEnabledCaps.argtypes = [ovrHmd]
    ovrHmd_GetEnabledCaps.restype = c_uint

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 452
if hasattr(_libs['libovr.so'], 'ovrHmd_SetEnabledCaps'):
    ovrHmd_SetEnabledCaps = _libs['libovr.so'].ovrHmd_SetEnabledCaps
    ovrHmd_SetEnabledCaps.argtypes = [ovrHmd, c_uint]
    ovrHmd_SetEnabledCaps.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 468
if hasattr(_libs['libovr.so'], 'ovrHmd_StartSensor'):
    ovrHmd_StartSensor = _libs['libovr.so'].ovrHmd_StartSensor
    ovrHmd_StartSensor.argtypes = [ovrHmd, c_uint, c_uint]
    ovrHmd_StartSensor.restype = ovrBool

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 471
if hasattr(_libs['libovr.so'], 'ovrHmd_StopSensor'):
    ovrHmd_StopSensor = _libs['libovr.so'].ovrHmd_StopSensor
    ovrHmd_StopSensor.argtypes = [ovrHmd]
    ovrHmd_StopSensor.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 473
if hasattr(_libs['libovr.so'], 'ovrHmd_ResetSensor'):
    ovrHmd_ResetSensor = _libs['libovr.so'].ovrHmd_ResetSensor
    ovrHmd_ResetSensor.argtypes = [ovrHmd]
    ovrHmd_ResetSensor.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 480
if hasattr(_libs['libovr.so'], 'ovrHmd_GetSensorState'):
    ovrHmd_GetSensorState = _libs['libovr.so'].ovrHmd_GetSensorState
    ovrHmd_GetSensorState.argtypes = [ovrHmd, c_double]
    ovrHmd_GetSensorState.restype = ovrSensorState

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 484
if hasattr(_libs['libovr.so'], 'ovrHmd_GetSensorDesc'):
    ovrHmd_GetSensorDesc = _libs['libovr.so'].ovrHmd_GetSensorDesc
    ovrHmd_GetSensorDesc.argtypes = [ovrHmd, POINTER(ovrSensorDesc)]
    ovrHmd_GetSensorDesc.restype = ovrBool

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 491
if hasattr(_libs['libovr.so'], 'ovrHmd_GetDesc'):
    ovrHmd_GetDesc = _libs['libovr.so'].ovrHmd_GetDesc
    ovrHmd_GetDesc.argtypes = [ovrHmd, POINTER(ovrHmdDesc)]
    ovrHmd_GetDesc.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 498
if hasattr(_libs['libovr.so'], 'ovrHmd_GetFovTextureSize'):
    ovrHmd_GetFovTextureSize = _libs['libovr.so'].ovrHmd_GetFovTextureSize
    ovrHmd_GetFovTextureSize.argtypes = [ovrHmd, ovrEyeType, ovrFovPort, c_float]
    ovrHmd_GetFovTextureSize.restype = ovrSizei

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 540
if hasattr(_libs['libovr.so'], 'ovrHmd_ConfigureRendering'):
    ovrHmd_ConfigureRendering = _libs['libovr.so'].ovrHmd_ConfigureRendering
    ovrHmd_ConfigureRendering.argtypes = [ovrHmd, POINTER(ovrRenderAPIConfig), c_uint, ovrFovPort * 2, ovrEyeRenderDesc * 2]
    ovrHmd_ConfigureRendering.restype = ovrBool

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 551
if hasattr(_libs['libovr.so'], 'ovrHmd_BeginFrame'):
    ovrHmd_BeginFrame = _libs['libovr.so'].ovrHmd_BeginFrame
    ovrHmd_BeginFrame.argtypes = [ovrHmd, c_uint]
    ovrHmd_BeginFrame.restype = ovrFrameTiming

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 557
if hasattr(_libs['libovr.so'], 'ovrHmd_EndFrame'):
    ovrHmd_EndFrame = _libs['libovr.so'].ovrHmd_EndFrame
    ovrHmd_EndFrame.argtypes = [ovrHmd]
    ovrHmd_EndFrame.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 567
if hasattr(_libs['libovr.so'], 'ovrHmd_BeginEyeRender'):
    ovrHmd_BeginEyeRender = _libs['libovr.so'].ovrHmd_BeginEyeRender
    ovrHmd_BeginEyeRender.argtypes = [ovrHmd, ovrEyeType]
    ovrHmd_BeginEyeRender.restype = ovrPosef

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 575
if hasattr(_libs['libovr.so'], 'ovrHmd_EndEyeRender'):
    ovrHmd_EndEyeRender = _libs['libovr.so'].ovrHmd_EndEyeRender
    ovrHmd_EndEyeRender.argtypes = [ovrHmd, ovrEyeType, ovrPosef, POINTER(ovrTexture)]
    ovrHmd_EndEyeRender.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 602
if hasattr(_libs['libovr.so'], 'ovrHmd_GetRenderDesc'):
    ovrHmd_GetRenderDesc = _libs['libovr.so'].ovrHmd_GetRenderDesc
    ovrHmd_GetRenderDesc.argtypes = [ovrHmd, ovrEyeType, ovrFovPort]
    ovrHmd_GetRenderDesc.restype = ovrEyeRenderDesc

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 618
class struct_ovrDistortionVertex_(Structure):
    pass

struct_ovrDistortionVertex_.__slots__ = [
    'Pos',
    'TimeWarpFactor',
    'VignetteFactor',
    'TexR',
    'TexG',
    'TexB',
]
struct_ovrDistortionVertex_._fields_ = [
    ('Pos', ovrVector2f),
    ('TimeWarpFactor', c_float),
    ('VignetteFactor', c_float),
    ('TexR', ovrVector2f),
    ('TexG', ovrVector2f),
    ('TexB', ovrVector2f),
]

ovrDistortionVertex = struct_ovrDistortionVertex_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 618

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 628
class struct_ovrDistortionMesh_(Structure):
    pass

struct_ovrDistortionMesh_.__slots__ = [
    'pVertexData',
    'pIndexData',
    'VertexCount',
    'IndexCount',
]
struct_ovrDistortionMesh_._fields_ = [
    ('pVertexData', POINTER(ovrDistortionVertex)),
    ('pIndexData', POINTER(c_ushort)),
    ('VertexCount', c_uint),
    ('IndexCount', c_uint),
]

ovrDistortionMesh = struct_ovrDistortionMesh_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 628

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 638
if hasattr(_libs['libovr.so'], 'ovrHmd_CreateDistortionMesh'):
    ovrHmd_CreateDistortionMesh = _libs['libovr.so'].ovrHmd_CreateDistortionMesh
    ovrHmd_CreateDistortionMesh.argtypes = [ovrHmd, ovrEyeType, ovrFovPort, c_uint, POINTER(ovrDistortionMesh)]
    ovrHmd_CreateDistortionMesh.restype = ovrBool

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 645
if hasattr(_libs['libovr.so'], 'ovrHmd_DestroyDistortionMesh'):
    ovrHmd_DestroyDistortionMesh = _libs['libovr.so'].ovrHmd_DestroyDistortionMesh
    ovrHmd_DestroyDistortionMesh.argtypes = [POINTER(ovrDistortionMesh)]
    ovrHmd_DestroyDistortionMesh.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 649
if hasattr(_libs['libovr.so'], 'ovrHmd_GetRenderScaleAndOffset'):
    ovrHmd_GetRenderScaleAndOffset = _libs['libovr.so'].ovrHmd_GetRenderScaleAndOffset
    ovrHmd_GetRenderScaleAndOffset.argtypes = [ovrFovPort, ovrSizei, ovrRecti, ovrVector2f * 2]
    ovrHmd_GetRenderScaleAndOffset.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 656
if hasattr(_libs['libovr.so'], 'ovrHmd_GetFrameTiming'):
    ovrHmd_GetFrameTiming = _libs['libovr.so'].ovrHmd_GetFrameTiming
    ovrHmd_GetFrameTiming.argtypes = [ovrHmd, c_uint]
    ovrHmd_GetFrameTiming.restype = ovrFrameTiming

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 661
if hasattr(_libs['libovr.so'], 'ovrHmd_BeginFrameTiming'):
    ovrHmd_BeginFrameTiming = _libs['libovr.so'].ovrHmd_BeginFrameTiming
    ovrHmd_BeginFrameTiming.argtypes = [ovrHmd, c_uint]
    ovrHmd_BeginFrameTiming.restype = ovrFrameTiming

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 666
if hasattr(_libs['libovr.so'], 'ovrHmd_EndFrameTiming'):
    ovrHmd_EndFrameTiming = _libs['libovr.so'].ovrHmd_EndFrameTiming
    ovrHmd_EndFrameTiming.argtypes = [ovrHmd]
    ovrHmd_EndFrameTiming.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 671
if hasattr(_libs['libovr.so'], 'ovrHmd_ResetFrameTiming'):
    ovrHmd_ResetFrameTiming = _libs['libovr.so'].ovrHmd_ResetFrameTiming
    ovrHmd_ResetFrameTiming.argtypes = [ovrHmd, c_uint]
    ovrHmd_ResetFrameTiming.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 676
if hasattr(_libs['libovr.so'], 'ovrHmd_GetEyePose'):
    ovrHmd_GetEyePose = _libs['libovr.so'].ovrHmd_GetEyePose
    ovrHmd_GetEyePose.argtypes = [ovrHmd, ovrEyeType]
    ovrHmd_GetEyePose.restype = ovrPosef

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 683
if hasattr(_libs['libovr.so'], 'ovrHmd_GetEyeTimewarpMatrices'):
    ovrHmd_GetEyeTimewarpMatrices = _libs['libovr.so'].ovrHmd_GetEyeTimewarpMatrices
    ovrHmd_GetEyeTimewarpMatrices.argtypes = [ovrHmd, ovrEyeType, ovrPosef, ovrMatrix4f * 2]
    ovrHmd_GetEyeTimewarpMatrices.restype = None

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 692
if hasattr(_libs['libovr.so'], 'ovrMatrix4f_Projection'):
    ovrMatrix4f_Projection = _libs['libovr.so'].ovrMatrix4f_Projection
    ovrMatrix4f_Projection.argtypes = [ovrFovPort, c_float, c_float, ovrBool]
    ovrMatrix4f_Projection.restype = ovrMatrix4f

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 698
if hasattr(_libs['libovr.so'], 'ovrMatrix4f_OrthoSubProjection'):
    ovrMatrix4f_OrthoSubProjection = _libs['libovr.so'].ovrMatrix4f_OrthoSubProjection
    ovrMatrix4f_OrthoSubProjection.argtypes = [ovrMatrix4f, ovrVector2f, c_float, c_float]
    ovrMatrix4f_OrthoSubProjection.restype = ovrMatrix4f

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 703
if hasattr(_libs['libovr.so'], 'ovr_GetTimeInSeconds'):
    ovr_GetTimeInSeconds = _libs['libovr.so'].ovr_GetTimeInSeconds
    ovr_GetTimeInSeconds.argtypes = []
    ovr_GetTimeInSeconds.restype = c_double

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 706
if hasattr(_libs['libovr.so'], 'ovr_WaitTillTime'):
    ovr_WaitTillTime = _libs['libovr.so'].ovr_WaitTillTime
    ovr_WaitTillTime.argtypes = [c_double]
    ovr_WaitTillTime.restype = c_double

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 715
for _lib in _libs.values():
    if not hasattr(_lib, 'ovrHmd_ProcessLatencyTest'):
        continue
    ovrHmd_ProcessLatencyTest = _lib.ovrHmd_ProcessLatencyTest
    ovrHmd_ProcessLatencyTest.argtypes = [ovrHmd, c_ubyte * 3]
    ovrHmd_ProcessLatencyTest.restype = ovrBool
    break

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 719
if hasattr(_libs['libovr.so'], 'ovrHmd_GetLatencyTestResult'):
    ovrHmd_GetLatencyTestResult = _libs['libovr.so'].ovrHmd_GetLatencyTestResult
    ovrHmd_GetLatencyTestResult.argtypes = [ovrHmd]
    if sizeof(c_int) == sizeof(c_void_p):
        ovrHmd_GetLatencyTestResult.restype = ReturnString
    else:
        ovrHmd_GetLatencyTestResult.restype = String
        ovrHmd_GetLatencyTestResult.errcheck = ReturnString

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 723
if hasattr(_libs['libovr.so'], 'ovrHmd_GetMeasuredLatencyTest2'):
    ovrHmd_GetMeasuredLatencyTest2 = _libs['libovr.so'].ovrHmd_GetMeasuredLatencyTest2
    ovrHmd_GetMeasuredLatencyTest2.argtypes = [ovrHmd]
    ovrHmd_GetMeasuredLatencyTest2.restype = c_double

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 759
if hasattr(_libs['libovr.so'], 'ovrHmd_GetFloat'):
    ovrHmd_GetFloat = _libs['libovr.so'].ovrHmd_GetFloat
    ovrHmd_GetFloat.argtypes = [ovrHmd, String, c_float]
    ovrHmd_GetFloat.restype = c_float

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 762
if hasattr(_libs['libovr.so'], 'ovrHmd_SetFloat'):
    ovrHmd_SetFloat = _libs['libovr.so'].ovrHmd_SetFloat
    ovrHmd_SetFloat.argtypes = [ovrHmd, String, c_float]
    ovrHmd_SetFloat.restype = ovrBool

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 767
if hasattr(_libs['libovr.so'], 'ovrHmd_GetFloatArray'):
    ovrHmd_GetFloatArray = _libs['libovr.so'].ovrHmd_GetFloatArray
    ovrHmd_GetFloatArray.argtypes = [ovrHmd, String, POINTER(c_float), c_uint]
    ovrHmd_GetFloatArray.restype = c_uint

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 771
if hasattr(_libs['libovr.so'], 'ovrHmd_SetFloatArray'):
    ovrHmd_SetFloatArray = _libs['libovr.so'].ovrHmd_SetFloatArray
    ovrHmd_SetFloatArray.argtypes = [ovrHmd, String, POINTER(c_float), c_uint]
    ovrHmd_SetFloatArray.restype = ovrBool

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 777
if hasattr(_libs['libovr.so'], 'ovrHmd_GetString'):
    ovrHmd_GetString = _libs['libovr.so'].ovrHmd_GetString
    ovrHmd_GetString.argtypes = [ovrHmd, String, String]
    if sizeof(c_int) == sizeof(c_void_p):
        ovrHmd_GetString.restype = ReturnString
    else:
        ovrHmd_GetString.restype = String
        ovrHmd_GetString.errcheck = ReturnString

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 782
if hasattr(_libs['libovr.so'], 'ovrHmd_GetArraySize'):
    ovrHmd_GetArraySize = _libs['libovr.so'].ovrHmd_GetArraySize
    ovrHmd_GetArraySize.argtypes = [ovrHmd, String]
    ovrHmd_GetArraySize.restype = c_uint

# /tmp/tmptIoTl_.h: 1
try:
    __STDC__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __STDC_HOSTED__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GNUC__ = 4
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GNUC_MINOR__ = 8
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GNUC_PATCHLEVEL__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __VERSION__ = '4.8.1'
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_RELAXED = 0
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_SEQ_CST = 5
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_ACQUIRE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_RELEASE = 3
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_ACQ_REL = 4
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_CONSUME = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FINITE_MATH_ONLY__ = 0
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    _LP64 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LP64__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_INT__ = 4
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_LONG__ = 8
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_LONG_LONG__ = 8
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_SHORT__ = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_FLOAT__ = 4
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_DOUBLE__ = 8
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_LONG_DOUBLE__ = 16
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_SIZE_T__ = 8
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __CHAR_BIT__ = 8
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __BIGGEST_ALIGNMENT__ = 16
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ORDER_LITTLE_ENDIAN__ = 1234
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ORDER_BIG_ENDIAN__ = 4321
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ORDER_PDP_ENDIAN__ = 3412
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __BYTE_ORDER__ = __ORDER_LITTLE_ENDIAN__
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLOAT_WORD_ORDER__ = __ORDER_LITTLE_ENDIAN__
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_POINTER__ = 8
except:
    pass

__SIZE_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

__PTRDIFF_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__WCHAR_TYPE__ = c_int # /tmp/tmptIoTl_.h: 1

__WINT_TYPE__ = c_uint # /tmp/tmptIoTl_.h: 1

__INTMAX_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__UINTMAX_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

__CHAR16_TYPE__ = c_uint # /tmp/tmptIoTl_.h: 1

__CHAR32_TYPE__ = c_uint # /tmp/tmptIoTl_.h: 1

__SIG_ATOMIC_TYPE__ = c_int # /tmp/tmptIoTl_.h: 1

__INT8_TYPE__ = c_char # /tmp/tmptIoTl_.h: 1

__INT16_TYPE__ = c_int # /tmp/tmptIoTl_.h: 1

__INT32_TYPE__ = c_int # /tmp/tmptIoTl_.h: 1

__INT64_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__UINT8_TYPE__ = c_ubyte # /tmp/tmptIoTl_.h: 1

__UINT16_TYPE__ = c_uint # /tmp/tmptIoTl_.h: 1

__UINT32_TYPE__ = c_uint # /tmp/tmptIoTl_.h: 1

__UINT64_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

__INT_LEAST8_TYPE__ = c_char # /tmp/tmptIoTl_.h: 1

__INT_LEAST16_TYPE__ = c_int # /tmp/tmptIoTl_.h: 1

__INT_LEAST32_TYPE__ = c_int # /tmp/tmptIoTl_.h: 1

__INT_LEAST64_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__UINT_LEAST8_TYPE__ = c_ubyte # /tmp/tmptIoTl_.h: 1

__UINT_LEAST16_TYPE__ = c_uint # /tmp/tmptIoTl_.h: 1

__UINT_LEAST32_TYPE__ = c_uint # /tmp/tmptIoTl_.h: 1

__UINT_LEAST64_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

__INT_FAST8_TYPE__ = c_char # /tmp/tmptIoTl_.h: 1

__INT_FAST16_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__INT_FAST32_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__INT_FAST64_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__UINT_FAST8_TYPE__ = c_ubyte # /tmp/tmptIoTl_.h: 1

__UINT_FAST16_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

__UINT_FAST32_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

__UINT_FAST64_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

__INTPTR_TYPE__ = c_long # /tmp/tmptIoTl_.h: 1

__UINTPTR_TYPE__ = c_ulong # /tmp/tmptIoTl_.h: 1

# /tmp/tmptIoTl_.h: 1
try:
    __GXX_ABI_VERSION = 1002
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SCHAR_MAX__ = 127
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SHRT_MAX__ = 32767
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT_MAX__ = 2147483647
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LONG_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LONG_LONG_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __WCHAR_MAX__ = 2147483647
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __WCHAR_MIN__ = ((-__WCHAR_MAX__) - 1)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __WINT_MAX__ = 4294967295
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __WINT_MIN__ = 0
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __PTRDIFF_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZE_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INTMAX_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINTMAX_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIG_ATOMIC_MAX__ = 2147483647
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIG_ATOMIC_MIN__ = ((-__SIG_ATOMIC_MAX__) - 1)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT8_MAX__ = 127
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT16_MAX__ = 32767
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT32_MAX__ = 2147483647
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT64_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT8_MAX__ = 255
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT16_MAX__ = 65535
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT32_MAX__ = 4294967295
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT64_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT_LEAST8_MAX__ = 127
except:
    pass

# /tmp/tmptIoTl_.h: 1
def __INT8_C(c):
    return c

# /tmp/tmptIoTl_.h: 1
try:
    __INT_LEAST16_MAX__ = 32767
except:
    pass

# /tmp/tmptIoTl_.h: 1
def __INT16_C(c):
    return c

# /tmp/tmptIoTl_.h: 1
try:
    __INT_LEAST32_MAX__ = 2147483647
except:
    pass

# /tmp/tmptIoTl_.h: 1
def __INT32_C(c):
    return c

# /tmp/tmptIoTl_.h: 1
try:
    __INT_LEAST64_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_LEAST8_MAX__ = 255
except:
    pass

# /tmp/tmptIoTl_.h: 1
def __UINT8_C(c):
    return c

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_LEAST16_MAX__ = 65535
except:
    pass

# /tmp/tmptIoTl_.h: 1
def __UINT16_C(c):
    return c

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_LEAST32_MAX__ = 4294967295
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_LEAST64_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT_FAST8_MAX__ = 127
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT_FAST16_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT_FAST32_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INT_FAST64_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_FAST8_MAX__ = 255
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_FAST16_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_FAST32_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINT_FAST64_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __INTPTR_MAX__ = 9223372036854775807
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __UINTPTR_MAX__ = 18446744073709551615
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_EVAL_METHOD__ = 0
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC_EVAL_METHOD__ = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_RADIX__ = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_MANT_DIG__ = 24
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_DIG__ = 6
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_MIN_EXP__ = (-125)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_MIN_10_EXP__ = (-37)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_MAX_EXP__ = 128
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_MAX_10_EXP__ = 38
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_DECIMAL_DIG__ = 9
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_MAX__ = 3.4028234663852886e+38
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_MIN__ = 1.1754943508222875e-38
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_EPSILON__ = 1.1920928955078125e-07
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_DENORM_MIN__ = 1.401298464324817e-45
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_HAS_DENORM__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_HAS_INFINITY__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FLT_HAS_QUIET_NAN__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_MANT_DIG__ = 53
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_DIG__ = 15
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_MIN_EXP__ = (-1021)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_MIN_10_EXP__ = (-307)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_MAX_EXP__ = 1024
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_MAX_10_EXP__ = 308
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_DECIMAL_DIG__ = 17
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_MAX__ = 1.7976931348623157e+308
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_MIN__ = 2.2250738585072014e-308
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_EPSILON__ = 2.220446049250313e-16
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_DENORM_MIN__ = 5e-324
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_HAS_DENORM__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_HAS_INFINITY__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DBL_HAS_QUIET_NAN__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_MANT_DIG__ = 64
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_DIG__ = 18
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_MIN_EXP__ = (-16381)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_MIN_10_EXP__ = (-4931)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_MAX_EXP__ = 16384
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_MAX_10_EXP__ = 4932
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DECIMAL_DIG__ = 21
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_MAX__ = float('inf')
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_MIN__ = 0.0
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_EPSILON__ = 1.0842021724855044e-19
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_DENORM_MIN__ = 0.0
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_HAS_DENORM__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_HAS_INFINITY__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __LDBL_HAS_QUIET_NAN__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC32_MANT_DIG__ = 7
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC32_MIN_EXP__ = (-94)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC32_MAX_EXP__ = 97
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC64_MANT_DIG__ = 16
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC64_MIN_EXP__ = (-382)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC64_MAX_EXP__ = 385
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC128_MANT_DIG__ = 34
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC128_MIN_EXP__ = (-6142)
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DEC128_MAX_EXP__ = 6145
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GNUC_GNU_INLINE__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __NO_INLINE__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_BOOL_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_CHAR_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_CHAR16_T_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_CHAR32_T_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_WCHAR_T_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_SHORT_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_INT_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_LONG_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_LLONG_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_TEST_AND_SET_TRUEVAL = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_ATOMIC_POINTER_LOCK_FREE = 2
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __GCC_HAVE_DWARF2_CFI_ASM = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __PRAGMA_REDEFINE_EXTNAME = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SSP__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_INT128__ = 16
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_WCHAR_T__ = 4
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_WINT_T__ = 4
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SIZEOF_PTRDIFF_T__ = 8
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __amd64 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __amd64__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __x86_64 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __x86_64__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_HLE_ACQUIRE = 65536
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ATOMIC_HLE_RELEASE = 131072
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __k8 = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __k8__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __code_model_small__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __MMX__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SSE__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SSE2__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __FXSR__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SSE_MATH__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __SSE2_MATH__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __gnu_linux__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __linux = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __linux__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    linux = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __unix = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __unix__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    unix = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __ELF__ = 1
except:
    pass

# /tmp/tmptIoTl_.h: 1
try:
    __DECIMAL_BID_FORMAT__ = 1
except:
    pass

__const = c_int # <command-line>: 4

# <command-line>: 7
try:
    CTYPESGEN = 1
except:
    pass

# /usr/include/stdc-predef.h: 19
try:
    _STDC_PREDEF_H = 1
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/predefs.h: 27
try:
    __STDC_IEC_559__ = 1
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/predefs.h: 28
try:
    __STDC_IEC_559_COMPLEX__ = 1
except:
    pass

# /usr/include/stdc-predef.h: 34
try:
    __STDC_ISO_10646__ = 201103
except:
    pass

# /usr/include/stdc-predef.h: 37
try:
    __STDC_NO_THREADS__ = 1
except:
    pass

# /usr/include/stdint.h: 23
try:
    _STDINT_H = 1
except:
    pass

# /usr/include/features.h: 19
try:
    _FEATURES_H = 1
except:
    pass

# /usr/include/features.h: 133
try:
    __USE_ANSI = 1
except:
    pass

# /usr/include/features.h: 146
def __GNUC_PREREQ(maj, min):
    return 0

# /usr/include/features.h: 188
try:
    _BSD_SOURCE = 1
except:
    pass

# /usr/include/features.h: 189
try:
    _SVID_SOURCE = 1
except:
    pass

# /usr/include/features.h: 223
try:
    _POSIX_SOURCE = 1
except:
    pass

# /usr/include/features.h: 231
try:
    _POSIX_C_SOURCE = 200809
except:
    pass

# /usr/include/features.h: 233
try:
    __USE_POSIX_IMPLICITLY = 1
except:
    pass

# /usr/include/features.h: 237
try:
    __USE_POSIX = 1
except:
    pass

# /usr/include/features.h: 241
try:
    __USE_POSIX2 = 1
except:
    pass

# /usr/include/features.h: 245
try:
    __USE_POSIX199309 = 1
except:
    pass

# /usr/include/features.h: 249
try:
    __USE_POSIX199506 = 1
except:
    pass

# /usr/include/features.h: 253
try:
    __USE_XOPEN2K = 1
except:
    pass

# /usr/include/features.h: 255
try:
    __USE_ISOC95 = 1
except:
    pass

# /usr/include/features.h: 257
try:
    __USE_ISOC99 = 1
except:
    pass

# /usr/include/features.h: 261
try:
    __USE_XOPEN2K8 = 1
except:
    pass

# /usr/include/features.h: 263
try:
    _ATFILE_SOURCE = 1
except:
    pass

# /usr/include/features.h: 305
try:
    __USE_MISC = 1
except:
    pass

# /usr/include/features.h: 309
try:
    __USE_BSD = 1
except:
    pass

# /usr/include/features.h: 313
try:
    __USE_SVID = 1
except:
    pass

# /usr/include/features.h: 317
try:
    __USE_ATFILE = 1
except:
    pass

# /usr/include/features.h: 336
try:
    __USE_FORTIFY_LEVEL = 0
except:
    pass

# /usr/include/features.h: 350
try:
    __GNU_LIBRARY__ = 6
except:
    pass

# /usr/include/features.h: 354
try:
    __GLIBC__ = 2
except:
    pass

# /usr/include/features.h: 355
try:
    __GLIBC_MINOR__ = 17
except:
    pass

# /usr/include/features.h: 357
def __GLIBC_PREREQ(maj, min):
    return (((__GLIBC__ << 16) + __GLIBC_MINOR__) >= ((maj << 16) + min))

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 20
try:
    _SYS_CDEFS_H = 1
except:
    pass

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 77
def __NTH(fct):
    return fct

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 83
def __P(args):
    return args

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 84
def __PMT(args):
    return args

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 90
def __STRING(x):
    return x

__ptr_t = POINTER(None) # /usr/include/x86_64-linux-gnu/sys/cdefs.h: 93

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 382
def __glibc_unlikely(cond):
    return cond

# /usr/include/x86_64-linux-gnu/bits/wordsize.h: 4
try:
    __WORDSIZE = 64
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/wordsize.h: 10
try:
    __WORDSIZE_TIME64_COMPAT32 = 1
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/wordsize.h: 12
try:
    __SYSCALL_WORDSIZE = 64
except:
    pass

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 407
def __LDBL_REDIR1(name, proto, alias):
    return (name + proto)

# /usr/include/x86_64-linux-gnu/sys/cdefs.h: 408
def __LDBL_REDIR(name, proto):
    return (name + proto)

# /usr/include/x86_64-linux-gnu/bits/wchar.h: 20
try:
    _BITS_WCHAR_H = 1
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/wchar.h: 34
try:
    __WCHAR_MAX = __WCHAR_MAX__
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/wchar.h: 42
try:
    __WCHAR_MIN = __WCHAR_MIN__
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/wordsize.h: 4
try:
    __WORDSIZE = 64
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/wordsize.h: 10
try:
    __WORDSIZE_TIME64_COMPAT32 = 1
except:
    pass

# /usr/include/x86_64-linux-gnu/bits/wordsize.h: 12
try:
    __SYSCALL_WORDSIZE = 64
except:
    pass

# /usr/include/stdint.h: 159
try:
    INT8_MIN = (-128)
except:
    pass

# /usr/include/stdint.h: 160
try:
    INT16_MIN = ((-32767) - 1)
except:
    pass

# /usr/include/stdint.h: 161
try:
    INT32_MIN = ((-2147483647) - 1)
except:
    pass

# /usr/include/stdint.h: 164
try:
    INT8_MAX = 127
except:
    pass

# /usr/include/stdint.h: 165
try:
    INT16_MAX = 32767
except:
    pass

# /usr/include/stdint.h: 166
try:
    INT32_MAX = 2147483647
except:
    pass

# /usr/include/stdint.h: 170
try:
    UINT8_MAX = 255
except:
    pass

# /usr/include/stdint.h: 171
try:
    UINT16_MAX = 65535
except:
    pass

# /usr/include/stdint.h: 172
try:
    UINT32_MAX = 4294967295
except:
    pass

# /usr/include/stdint.h: 177
try:
    INT_LEAST8_MIN = (-128)
except:
    pass

# /usr/include/stdint.h: 178
try:
    INT_LEAST16_MIN = ((-32767) - 1)
except:
    pass

# /usr/include/stdint.h: 179
try:
    INT_LEAST32_MIN = ((-2147483647) - 1)
except:
    pass

# /usr/include/stdint.h: 182
try:
    INT_LEAST8_MAX = 127
except:
    pass

# /usr/include/stdint.h: 183
try:
    INT_LEAST16_MAX = 32767
except:
    pass

# /usr/include/stdint.h: 184
try:
    INT_LEAST32_MAX = 2147483647
except:
    pass

# /usr/include/stdint.h: 188
try:
    UINT_LEAST8_MAX = 255
except:
    pass

# /usr/include/stdint.h: 189
try:
    UINT_LEAST16_MAX = 65535
except:
    pass

# /usr/include/stdint.h: 190
try:
    UINT_LEAST32_MAX = 4294967295
except:
    pass

# /usr/include/stdint.h: 195
try:
    INT_FAST8_MIN = (-128)
except:
    pass

# /usr/include/stdint.h: 197
try:
    INT_FAST16_MIN = ((-9223372036854775807) - 1)
except:
    pass

# /usr/include/stdint.h: 198
try:
    INT_FAST32_MIN = ((-9223372036854775807) - 1)
except:
    pass

# /usr/include/stdint.h: 205
try:
    INT_FAST8_MAX = 127
except:
    pass

# /usr/include/stdint.h: 207
try:
    INT_FAST16_MAX = 9223372036854775807
except:
    pass

# /usr/include/stdint.h: 208
try:
    INT_FAST32_MAX = 9223372036854775807
except:
    pass

# /usr/include/stdint.h: 216
try:
    UINT_FAST8_MAX = 255
except:
    pass

# /usr/include/stdint.h: 218
try:
    UINT_FAST16_MAX = 18446744073709551615
except:
    pass

# /usr/include/stdint.h: 219
try:
    UINT_FAST32_MAX = 18446744073709551615
except:
    pass

# /usr/include/stdint.h: 229
try:
    INTPTR_MIN = ((-9223372036854775807) - 1)
except:
    pass

# /usr/include/stdint.h: 230
try:
    INTPTR_MAX = 9223372036854775807
except:
    pass

# /usr/include/stdint.h: 231
try:
    UINTPTR_MAX = 18446744073709551615
except:
    pass

# /usr/include/stdint.h: 252
try:
    PTRDIFF_MIN = ((-9223372036854775807) - 1)
except:
    pass

# /usr/include/stdint.h: 253
try:
    PTRDIFF_MAX = 9223372036854775807
except:
    pass

# /usr/include/stdint.h: 260
try:
    SIG_ATOMIC_MIN = ((-2147483647) - 1)
except:
    pass

# /usr/include/stdint.h: 261
try:
    SIG_ATOMIC_MAX = 2147483647
except:
    pass

# /usr/include/stdint.h: 265
try:
    SIZE_MAX = 18446744073709551615
except:
    pass

# /usr/include/stdint.h: 273
try:
    WCHAR_MIN = __WCHAR_MIN
except:
    pass

# /usr/include/stdint.h: 274
try:
    WCHAR_MAX = __WCHAR_MAX
except:
    pass

# /usr/include/stdint.h: 278
try:
    WINT_MIN = 0
except:
    pass

# /usr/include/stdint.h: 279
try:
    WINT_MAX = 4294967295
except:
    pass

# /usr/include/stdint.h: 289
def INT8_C(c):
    return c

# /usr/include/stdint.h: 290
def INT16_C(c):
    return c

# /usr/include/stdint.h: 291
def INT32_C(c):
    return c

# /usr/include/stdint.h: 299
def UINT8_C(c):
    return c

# /usr/include/stdint.h: 300
def UINT16_C(c):
    return c

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 739
try:
    OVR_KEY_USER = 'User'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 740
try:
    OVR_KEY_NAME = 'Name'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 741
try:
    OVR_KEY_GENDER = 'Gender'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 742
try:
    OVR_KEY_PLAYER_HEIGHT = 'PlayerHeight'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 743
try:
    OVR_KEY_EYE_HEIGHT = 'EyeHeight'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 744
try:
    OVR_KEY_IPD = 'IPD'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 745
try:
    OVR_KEY_NECK_TO_EYE_HORIZONTAL = 'NeckEyeHori'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 746
try:
    OVR_KEY_NECK_TO_EYE_VERTICAL = 'NeckEyeVert'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 748
try:
    OVR_DEFAULT_GENDER = 'Male'
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 749
try:
    OVR_DEFAULT_PLAYER_HEIGHT = 1.778
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 750
try:
    OVR_DEFAULT_EYE_HEIGHT = 1.675
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 751
try:
    OVR_DEFAULT_IPD = 0.064
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 752
try:
    OVR_DEFAULT_NECK_TO_EYE_HORIZONTAL = 0.12
except:
    pass

# /OculusSDK/LibOVR/Src/OVR_CAPI.h: 753
try:
    OVR_DEFAULT_NECK_TO_EYE_VERTICAL = 0.12
except:
    pass

ovrVector2i_ = struct_ovrVector2i_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 51

ovrSizei_ = struct_ovrSizei_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 55

ovrRecti_ = struct_ovrRecti_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 60

ovrQuatf_ = struct_ovrQuatf_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 66

ovrVector2f_ = struct_ovrVector2f_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 70

ovrVector3f_ = struct_ovrVector3f_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 74

ovrMatrix4f_ = struct_ovrMatrix4f_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 78

ovrPosef_ = struct_ovrPosef_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 84

ovrPoseStatef_ = struct_ovrPoseStatef_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 95

ovrFovPort_ = struct_ovrFovPort_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 106

ovrHmdStruct = struct_ovrHmdStruct # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 186

ovrHmdDesc_ = struct_ovrHmdDesc_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 228

ovrSensorState_ = struct_ovrSensorState_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 266

ovrSensorDesc_ = struct_ovrSensorDesc_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 277

ovrFrameTiming_ = struct_ovrFrameTiming_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 309

ovrEyeRenderDesc_ = struct_ovrEyeRenderDesc_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 326

ovrRenderAPIConfigHeader_ = struct_ovrRenderAPIConfigHeader_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 359

ovrRenderAPIConfig_ = struct_ovrRenderAPIConfig_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 365

ovrTextureHeader_ = struct_ovrTextureHeader_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 375

ovrTexture_ = struct_ovrTexture_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 381

ovrDistortionVertex_ = struct_ovrDistortionVertex_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 618

ovrDistortionMesh_ = struct_ovrDistortionMesh_ # /OculusSDK/LibOVR/Src/OVR_CAPI.h: 628

# No inserted files

