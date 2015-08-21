Name: v4l2stereo
Version: 0.1
Release: 1%{?dist}
Summary: Utility for stereoscopic vision
License: GPL3
URL: https://github.com/fuzzgun/v4l2stereo
Packager: Bob Mottram (4096 bits) <bob@robotics.uk.to>
Source0: http://yourdomainname.com/src/%{name}_%{version}.orig.tar.gz
Group: Utility/ConsoleOnly

Requires: libopencv-devel

BuildRequires: libopencv-devel

%description
Enables the generation of depth maps from stereo vision

%prep
%setup -q

%build
%configure
make %{?_smp_mflags}

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}
mkdir -p %{buildroot}/etc
mkdir -p %{buildroot}/etc/%{name}
mkdir -p %{buildroot}/usr
mkdir -p %{buildroot}/usr/bin
mkdir -p %{buildroot}/usr/share
mkdir -p %{buildroot}/usr/share/man
mkdir -p %{buildroot}/usr/share/man/man1
# Make install but to the RPM BUILDROOT directory
make install -B DESTDIR=%{buildroot} PREFIX=/usr

%files
%doc README.md LICENSE
%defattr(-,root,root,-)
%{_bindir}/*
%{_mandir}/man1/*

%changelog
* Wed Nov 20 2013 Bob Mottram (4096 bits) <bob@robotics.uk.to> - 0.1-1
- Initial release
