%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/rolling/.*$
%global __requires_exclude_from ^/opt/ros/rolling/.*$

Name:           ros-rolling-tf2-ros-py
Version:        0.21.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS tf2_ros_py package

License:        BSD
URL:            http://www.ros.org/wiki/tf2_ros
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-rolling-geometry-msgs
Requires:       ros-rolling-rclpy
Requires:       ros-rolling-std-msgs
Requires:       ros-rolling-tf2-msgs
Requires:       ros-rolling-tf2-py
Requires:       ros-rolling-ros-workspace
BuildRequires:  python%{python3_pkgversion}-devel
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  ros-rolling-ros-workspace
BuildRequires:  ros-rolling-sensor-msgs
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
This package contains the ROS Python bindings for the tf2 library

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%py3_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%py3_install -- --prefix "/opt/ros/rolling"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/rolling

%changelog
* Fri Jan 14 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.21.0-1
- Autogenerated by Bloom

* Fri Dec 17 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.20.0-1
- Autogenerated by Bloom

* Fri Oct 15 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.19.0-1
- Autogenerated by Bloom

* Tue Jun 01 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.18.0-1
- Autogenerated by Bloom

* Tue Apr 06 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.17.1-1
- Autogenerated by Bloom

* Thu Mar 25 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.17.0-2
- Autogenerated by Bloom

* Fri Mar 19 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.17.0-1
- Autogenerated by Bloom

* Thu Mar 11 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.16.0-2
- Autogenerated by Bloom

* Mon Mar 08 2021 Chris Lalancette <clalancette@openrobotics.org> - 0.16.0-1
- Autogenerated by Bloom

