%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/rolling/.*$
%global __requires_exclude_from ^/opt/ros/rolling/.*$

Name:           ros-rolling-tf2-kdl
Version:        0.27.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS tf2_kdl package

License:        BSD
URL:            http://ros.org/wiki/tf2
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-rolling-builtin-interfaces
Requires:       ros-rolling-geometry-msgs
Requires:       ros-rolling-orocos-kdl-vendor
Requires:       ros-rolling-tf2
Requires:       ros-rolling-tf2-ros
Requires:       ros-rolling-tf2-ros-py
Requires:       ros-rolling-ros-workspace
BuildRequires:  ros-rolling-ament-cmake
BuildRequires:  ros-rolling-builtin-interfaces
BuildRequires:  ros-rolling-geometry-msgs
BuildRequires:  ros-rolling-orocos-kdl-vendor
BuildRequires:  ros-rolling-tf2
BuildRequires:  ros-rolling-tf2-ros
BuildRequires:  ros-rolling-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-rolling-ament-cmake-gtest
BuildRequires:  ros-rolling-rclcpp
BuildRequires:  ros-rolling-tf2-msgs
%endif

%description
KDL binding for tf2

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/rolling" \
    -DAMENT_PREFIX_PATH="/opt/ros/rolling" \
    -DCMAKE_PREFIX_PATH="/opt/ros/rolling" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/rolling

%changelog
* Tue Sep 13 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.27.0-1
- Autogenerated by Bloom

* Mon Aug 15 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.26.2-1
- Autogenerated by Bloom

* Wed Jun 29 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.26.1-1
- Autogenerated by Bloom

* Fri Apr 29 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.26.0-1
- Autogenerated by Bloom

* Tue Apr 05 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.25.0-1
- Autogenerated by Bloom

* Thu Mar 31 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.24.0-1
- Autogenerated by Bloom

* Mon Mar 28 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.23.0-1
- Autogenerated by Bloom

* Tue Mar 01 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.22.0-1
- Autogenerated by Bloom

* Tue Feb 08 2022 Chris Lalancette <clalancette@openrobotics.org> - 0.21.0-2
- Autogenerated by Bloom

