%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/rolling/.*$
%global __requires_exclude_from ^/opt/ros/rolling/.*$

Name:           ros-rolling-tf2-ros
Version:        0.40.1
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS tf2_ros package

License:        BSD
URL:            http://www.ros.org/wiki/tf2_ros
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-rolling-builtin-interfaces
Requires:       ros-rolling-geometry-msgs
Requires:       ros-rolling-message-filters
Requires:       ros-rolling-rcl-interfaces
Requires:       ros-rolling-rclcpp
Requires:       ros-rolling-rclcpp-action
Requires:       ros-rolling-rclcpp-components
Requires:       ros-rolling-tf2
Requires:       ros-rolling-tf2-msgs
Requires:       ros-rolling-ros-workspace
BuildRequires:  ros-rolling-ament-cmake
BuildRequires:  ros-rolling-builtin-interfaces
BuildRequires:  ros-rolling-geometry-msgs
BuildRequires:  ros-rolling-message-filters
BuildRequires:  ros-rolling-rcl-interfaces
BuildRequires:  ros-rolling-rclcpp
BuildRequires:  ros-rolling-rclcpp-action
BuildRequires:  ros-rolling-rclcpp-components
BuildRequires:  ros-rolling-tf2
BuildRequires:  ros-rolling-tf2-msgs
BuildRequires:  ros-rolling-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-rolling-ament-cmake-gtest
BuildRequires:  ros-rolling-ament-lint-auto
BuildRequires:  ros-rolling-ament-lint-common
BuildRequires:  ros-rolling-rosgraph-msgs
%endif

%description
This package contains the C++ ROS bindings for the tf2 library

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
* Wed Jan 15 2025 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.40.1-1
- Autogenerated by Bloom

* Fri Dec 20 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.40.0-1
- Autogenerated by Bloom

* Mon Nov 25 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.39.3-1
- Autogenerated by Bloom

* Wed Nov 20 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.39.2-1
- Autogenerated by Bloom

* Tue Oct 15 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.39.1-1
- Autogenerated by Bloom

* Thu Oct 03 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.39.0-1
- Autogenerated by Bloom

* Fri Jul 19 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.38.2-1
- Autogenerated by Bloom

* Tue Jul 09 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.38.1-1
- Autogenerated by Bloom

* Mon Jun 17 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.38.0-1
- Autogenerated by Bloom

* Wed May 29 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.37.1-1
- Autogenerated by Bloom

* Fri Apr 26 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.37.0-1
- Autogenerated by Bloom

* Wed Apr 10 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.2-1
- Autogenerated by Bloom

* Thu Mar 28 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.1-1
- Autogenerated by Bloom

* Wed Mar 06 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.0-2
- Autogenerated by Bloom

