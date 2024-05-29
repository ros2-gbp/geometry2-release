%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-tf2-sensor-msgs
Version:        0.31.7
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS tf2_sensor_msgs package

License:        BSD
URL:            http://www.ros.org/wiki/tf2_ros
Source0:        %{name}-%{version}.tar.gz

Requires:       eigen3-devel
Requires:       python%{python3_pkgversion}-numpy
Requires:       ros-iron-eigen3-cmake-module
Requires:       ros-iron-geometry-msgs
Requires:       ros-iron-sensor-msgs
Requires:       ros-iron-sensor-msgs-py
Requires:       ros-iron-std-msgs
Requires:       ros-iron-tf2
Requires:       ros-iron-tf2-ros
Requires:       ros-iron-tf2-ros-py
Requires:       ros-iron-ros-workspace
BuildRequires:  eigen3-devel
BuildRequires:  ros-iron-ament-cmake
BuildRequires:  ros-iron-eigen3-cmake-module
BuildRequires:  ros-iron-geometry-msgs
BuildRequires:  ros-iron-python-cmake-module
BuildRequires:  ros-iron-sensor-msgs
BuildRequires:  ros-iron-tf2
BuildRequires:  ros-iron-tf2-ros
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-iron-ament-cmake-gtest
BuildRequires:  ros-iron-ament-cmake-pytest
BuildRequires:  ros-iron-ament-lint-auto
BuildRequires:  ros-iron-ament-lint-common
BuildRequires:  ros-iron-rclcpp
%endif

%description
Small lib to transform sensor_msgs with tf. Most notably, PointCloud2

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/iron" \
    -DAMENT_PREFIX_PATH="/opt/ros/iron" \
    -DCMAKE_PREFIX_PATH="/opt/ros/iron" \
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
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/iron

%changelog
* Wed May 29 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.7-1
- Autogenerated by Bloom

* Fri Apr 19 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.6-1
- Autogenerated by Bloom

* Fri Sep 08 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.5-1
- Autogenerated by Bloom

* Fri Jul 14 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.4-1
- Autogenerated by Bloom

* Thu May 11 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.3-1
- Autogenerated by Bloom

* Thu Apr 20 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.2-2
- Autogenerated by Bloom

* Thu Apr 13 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.2-1
- Autogenerated by Bloom

* Wed Apr 12 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.1-1
- Autogenerated by Bloom

* Tue Apr 11 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.31.0-1
- Autogenerated by Bloom

* Tue Mar 21 2023 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.30.0-3
- Autogenerated by Bloom

