%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/jazzy/.*$
%global __requires_exclude_from ^/opt/ros/jazzy/.*$

Name:           ros-jazzy-examples-tf2-py
Version:        0.36.5
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS examples_tf2_py package

License:        Apache License 2.0
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-jazzy-geometry-msgs
Requires:       ros-jazzy-launch-ros
Requires:       ros-jazzy-rclpy
Requires:       ros-jazzy-sensor-msgs
Requires:       ros-jazzy-tf2-ros-py
Requires:       ros-jazzy-ros-workspace
BuildRequires:  python%{python3_pkgversion}-devel
BuildRequires:  ros-jazzy-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  ros-jazzy-ament-copyright
BuildRequires:  ros-jazzy-ament-flake8
BuildRequires:  ros-jazzy-ament-pep257
%endif

%description
Has examples of using the tf2 Python API.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
%py3_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
%py3_install -- --prefix "/opt/ros/jazzy"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/jazzy

%changelog
* Wed Nov 20 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.5-1
- Autogenerated by Bloom

* Wed May 29 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.4-1
- Autogenerated by Bloom

* Mon May 13 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.3-1
- Autogenerated by Bloom

* Thu Apr 18 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.2-2
- Autogenerated by Bloom

* Wed Apr 10 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.2-1
- Autogenerated by Bloom

* Thu Mar 28 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.1-1
- Autogenerated by Bloom

* Wed Mar 06 2024 Alejandro Hernandez Cordero <alejandro@openrobotics.org> - 0.36.0-2
- Autogenerated by Bloom

