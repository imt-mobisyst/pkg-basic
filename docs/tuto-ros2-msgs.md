# Learning ROS Mesages:

ressource 

- https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html


## On the mesages package side

See _basic_msgs_ for exemple.

1. Create `msg` directory and a mesage definition file `MesageName.msg` on it.
2. The `MesageName.msg` contains one line by component (`type name`).
    - type can be simple: `string`, `int32`, ...
    - resulting from another ros package : `pkg_name/msg/type`.
3. update the `rosidl_generate_interfaces` command in the `CMakeList.txt` file. Do not forget the dependance to the `pkg_name` ROS2 package.

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(pkg_name REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MesageName.msg"
  DEPENDENCIES pkg_name
)
```

4. Same with `package.xml`: activate the build tool for mesages and services, and add the appropriate dependencies.

```xml
<depend>pkg_name</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

5. You can build your package...


## On the node package side

Update your `package.xml` and `CMakeList.txt` with dependencies to your mesages package.

