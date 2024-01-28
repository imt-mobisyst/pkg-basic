# Draft - ROS packages

This documentation is based on  the official documentations: [docs.ros.org](https://docs.ros.org),
with somes tuning considering our preferencies...
Typically we prefer cmake methode, what ever the targeted langauge (C++ or python).

You also find a lot of valuable information on the ROS1 wiki: [wiki.ros.org](https://wiki.ros.org) (for instance the definition of the mesages).

## Your first package in ROS2

You can build your first package using the ros tool, with the build-type `ament_cmake` build type, then compile and install the package:

```sh
ros2 pkg create --build-type ament_cmake my_first_package
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON --packages-select my_first_package
```

Package creation generate the minimal files to identify `my_first_package` directory as a ROS package source.
The file `package.xml` setup the ROS configuration, the name of the packages its dependanties in ROS ecosystem. 
The file `CMakeLists.txt` is required to use [CMake](https://en.wikipedia.org/wiki/CMake) tool, a very famous cross platform tool for automatizing building processes.
The `colcon build` command produces `build` and `install` directory with tempory file for the build process and the built ressources of your package (yes, none at this point...).
To informe your terminal that some resources are availlable here, you have to `source` the ROS setup file. 

```sh
ls
source ./install/setup.bash
```

## The package.xml file

As we said, `package.xml` is the entrance point of the package for ROS tools.
It is a text file in [eXtensible Markup Language](https://fr.wikipedia.org/wiki/Extensible_Markup_Language) format.
Typically, it regroups all the dependancies, for instance `rclcpp` and `rclpy` the ROS client librairies for Cpp and Python or `std_msgs`.
[std_msgs](http://wiki.ros.org/std_msgs) is a ROS package defning the format of the simple and commun mesages allowing the communication between ROS processes.

In the `package.xml` file :

```xml
<package format="3">
  ...
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  ...
</package>
```

For information about `package.xml` you can referer to the official specifications on [www.ros.org](https://www.ros.org/reps/rep-0149.html).


## The CMakeLits.txt 

The `CMakeLits.txt` provide the instruction on how to build librairies and programes.
Generally the file start with a project name and the importation of dependancies.
The dependancies must be installed, reachable on the machine and with the appropriate version number.
Then its define how to buid new resources (typically, librairies and programe/executable).
And finally the element to install and where.

For instance, all launchfile included in the `launch` driectory can be installed with the cmake command: 

```sh
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

### Python scripts with Cmake

Scripts are short executable code file, defining a ROS Node (in our context).

The simplest way to include Python-based ROS node depending on a specific [Python Package](https://docs.python.org/3/glossary.html#term-package) is to use `install` instruction in `CMakeLists.txt`.
Considering that your work is in a `scripts` directory and knowing that the ROS destination to make the ressource available for `ros2 run` is `lib` (...),
the install instructions would looklike:

```sh
# Python scripts
install( DIRECTORY scripts/myPythonPkg DESTINATION lib/${PROJECT_NAME})
install( PROGRAMS scripts/myNode DESTINATION lib/${PROJECT_NAME})
```

Most of the primitive (`find_package`, `add_executable`, `install`) and macros (`PROJECT_NAME`, `REQUIRED`, ... ) are CMake primitives and macros.
The `ament` tools provides some of primitive dedicated to [ROS build automation](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html).
