# proprietary_template
This provides a template for new NRG repos. It includes files that NRG repos generally all need.

## Things you'll need to fill in or update:
 - Copyright year in LICENSE.txt
 - The CHANGELOG.md
 - This README!
 - Add the new repository to the [NRG Repository List](https://wikis.utexas.edu/display/NRG/NRG+Repository+List)

## Adding a new catkin package
If you want to create a new catkin package, use `catkin_create_pkg` and move the files:
```sh
cd <INTO YOUR NEW REPOSITORY>
catkin_create_pkg <pkg_name> <depend1> <depend2> <...>
mv <pkg_name>/* .
rm -r <pkg_name>/
```

## NRG Sample ROS Code
See [nrg_ros_samples](https://github.com/UTNuclearRobotics/nrg_ros_samples) for examples of good ROS practices, CMakeLists help, and more.
