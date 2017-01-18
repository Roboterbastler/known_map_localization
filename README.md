# known_map_localization
ROS code related to my bachelor thesis "ROS-Modul zur Lokalisierung in a priori bekannten Karten".

This work is related to the project EffFeu of the DAI-Labor at TU Berlin (http://www.dai-labor.de/cog/laufende_projekte/efffeu/). It has been supervised by Christopher-Eyk Hrabia.

## Content
This repository consists of multiple packages

- **known_map_localization**: The main package containing the localization node developed in the thesis
- **mapmerge**: This package contains a slightly modified version of the mapmerge library by Stefano Carpin [1] available from [2]
- **mapstitch**: This package contains an also slightly modified version of mapstitch, a ROS package available from [3]
- **cs_merge_methods**: A modified package from the cs_merge stack available from [4]
- **cs_merge_msgs**: A package from the cs_merge stack available from [4]

## Dependencies
Additionally to standard ROS packages and the packages from this repository the following ROS packages or system libraries are needed to build known_map_localization:

- map_server
- geodesy
- geographic_msgs
- orb_slam (a modified version of ORB-SLAM from https://github.com/cehberlin/ORB_SLAM)
- Boost
- OpenCV

See `known_map_localization/package.xml` for a full list of dependencies.

## Usage
The known_map_localization node is launched with the `known_map_localization/launch/kml_node.launch` file. It takes 3 mandatory arguments:

- **slam_map_topic**: The topic where the SLAM occupancy grid map is published as a [nav_msgs/OccupancyGrid](http://docs.ros.org/indigo/api/nav_msgs/html/msg/OccupancyGrid.html) message
- **gps_topic**: The topic where GPS fixes are published as [sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) messages
- **known_map**: The full path to a configuration file which determines the known map. The format should be as described in http://wiki.ros.org/map_server#YAML_format extended by an additional tag specifying the anchor coordinates. As an example:

```
# Geographic pose of the anchor, using the WGS 84 reference ellipsoid.
# Orientation uses the East-North-Up (ENU) frame of reference (yaw angle, ccw, in degrees). 
anchor: {position: {altitude: 0., latitude: 52.51273962, longitude: 13.32486415}, heading: 90}

free_thresh: 0.196
image: known_map.png
negate: 0
occupied_thresh: 0.65
origin: [-10.0, -10.0, 0.0]
resolution: 0.1
```

The optional argument **algorithm** specifies which alignment algorithm to use. Valid options are: `mapmerge` (default), `mapstitch`, `cs_merge_icp_gradient` and `cs_merge_icp_svd`.

A sample roslaunch call starting known_map_localization:

```
roslaunch known_map_localization kml_node.launch slam_map_topic:=/orb_slam/projected_map gps_topic:=/robot/gps known_map:=PATH_TO_YAML.yaml
```

### Output
The following topics are used to publish the results of the localization:

- `known_map_localization/localization_node/geo_pose`: The estimated robot pose
- `known_map_localization/localization_node/kml_base_link`: The transform from the anchor to the robot pose

## Documentation

The known_map_localization code is documented using doxygen-style comments. Therefore documentation can be generated using doxygen.

## Resources
[1] Stefano Carpin. *Fast and accurate map merging for multi-robot systems*. In: *Autonomous Robots* 25.3 (2008), S. 305-316

[2] mapmerge library: http://robotics.ucmerced.edu/map-merging-code

[3] mapstitch ROS package: http://wiki.ros.org/mapstitch

[4] cs_merge ROS stack: http://wiki.ros.org/cs_merge
