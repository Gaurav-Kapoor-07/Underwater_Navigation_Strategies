# Gaurav_ROS_Packages

## Hovering Node

1. Subscribes to distance and confidence values from the pinger sonar and the depth values from the ms5803 pressure sensor.
2. hovering_distance parameter is used to set the hovering distance from the ground in meters.
3. Hovering can be enabled or disabled using /hovering/enable service.
4. Added median filter for filtering pinger distance data with low confidence.
5. After cloning and building this package, run the following command to launch the hovering node - `roslaunch hovering hovering_node.launch`.