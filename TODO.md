# TO DO

Figure out if we can use the dji position commands or if we need to use our own PID loop on local navigation

Most likely going to need our own PID loop

We need some kind of state estimation loop.
 - GPS (lat, long, altitude) [/dji_sdk/gps_position (sensor_msgs/NavSatFix)]
 - altitude [/dji_sdk/height_above_takeoff (std_msgs/Float32)] (only valid after drone armed)
 - imu [/dji_sdk/imu (sensor_msgs/Imu)] 100Hz
 - velocity [/dji_sdk/velocity (geometry_msgs/Vector3Stamped)]
 - attitude [/dji_sdk/attitude (geometry_msgs/QuaternionStamped)]

    We also care about 
    [/dji_sdk/flight_status (std_msgs/UInt8)]

At the bare minimum we need to choose which information to use for each step.
There are two, maybe three phases. 
 1. GPS Navigation to site (hopefully we can just use dji waypoints)
 2. Mapping Area
 3. Landing 


Demos: 
 - Upload a bunch of GPS waypoints
 - Fly in a grid using local coordinate reference
 - Build pcl map in gazebo
 - Land on a point in gazebo


End goal:
 - Use GPS waypoints to fly to location. Descend until ground is in range of down facing sensors
    [/dji_sdk/mission_waypoint_setSpeed (dji_sdk/MissionWpSetSpeed)]
    [/dji_sdk/mission_waypoint_upload (dji_sdk/MissionWpUpload)]
    [/dji_sdk/sdk_control_authority (dji_sdk/SDKControlAuthority)]
    [/dji_sdk/set_local_pos_ref (dji_sdk/SetLocalPosRef)]

 - Fly in grid pattern to map out a radius around the location (GPS or local state map)
 - Select landing site
 - Land at selected site

