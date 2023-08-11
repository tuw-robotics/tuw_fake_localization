# tuw_fake_localization
The node fake_localization publishes a tf-tree based on published odometry msgs.<br>
There are multiple use cases for a fake self-localization. 
In the end, one likes to have a tf-tree such as:<br>

(world -->) map --> odom --> base_link

world is not allways wanted or needed.

## Perfect odometry
In your simulation, you may be able to publish a perfect odometry. But where is the origin of this odometry? Is it the same as the world coordinates or relative to the position where the robot was spawned first?
### Perfect in world coordinates
* Background knowledge: __world --> map__  

In this case, the user has to know the offset between the used map and the world coordinate system and also 
### Perfect in odometry coordinates
* Background knowledge: __map --> odom__  

In this case, the user has to know where the robot was spawned on the map.

## Coordinate systems
A very brief review of the most commonly used coordinate systems
### Vehicle base coordinates
The robot base coordinate system defined by the URDF or RobotModel
### Vehicle odometry coordinates
Origin where the robots odometry started. Be aware that odometry is normally flawed (with errors).
### Map coordinates
Origin of the used map or local coordinate system
### World coordinates
#### Real system 
The earth's origin, or GNSS origin
#### On a simulation
The origin inside the simulation, or a simulated GNSS origin

