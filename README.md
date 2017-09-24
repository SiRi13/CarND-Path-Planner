# CarND-Path-Planner
Implement a path planner to navigate a car around a simulated highway

## Reflection on how the path is generated

The path gets in the class _PathPlanner_, implemented in `src/path.planner.cpp`, generated. The process starts by parsing the *json* data from the web server in the ``set_data(string data)`` method (lines 3 through 25).
To create a path, the `createPath()` method gets called. The behavior planner initially determines where the other cars are and if a lane change is necessary. After running `_behaviorPlanner.set_sensor_data(_sensor_fusion, _previousPath)` in line 30, the behavior planner contains the new lane and the speed of the goal lane.  
In line 36 starts the path creation. To sustain a smooth transmission from the previous path to the new, two points from the previous path are used as reference for the new trajectory. If there a no old points to use, a point is interpolated from the current position of the ego car.  
Those points are pushed onto the x and y list of the future anchor points as start of the spline. In line 68 three new anchor points, about 30 meters apart, were created and appended to the list. To avoid multiple y-values to one x-value, the coordinates are shifted and rotated to match the car coordinates. Positive x points against driving direction and positive y points towards the right hand side.  
After creating the anchor spline in line 87 and setting the anchor points in line 89, the old path points get pushed onto the new path. Then the missing points get calculated in the for-loop at line 101. Within the for-loop gets the reference velocity updated, depending on the lane velocity from the behavior planner and the maximum velocity of 49.5 MPH. To meet this velocity a part of the spline is divided by how far the car has to travel in 0.02 ms, which is the update rate of the simulator. With the amount of points on this stretch, the corresponding point on the spline can be calculated, transformed back from the car coordinates and appended to the new path.  
This process guarantees a smooth transmission from the previous path and avoids exceeding the velocity, acceleration and jerk thresholds.
