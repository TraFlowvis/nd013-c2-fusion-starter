# Writeup Midterm

## Step 1 Part 2 PointCould findings

#### __Find and display 6 examples of vehicles with varying degrees of visibility in the point-cloud__

Vehicle 1

![vehicle 1](/doc/img/pcd_vehicle_1.png)

Vehicle 2

![vehicle 2](/doc/img/pcd_vehicle_2.png)

Vehicle 3

![vehicle 3](/doc/img/pcd_vehicle_3.png)

Vehicle 4

![vehicle 4](/doc/img/pcd_vehicle_4.png)

Vehicle 5

![vehicle 5](/doc/img/pcd_vehicle_5.png)

Vehicle 6

![vehicle 6](/doc/img/pcd_vehicle_6.png)

#### __Identify vehicle features that appear as a stable feature on most vehicles (e.g. rear-bumper, tail-lights) and describe them briefly. Also, use the range image viewer from the last example to underpin your findings using the lidar intensity channel.__
- Height, depth and width of the vehicle can be used for the cassification (keeping diffent vehicle types in mind)
- wheels can often be identified if the view sees a bit of the carside
- often smaller gradient in heigt on the front of the car (windschield and )


# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
Step 1 Filter: straight forward implement the functions of the EKF and adapt a bit for 6D matrices

Step 2 Track Managment: Hardest part to get the score calculation working. In the beginnen objects would not be deleted

Step 3 association: was easy to create the assosiation matrix and compute the MHD adn teh gating function

Step 4 camera fusion: Implemet fov function, Camera Meas init, geth(x) for camera
was realativ simple to implement the tasks with the formulas form the scrtipt with skipping a bit through the lectures


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
Yes there is much more information available for the detected object for classification or just for checking the results udn for visualisation. it's much easier to interpret real images than pointclouds or other representations from diffrent sensors


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
to differate between relevant ant irelevant objects (example parking cars in a parkin lot on the side of the road (end of video sequence))
computation limits can be probvlematic for real life applications 
it needs optimized algorithems in embedded hardware 


### 4. Can you think of ways to improve your tracking results in the future?
add more sensors for better detection and tracking
choose better sensors with higer resolution for better results
change the prediction for different types of objects (pedestrians discard when velocity to high as example)

