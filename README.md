# TORCS_ACC

***Setup***

(1)Two car platooning using TORCS.\
(2)The follower car tries to follows the leader car with a minimum specified distance.\

First car: The car follows the track angle and vehicle position to control the steering, and moves forward.\

Second car: The control is taken car in multipart's.\
Controls the acceleration and brakes (depends on the leader car)\
Controls the steering movements (depends on the side wall distance)

***Goals of the Cars***

Leader Car:\
Try to maintain a uniform speed (60-70km/hr) (target speed in our case 60 km/ hr)

Follower Car:\
Follow the Leader car. (do not go ahead/ lag too much)\
Try to maintain the speed limits of 70km/hr.\
Maintain a desirable distance from the leader Car. Here the desired distance is 20m.\



