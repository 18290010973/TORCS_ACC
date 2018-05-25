# TORCS_ACC

***Source for compilation***
(1) https://github.com/ugo-nama-kun/gym_torcs

(2) https://github.com/fmirus/torcs-1.3.7#torcs-137

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

***Installation***

***Vehicle Simulator - TORCS 1.3.7***

Source - https://github.com/fmirus/torcs-1.3.7#torcs-137

This installation guide has been tested with Ubuntu 16.04 and 18.04

Install torcs dependencies

First we need to get some necessary debian packages
```
sudo apt-get install mesa-utils libalut-dev libvorbis-dev cmake libxrender-dev libxrender1 libxrandr-dev zlib1g-dev libpng16-dev
```
Now check for openGL/DRI by running
```
glxinfo | grep direct
```
The result should look like

direct rendering: Yes

Check for glut by running
```
dpkg -l | grep glut
```
If it is not installed run
```
sudo apt-get install freeglut3 freeglut3-dev
```
Check for libpng by running
```
dpkg -l | grep png
```
Install PLIB

First we have to create a folder for all torcs-related stuff. Therefore, run the following commands

cd /your_desired_location/

sudo mkdir torcs
```
export TORCS_PATH=/your_desired_location/torcs
```
cd $TORCS_PATH

Install PLIB-dependencies
```
sudo apt-get install libxmu-dev libxmu6 libxi-dev
```
Now download PLIB 1.8.5, unpack to the created directory and enter the plib folder by
```
sudo tar xfvz /path_to_downloaded_files/plib-1.8.5.tar.gz

cd plib-1.8.5
```
Before we compile plib we need need to set some environment variables
```
export CFLAGS="-fPIC"

export CPPFLAGS=$CFLAGS

export CXXFLAGS=$CFLAGS
```
Now we can configure and compile PLIB
```
./configure

make

sudo make install
```
Just for safety, wen unset our environment variables again
```
export CFLAGS=

export CPPFLAGS=

export CXXFLAGS=
```
Install openal

let's enter our base directory again
```
cd $TORCS_PATH
```
Now we download openal 1.17.2 and unpack it
```
sudo tar xfvj /path_to_downloaded_files/openal-soft-1.17.2.tar.bz2
```
We enter the build folder and compile openal
```
cd openal-soft-1.17.2/build

sudo cmake ..

sudo make

sudo make install
```
Install TORCS

Enter your TORCS_PATH
```
cd $TORCS_PATH
```
and clone this repository
```
git clone https://github.com/fmirus/torcs-1.3.7.git
```
Now we enter our torcs folder
```
cd torcs-1.3.7
```
Now build we build TORCS and log the output to a text-files as TORCS does not interrupt the build on errors
```
make >& error.log
```
Now open error.log with your favourite text editor and search for errors. If there are no errors you can proceed, otherwise you have to resolve.

Now we are ready to install torcs by running
```
sudo make install
```
Also install the torcs data-files by running
```
sudo make datainstall
```
If you made it this far, you can delete the TORCS_PATH variable by unset TORCS_PATH and are now ready to go. Congratulations :-)




