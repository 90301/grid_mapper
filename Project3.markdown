
Josh Benton
Mar-17-2015
Robotics

Project 3 Grid Mapping (Part1)
==========================

The majority of the time I spent on this project was actually making utility functions to take in laser data and convert it to a set (no duplicates) of pairs, and fixing the syntax issues with pairs. along with setting up unit testing.

random walk
-----------
I re-implemented random walk for this project, and in a way that should allow implementing a more advanced path planning algorithm later. I changed the FSM to just be an integer, and used constant integers instead of the enum variables. It seems to be a nice improvement, and has allowed me to create a code system for different movement states.

currently 1000 series states refer to forward moving states and 2000 series states refer to rotating states.


Getting the laser data
------------
Laser data is currently converted to vectors / sets of x,y coordinates of laser strikes. it seems to be working ok, but it is not production ready, and is currently disabled. (it prints to the console.)


XY vectors and sets
--------------
One of the major issues I keep running into is how to store x,y cordinates. I keep making a vector for each (one for x, one for y, sometimes one for angles).

I might need to make a custom data structure for it, as std::pairs seem to trip me up, it's really easy to forget to type std::pair, also if I make a class, I can make utility functions for the class.



Unit testing
--------------
There's a new namespace in town, IUtilsUnitTests, defined below the IUtils, which allows me to do unit testing, and I can hide the code with netbeans' bracket collapser.
I also added a feature to just do unit tests and not actually run the program. ultimately without the GUI working, unit testing geometric things isn't super helpful, but I have a namespace for it.


GUI Progress update
-----------
The qt5 ros gui compiles, and displays a qt5 window, is capable of drawing lines, is listening to a ros topic for those draw commmands, but I have not been able to test the actual draw commands. Hopefully with the new unit testing namespace, I can safely test that. The project is currently on github.
https://github.com/90301/inhaler_gui


TODO:
---------
>implement a set to contain all wall strikes, and a different version of the utility function that places new strikes into this set.
>better data-structures to hold the map.
>Exploration????
>Getting the gui working better



Screenshots
=====================
![mapping1](/home/inhaler/catkin_ws/src/bentonj/grid_mapper/gridmappingOccupancy.png)
![mapping2](/home/inhaler/catkin_ws/src/bentonj/grid_mapper/AnotherOccupancyTest.png)
![Inhaler GUI](/home/inhaler/catkin_ws/src/bentonj/grid_mapper/TheLineQt.png)
