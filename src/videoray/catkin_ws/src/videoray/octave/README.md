Open Loop Simulation of VideoRay Pro III Dynamics
==================================================
Kevin DeMarco <demarco@gatech.edu>

Notes
-------
This simulation was written within the Octave framework.  
http://www.gnu.org/software/octave/

Almost all of the code could easily be executed in Matlab, with just a
modification to the ode solver call.

Run the simulation
--------------------
1.) Start octave in this directory.  
2.) Execute the sim.m script  

### Ubuntu example:
$ cd /path/to/syllo-ros/catkin_ws/src/videoray/octave/open-loop  
$ octave  
$ sim  

The user will be presented with several options. The sim_lsode call should 
generate six (6) figures that show the output of the simulation.

Installation of Octave's odepkg
---------------------------------

### Ubuntu
$ sudo apt-get install sudo apt-get install octave-odepkg

### Others
1.) Download the odepkg source from:  
http://octave.sourceforge.net/odepkg

2.) Run octave in the same directory as the downloaded file, which will 
be named something like odepkg-0.8.4.tar.gz. In my case, it downloaded the
file to the ~/Downloads directory.

$ cd ~/Downloads

3.) Start Octave (with sudo / installation privileges) and install the 
package.

$ octave   (or "sudo octave" in a UNIX environment)  
$ pkg install odepkg-0.8.4.tar.gz
