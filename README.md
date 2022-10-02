# Sensor algorithms
- This module contains the algorithms for the larus soaring flight sensor

It is intended to be imported into another project via a .gitmodules file.

Presently this is used for the larus sensor software or the larus flight computer Software-In-The-Loop simulator.
    
# Add as submodule to repository:
- git submodule add git@github.com:larus-breeze/sw_sensor_algorithms.git lib

# Init and Update submodule
- git submodule init 
- git submodule update
- Open submodule folder "lib" and run "git pull" to update the submodule to the latest commit.

# Clone repository including the submodules: 
git clone --recursive URL git://github.com/foo/example.git

# Update submodule from using repository
just cd into the submodules folder and use git commit, push , ..
