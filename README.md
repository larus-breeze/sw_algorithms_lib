# Sensor algorithms
- A module which contains the algorithms. It is intended to be imported into another repository via a .gitmodules file
    
# Add as submodule to repository:
git submodule add git@github.com:larus-breeze/sw_sensor_algorithms.git lib

# Update submodule
Open submodule folder "lib" and run "git pull" to update the submodule to the latest commit. 

# Clone repository including the submodules: 
git clone --recursive URL git://github.com/foo/example.git

# Update submodule from using repository
just cd into the submodules folder and use git commit, push , ..
