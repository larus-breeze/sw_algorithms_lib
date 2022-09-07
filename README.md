# Sensor algorithms
- A module which contains the algorithms. It is intended to be imported into another repository via a .gitmodules file
    
# Add as submodule to repository:
git submodule add git@github.com:larus-breeze/sw_sensor_algorithms.git lib

# Update submodule
git submodule update --init --recursive

# Clone repository including the submodules: 
git clone --recursive URL git://github.com/foo/example.git
