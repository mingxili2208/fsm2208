# how to install carla & UE4

follow the steps of <https://carla.readthedocs.io/en/latest/build_linux/>

while:

pip3 install -Iv setuptools==47.3.1

1. set python version at least above 3.5.15
   1. make PythonAPI until：ninja: error: unknown target 'osm2odr'
2. use the tag 0.9.15.2 of carla

when complie autoware

colcon build --symlink-install 

g++ >8  

while install autoware

pip install 'setuptools==59.6.0'