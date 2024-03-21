# RobeePclTests

## Install

1) [pcl linux install ](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html)
1) git clone this repo
1) compile this repo
```
cd <path/to/this/repo>/build/
cmake ..
make
```
## Run
1) change in viewer main the desired pcd file 
1) compile and run:
```
make 
./pcl_visualizer_demo 
```
### args:
Integrated with this module is arg parse from the terminal

* -l: loads a PCD file flag
* -f: file path for the PCD file
* -s: use realsense stream
* -c: use colored realsense stream
* -h: print help
* -v: print verbose
