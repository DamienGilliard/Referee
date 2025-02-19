# Installation

## Dependencies
Referee relies on PCL for basic point cloud manipulation such as transformations and visualisation.
To install PCL system-wide on your computer, follow those steps:
### PCL dependencies
in a terminal run:
`sudo apt-get install build-essential ibeigen3-dev libflann-dev libusb-1.0-0-dev libopenni-dev libopenni2-dev libpng-dev libqhull-dev zlib1g-dev libpcap-dev freeglut3-dev libboost-all-dev libvtk9-dev qtbase5-dev qtdeclarative5-dev libqt5opengl5-dev`

### PCL
Then, follow the instructions from [pcl](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html) to install it.

## build
detailed instructions for library and executables building and linking will come when the CMake project is in its "final" state, but for now, in short:

```bash
cd to/Referee/root/folder # if you are not already in the root folder of Referee

mkdir build && cd build # create a build folder and go there

cmake .. # Create the configuration files

make # build and link the library and the executable in one command
```