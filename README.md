# Readme #

A project integrating Aruco markers, OGRE 3D engine and Oculus Rift with OpenCV for Augment Reality.

# Requirements #

- OGRE 1.9.0
- OpenCV 2.4.10
- Aruco (http://sourceforge.net/projects/aruco/)

# Quick Getting Compiled #

** Include paths (-I):** 

- {path_to_opencv_installation}/include
- {path_to_aruco_installation}/utils
- {path_to_aruco_installation}/src
- /usr/include/OGRE
- {path_to_ovr_sdk_installation}/LibOVRKernel/Src


**Libraries (-l): **

* opencv_core
* aruco
* OgreMain
* OVR
* pthread
* dl
* rt
* OIS
* boost_system
* opencv_imgproc
* opencv_highgui
* opencv_ml
* opencv_video
* opencv_features2d
* opencv_calib3d
* opencv_nonfree
* opencv_objdetect
* opencv_contrib
* opencv_legacy
* opencv_flann

**Library Search Path (-L):**

{path_to_opencv_installation}/lib

# Notes #

The makefile inside 'Debug' should work already, if everything has been setup properly. The include paths are different obviously.