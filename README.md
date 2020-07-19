# Autocalibrate
An exercise in automatically calibrating a camera from a pair of images.

A command line utility that takes in the paths to two images taken with the same camera and tries to find the focal length and principal point.

It is assumed the images are taken relatively close together and the lens has no distortion. The actual parameters recovered are a two-dimensional principal point, a single focal length, and the pose of the second camera with respect to the first. The distance between the two cameras will be normalized to unit length.

There are a number of spots that make assumptions or do things in a less than optimal way. The final result is also unreliable and imprecise. There's a lot that needs to be added to make this project actually useful, but it's a good starting framework.