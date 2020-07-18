# autocalibrate
An exercise in automatically calibrating a camera from a pair of images.

A command line utility that takes in the paths to two images taken with the same camera and tries to find the focal length and principal point.

It is assumed the images are taken relatively close together and the lens has no distortion. The actual parameters recovered are a two-dimensional principal point, a single focal length, and the pose of the second camera with respect to the first. The distance between the two cameras will be normalized to unit length.
