# All screen measurements are in pixels, all spatial measurements are in meters.
# Angles in radians.
uint32 id
# This is a rating of confidence in the tag pattern identification 0 < cf < 1.
float64 cf
uint32 x
uint32 y
# The 'diameter' is the square root of the tag's actual area, as estimated by 
# the AR software.  You can use it to check the cf confidence.
float64 diameter
# This is the estimated distance from viewer to the center of the tag.
float64 distance
float64 xRot 
float64 yRot 
float64 zRot
float64 xMetric
float64 yMetric
float64 zMetric
# Screen coordinates of the four corners.
float64[8] cwCorners
# FOR TESTING ONLY
# Uncommenting this and uncommenting the similarly-marked lines in 
# ar_recog.cpp will put the ARToolkit rotation matrix into the Tag message,
# which can be useful for debugging and testing.
# float64[3] c3
# float64[3] t0
# float64[3] t1
# float64[3] t2
