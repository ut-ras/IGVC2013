# This data type holds a collection of points in n-space meant to
# represent a 'guess' or a 'belief'.  The points are meant to
# represent a solution to some problem in that space, and each of the
# points in this array is a potential solution, equal in probability
# to all the others.
Header header
#
# The time stamp recorded here is the time of the last real data on
# which this guess is based.  So if this is the result of an odometry
# guess, this should be the stamp of the odometry message that was the
# input.
time source_stamp
#
# A guess will always be based on some real data.  For debugging and
# ground-truth purposes, put a string representation of that data into
# this string.  Think of this as a buffer for raw data.
string source_data
#
# The data_type value should be a human-readable indication of the
# kind of data we're passing here, a name of the problem being
# solved, like "location" or "coverage."
string data_type
#
# The data_type may not be specific enough for many applications.
# For example, a team of robots may be passing around location
# information about different robots.  Use this field to differentiate
# among like data types.
string data_sub_type
#
# The sender string is meant to be a human-readable label for the
# process doing the sending.
string sender
#
# The means and stds provide an indication of the consensus view
# represented by the 'points' list of n-dimensional points.
float32[] means
float32[] stds
#
# The pers array specifies whether the dimensions are periodic.  A
# zero in this array means an aperiodic dimension, while a non-zero
# entry indicates a period.  So (x,y,theta) would have pers (0,0,2*pi).
float32[] pers
#
# Provides names for each of the dimensions, e.g. ("x", "y", "theta")
string[] dim_names
#
# The array of points
Pt[] points
