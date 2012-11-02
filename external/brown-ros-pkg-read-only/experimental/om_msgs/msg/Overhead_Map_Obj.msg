# For each type of name, there is an associated tuple:
#  * "point":  [x y]          draws a point at (x, y)
#  * "line":   [x0 y0 x1 y1]  draws a line from (x0, y0) to (x1, y1)
#  * "arrow":  [x0 y0 x1 y1]  draws an arrow (x0, y0) --> (x1, y1)
#  * "imname": [x y th]       draws an image named "imname" centered
#                               at (x,y), rotated th radians.
#                               location of image is
#                               "$OVERHEAD_MAP_ROOT/icons/imname.png".
string name
float32[] tuple
