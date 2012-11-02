#contains the values x,y,z,wx,wy,wz
float32[6] armPose
# 0 closes the hand, 1 opens the hand, any other value leaves the hand alone
float32 hand
# mask is a bit mask for which degrees of freedom you wish to control in the arm
# 1=x 2=y 4=z 8=wx 16=wy 32=wz, add to get desired mask
int32 mask
float32 duration
