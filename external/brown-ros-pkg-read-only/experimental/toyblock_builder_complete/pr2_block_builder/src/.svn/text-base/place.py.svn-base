import rospy

aborted_place = False

# Invoke whatever placing necessary.  Method should block until action completed.  
# Returns "completed", "failed" or "aborted"
def placeBlock(block):
    global aborted_place
    aborted_place = False
    rospy.sleep(5)
    if aborted_place:
        return "aborted"
    else:
        return "completed"

# Cancels the place motion.  Robot should freeze.  
# This method should do nothing if the robot is not currently placing.
def cancelPlace():
    global aborted_place
    aborted_place = True
