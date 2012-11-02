import rospy

aborted_pick = False

# Invoke whatever picking necessary.  Returns "completed", "failed" or "aborted"
def pickBlock():
    global aborted_pick
    aborted_pick = False
    rospy.sleep(5)
    if aborted_pick:
        return "aborted"
    else:
        return "completed"

# Cancels the pick motion.  Robot should freeze.  This method should do nothing if 
# the robot is not currently picking
def cancelPick():
    global aborted_pick
    aborted_pick = True
