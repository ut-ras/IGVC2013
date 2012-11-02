See http://code.google.com/p/brown-ros-pkg/wiki/ar_recog for documentation.

  NOTE: As of 26 Sep 2011, the following changes have been made to
  this code:

    - Bug fix for the zRot and xRot variables in the output Tag
      messages.  This has been fixed.

    - The yRot sign was incorrect according to usual ROS usage.  Sign
      was reversed from the way it was.

    - There was no time stamp in the output Tags message.  One has
      been added.

    - Output distance variable was in millimeters while xMetric and
      friends were in meters.  It is now reported in meters.
