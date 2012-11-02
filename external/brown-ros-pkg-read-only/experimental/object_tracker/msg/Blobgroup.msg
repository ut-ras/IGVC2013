# This message contains number of group of blob, group number
# and center pose of each blob
# variance of blob may be added
Header                      header
uint16[]                    group_num
uint16[]                    blob_size
geometry_msgs/Pose[]        pose
