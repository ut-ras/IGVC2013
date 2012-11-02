Header                  header
int8                    id
int8                    type
int8                    status
geometry_msgs/Pose      pose
geometry_msgs/Vector3   dim
std_msgs/ColorRGBA      color

bool                    inhand

int8                  SPHERE = 0
int8                  BOX    = 1

int8                  VISIBLE  = 1
int8                  INSHADOW = 2
