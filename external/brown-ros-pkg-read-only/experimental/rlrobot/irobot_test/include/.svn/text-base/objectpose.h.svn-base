/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#ifndef OBJECT_POSE_H
#define OBJECT_POSE_H OBJECT_POSE_H

namespace rlab {

class ObjectPose
{
public:
  ObjectPose();
  ObjectPose(float x, float y, float yaw);
  virtual ~ObjectPose();

  void setX(float x);
  float getX() const;

  void setY(float y);
  float getY() const;

  void setYaw(float yaw);
  float getYaw() const;

protected:
  float x_;
  float y_;
  float yaw_;
};

} // namespace rlab

#endif // OBJECT_POSE_H

