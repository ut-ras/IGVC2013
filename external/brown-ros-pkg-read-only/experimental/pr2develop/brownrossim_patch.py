#Run this file to automatically put the visualization files where they need to be for the ROS simulator

from shutil import copyfile, copytree, ignore_patterns
import os
import fnmatch
from subprocess import call
from subprocess import Popen
from subprocess import PIPE
from os import chdir
from os import makedirs
from os import remove
from uu import decode
from StringIO import StringIO



if __name__ == "__main__":
    dot = Popen(["pwd"], stdout=PIPE).communicate()[0]
    dot = dot[0:-1]
rospath = dot + "/ros-1.0.0"

#makedirs(rospath + "/pkg/roslite")
#names=os.listdir('/home/sosentos/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/launch');
#copytree('/home/sosentos/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/launch/', '~/ros/stacks/simulator_gazebo/gazebo_worlds/launch');

for file in os.listdir(os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/launch')):
   if fnmatch.fnmatch(file, '*.launch'):
      copyfile((os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/launch/')+ file), os.path.expanduser('~/ros/stacks/simulator_gazebo/gazebo_worlds/launch/' + file) );

for file in os.listdir(os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/models')):
   if fnmatch.fnmatch(file, '*.mesh'):
      copyfile((os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/models/')+ file), os.path.expanduser('~/ros/stacks/simulator_gazebo/gazebo_worlds/Media/models/' + file) );

for file in os.listdir(os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/objects')):
   if fnmatch.fnmatch(file, '*.model'):
      copyfile((os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/adding_objects/objects/')+ file), os.path.expanduser('~/ros/stacks/simulator_gazebo/gazebo_worlds/objects/' + file) );

for file in os.listdir(os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/texture_mapping')):
   if fnmatch.fnmatch(file, '*.png'):
      copyfile((os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/texture_mapping/')+ file), os.path.expanduser('~/ros/stacks/simulator_gazebo/gazebo_worlds/Media/materials/textures/'+ file) );

for file in os.listdir(os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/texture_mapping')):
   if fnmatch.fnmatch(file, '*.jpg'):
      copyfile((os.path.expanduser('~/ros/brown-ros-pkg/experimental/pr2develop/texture_mapping/')+ file), os.path.expanduser('~/ros/stacks/simulator_gazebo/gazebo_worlds/Media/materials/textures/'+ file) );


material=open(os.path.expanduser('~/ros/stacks/simulator_gazebo/gazebo_worlds/Media/materials/scripts/gazebo_worlds.material'), 'a');

materialtext="""material GazeboWorlds/ART_SA
{
  technique
  {
    pass
    {
      texture_unit
      {
	texture artag_sa.jpg
      }
    }
  }
}

material GazeboWorlds/TAGWALL1
{
  technique
  {
    pass
    {
      texture_unit
      {
	TagPat.png
      }
    }
  }
}

material GazeboWorlds/pattHiro
{
  technique
  {
    pass
    {
      texture_unit
      {
	TagPat.png
      }
    }
  }
}""";
material.write(materialtext)
