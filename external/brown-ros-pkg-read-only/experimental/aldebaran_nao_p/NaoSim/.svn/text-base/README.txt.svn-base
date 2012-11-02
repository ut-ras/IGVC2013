This is a node for using the NAO in simulation. Please use this before testing behaviors on the real robot. To use this node, you need to set the following environmental variables with the absolute path the following libraries. I recommend putting them in your rosenv file.

export AL_PYTHON_PATH={naoqi}/extern/python/aldebaran
export LD_LIBRARY_PATH={naoqi}/extern/c/linux/lib/python2.5

Then start up a naoqi (run ./naoqi in {naoqi}/bin). Start up Choregraphe and connect to the local naoqi at 9559. Now you can start the control.py and publish.py nodes. Currently the publish node does not report the correct values for the joints. Also the duration value is currently ignored.

