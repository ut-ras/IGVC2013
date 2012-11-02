% Auto-generated.  Do not edit!

% msg = irobot_test_Locations()
%
% Locations message type, fields include:
% irobot_test_LocalizableObject{} objectlist

% //! \htmlinclude Locations.msg.html
function msg = irobot_test_Locations()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/home/sosentos/ros/brown-ros-pkg/trunk/experimental/rlrobot/irobot_test/msg/oct/irobot_test');
end


msg = [];
msg.objectlist = {};
msg.md5sum_ = @irobot_test_Locations___md5sum;
msg.type_ = @irobot_test_Locations___type;
msg.serializationLength_ = @irobot_test_Locations___serializationLength;
msg.serialize_ = @irobot_test_Locations___serialize;
msg.deserialize_ = @irobot_test_Locations___deserialize;
msg.message_definition_ = @irobot_test_Locations___message_definition;

function x = irobot_test_Locations___md5sum()
x = 'd54a280f7da3551346c2e82e4298dc42';

function x = irobot_test_Locations___message_definition()
x = [    'LocalizableObject[] objectlist\n' ...
    '\n' ...
    '================================================================================\n' ...
    'MSG: irobot_test/LocalizableObject\n' ...
    'uint32 objecttype\n' ...
    'uint32 objectid\n' ...
    'float64 posx\n' ...
    'float64 posy\n' ...
    'float64 yaw\n' ...
    '\n' ...
    '\n' ...
];

function x = irobot_test_Locations___type()
x = 'irobot_test/Locations';

function l__ = irobot_test_Locations___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.objectlist) * (irobot_test_LocalizableObject().serializationLength_());

function dat__ = irobot_test_Locations___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.objectlist), 'uint32');
for objectlist_i__ = 1:numel(msg__.objectlist)
    msg__.objectlist{objectlist_i__}.serialize_(msg__.objectlist{objectlist_i__}, seq__, fid__);
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = irobot_test_Locations___deserialize(dat__, fid__)
msg__ = irobot_test_Locations();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.objectlist = cell(size__, 1);
for objectlist_i__ = 1:size__
    msg__.objectlist{objectlist_i__} = irobot_test_LocalizableObject();
msg__.objectlist{objectlist_i__} = msg__.objectlist{objectlist_i__}.deserialize_(msg__.objectlist{objectlist_i__}, fid__);
end
if( file_created__ )
    fclose(fid__);
end
function l__ = irobot_test_Locations___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

