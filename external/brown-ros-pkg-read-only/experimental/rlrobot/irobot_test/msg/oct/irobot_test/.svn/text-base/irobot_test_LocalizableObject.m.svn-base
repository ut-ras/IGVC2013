% Auto-generated.  Do not edit!

% msg = irobot_test_LocalizableObject()
%
% LocalizableObject message type, fields include:
% uint32 objecttype
% uint32 objectid
% double posx
% double posy
% double yaw

% //! \htmlinclude LocalizableObject.msg.html
function msg = irobot_test_LocalizableObject()

msg = [];
msg.objecttype = uint32(0);
msg.objectid = uint32(0);
msg.posx = double(0);
msg.posy = double(0);
msg.yaw = double(0);
msg.md5sum_ = @irobot_test_LocalizableObject___md5sum;
msg.type_ = @irobot_test_LocalizableObject___type;
msg.serializationLength_ = @irobot_test_LocalizableObject___serializationLength;
msg.serialize_ = @irobot_test_LocalizableObject___serialize;
msg.deserialize_ = @irobot_test_LocalizableObject___deserialize;
msg.message_definition_ = @irobot_test_LocalizableObject___message_definition;

function x = irobot_test_LocalizableObject___md5sum()
x = '7d1f0aa2894b6168a0ed8aeafb7bd204';

function x = irobot_test_LocalizableObject___message_definition()
x = [    'uint32 objecttype\n' ...
    'uint32 objectid\n' ...
    'float64 posx\n' ...
    'float64 posy\n' ...
    'float64 yaw\n' ...
    '\n' ...
    '\n' ...
];

function x = irobot_test_LocalizableObject___type()
x = 'irobot_test/LocalizableObject';

function l__ = irobot_test_LocalizableObject___serializationLength(msg)
l__ =  ...
    + 4 ...
    + 4 ...
    + 8 ...
    + 8 ...
    + 8;

function dat__ = irobot_test_LocalizableObject___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.objecttype, 'uint32');
c__ = c__ + fwrite(fid__, msg__.objectid, 'uint32');
c__ = c__ + fwrite(fid__, msg__.posx, 'double');
c__ = c__ + fwrite(fid__, msg__.posy, 'double');
c__ = c__ + fwrite(fid__, msg__.yaw, 'double');
if( c__ ~= 5 )
    error('some members of msg irobot_test:LocalizableObject are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = irobot_test_LocalizableObject___deserialize(dat__, fid__)
msg__ = irobot_test_LocalizableObject();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.objecttype = fread(fid__,1,'uint32=>uint32');
msg__.objectid = fread(fid__,1,'uint32=>uint32');
msg__.posx = fread(fid__,1,'double=>double');
msg__.posy = fread(fid__,1,'double=>double');
msg__.yaw = fread(fid__,1,'double=>double');
if( file_created__ )
    fclose(fid__);
end
function l__ = irobot_test_LocalizableObject___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

