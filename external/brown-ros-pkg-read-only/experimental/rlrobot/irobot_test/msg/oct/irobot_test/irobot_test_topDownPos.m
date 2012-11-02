% Auto-generated.  Do not edit!

% msg = irobot_test_topDownPos()
%
% topDownPos message type, fields include:
% single posx
% single posy
% single yaw

% //! \htmlinclude topDownPos.msg.html
function msg = irobot_test_topDownPos()

msg = [];
msg.posx = single(0);
msg.posy = single(0);
msg.yaw = single(0);
msg.md5sum_ = @irobot_test_topDownPos___md5sum;
msg.type_ = @irobot_test_topDownPos___type;
msg.serializationLength_ = @irobot_test_topDownPos___serializationLength;
msg.serialize_ = @irobot_test_topDownPos___serialize;
msg.deserialize_ = @irobot_test_topDownPos___deserialize;

function x = irobot_test_topDownPos___md5sum()
x = '7f8be2666b5b18a1db5ae6e18b2a0607';

function x = irobot_test_topDownPos___type()
x = 'irobot_test/topDownPos';

function l__ = irobot_test_topDownPos___serializationLength(msg)
l__ =  ...
    + 4 ...
    + 4 ...
    + 4;

function dat__ = irobot_test_topDownPos___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.posx, 'single');
c__ = c__ + fwrite(fid__, msg__.posy, 'single');
c__ = c__ + fwrite(fid__, msg__.yaw, 'single');
if( c__ ~= 3 )
    error('some members of msg irobot_test:topDownPos are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = irobot_test_topDownPos___deserialize(dat__, fid__)
msg__ = irobot_test_topDownPos();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.posx = fread(fid__,1,'single');
msg__.posy = fread(fid__,1,'single');
msg__.yaw = fread(fid__,1,'single');
if( file_created__ )
    fclose(fid__);
end
function l__ = irobot_test_topDownPos___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

