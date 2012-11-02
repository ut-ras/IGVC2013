% Auto-generated.  Do not edit!

% [reqmsg,resmsg] = core_testsrv1()
%
function [reqmsg,resmsg] = joint_states_listener_ReturnJointStates()
if( nargout > 0 )
    reqmsg = joint_states_listener_Request();
end
if( nargout > 0 )
    resmsg = joint_states_listener_Response();
end

% Auto-generated.  Do not edit!

% msg = joint_states_listener_Request()
%
% Request message type, fields include:
% string{} name

% //! \htmlinclude Request.msg.html
function msg = joint_states_listener_Request()

msg = [];
msg.create_response_ = @joint_states_listener_Response;
msg.name = {};
msg.md5sum_ = @joint_states_listener_Request___md5sum;
msg.server_md5sum_ = @joint_states_listener_Request___server_md5sum;
msg.server_type_ = @joint_states_listener_Request___server_type;
msg.type_ = @joint_states_listener_Request___type;
msg.serializationLength_ = @joint_states_listener_Request___serializationLength;
msg.serialize_ = @joint_states_listener_Request___serialize;
msg.deserialize_ = @joint_states_listener_Request___deserialize;
msg.message_definition_ = @joint_states_listener_Request___message_definition;

function x = joint_states_listener_Request___md5sum()
x = '';

function x = joint_states_listener_Request___server_md5sum()
x = 'ce9bd2b56c904b190a782a08482fb4e9';

function x = joint_states_listener_Request___server_type()
x = '';

function x = joint_states_listener_Request___message_definition()
x = [    '\n' ...
];

function x = joint_states_listener_Request___type()
x = 'joint_states_listener/ReturnJointStatesRequest';

function l__ = joint_states_listener_Request___serializationLength(msg)
l__ =  ...
    + 4 + joint_states_listener_Request___sum_array_length__(cellfun(@(name_elt__) 4 + numel(name_elt__), msg.name));

function dat__ = joint_states_listener_Request___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.name), 'uint32');
for name_i__ = 1:numel(msg__.name)
    fwrite(fid__, numel(msg__.name{name_i__}), 'uint32');
fwrite(fid__, msg__.name{name_i__}, 'uint8');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = joint_states_listener_Request___deserialize(dat__, fid__)
msg__ = joint_states_listener_Request();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.name = cell(size__, 1);
for name_i__ = 1:size__
    size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.name{name_i__} = fread(fid__, size__, '*char')';
end
if( file_created__ )
    fclose(fid__);
end
function l__ = joint_states_listener_Request___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

% msg = joint_states_listener_Response()
%
% Response message type, fields include:
% uint32{} found
% double{} position
% double{} velocity
% double{} effort

% //! \htmlinclude Response.msg.html
function msg = joint_states_listener_Response()

msg = [];
msg.found = [];
msg.position = [];
msg.velocity = [];
msg.effort = [];
msg.md5sum_ = @joint_states_listener_Response___md5sum;
msg.server_md5sum_ = @joint_states_listener_Response___server_md5sum;
msg.server_type_ = @joint_states_listener_Response___server_type;
msg.type_ = @joint_states_listener_Response___type;
msg.serializationLength_ = @joint_states_listener_Response___serializationLength;
msg.serialize_ = @joint_states_listener_Response___serialize;
msg.deserialize_ = @joint_states_listener_Response___deserialize;
msg.message_definition_ = @joint_states_listener_Response___message_definition;

function x = joint_states_listener_Response___md5sum()
x = '';

function x = joint_states_listener_Response___server_md5sum()
x = 'ce9bd2b56c904b190a782a08482fb4e9';

function x = joint_states_listener_Response___server_type()
x = '';

function x = joint_states_listener_Response___message_definition()
x = [    '\n' ...
];

function x = joint_states_listener_Response___type()
x = 'joint_states_listener/ReturnJointStatesResponse';

function l__ = joint_states_listener_Response___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.found) * (4) ...
    + 4 + numel(msg.position) * (8) ...
    + 4 + numel(msg.velocity) * (8) ...
    + 4 + numel(msg.effort) * (8);

function dat__ = joint_states_listener_Response___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.found), 'uint32');
fwrite(fid__, msg__.found(:), 'uint32');
fwrite(fid__, numel(msg__.position), 'uint32');
fwrite(fid__, msg__.position(:), 'double');
fwrite(fid__, numel(msg__.velocity), 'uint32');
fwrite(fid__, msg__.velocity(:), 'double');
fwrite(fid__, numel(msg__.effort), 'uint32');
fwrite(fid__, msg__.effort(:), 'double');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = joint_states_listener_Response___deserialize(dat__, fid__)
msg__ = joint_states_listener_Response();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.found = fread(fid__, size__, 'uint32=>uint32');
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.position = fread(fid__, size__, 'double=>double');
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.velocity = fread(fid__, size__, 'double=>double');
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.effort = fread(fid__, size__, 'double=>double');
if( file_created__ )
    fclose(fid__);
end
function l__ = joint_states_listener_Response___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

