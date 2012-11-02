% Auto-generated.  Do not edit!

% [reqmsg,resmsg] = core_testsrv1()
%
function [reqmsg,resmsg] = sogp_node_Predict()
if( nargout > 0 )
    reqmsg = sogp_node_Request();
end
if( nargout > 0 )
    resmsg = sogp_node_Response();
end

% Auto-generated.  Do not edit!

% msg = sogp_node_Request()
%
% Request message type, fields include:
%   sogp_node_Vector predictor

% //! \htmlinclude Request.msg.html
function msg = sogp_node_Request()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/home/iscander/ros/projects/sogp_node/msg/oct/sogp_node');
end


msg = [];
msg.create_response_ = @sogp_node_Response;
msg.predictor = sogp_node_Vector();
msg.md5sum_ = @sogp_node_Request___md5sum;
msg.server_md5sum_ = @sogp_node_Request___server_md5sum;
msg.type_ = @sogp_node_Request___type;
msg.serializationLength_ = @sogp_node_Request___serializationLength;
msg.serialize_ = @sogp_node_Request___serialize;
msg.deserialize_ = @sogp_node_Request___deserialize;

function x = sogp_node_Request___md5sum()
x = '';

function x = sogp_node_Request___server_md5sum()
x = 'fba85ef2d1dc3a06205281f14b943df8';

function x = sogp_node_Request___type()
x = 'sogp_node/PredictRequest';

function l__ = sogp_node_Request___serializationLength(msg)
l__ =  ...
    + msg.predictor.serializationLength_(msg.predictor);

function dat__ = sogp_node_Request___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
msg__.predictor.serialize_(msg__.predictor, seq__, fid__);
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = sogp_node_Request___deserialize(dat__, fid__)
msg__ = sogp_node_Request();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.predictor = sogp_node_Vector();
msg__.predictor = msg__.predictor.deserialize_(msg__.predictor, fid__);
if( file_created__ )
    fclose(fid__);
end
function l__ = sogp_node_Request___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

% msg = sogp_node_Response()
%
% Response message type, fields include:
%   sogp_node_Vector prediction
% string error_msg

% //! \htmlinclude Response.msg.html
function msg = sogp_node_Response()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/home/iscander/ros/projects/sogp_node/msg/oct/sogp_node');
end


msg = [];
msg.prediction = sogp_node_Vector();
msg.error_msg = '';
msg.md5sum_ = @sogp_node_Response___md5sum;
msg.server_md5sum_ = @sogp_node_Response___server_md5sum;
msg.type_ = @sogp_node_Response___type;
msg.serializationLength_ = @sogp_node_Response___serializationLength;
msg.serialize_ = @sogp_node_Response___serialize;
msg.deserialize_ = @sogp_node_Response___deserialize;

function x = sogp_node_Response___md5sum()
x = '';

function x = sogp_node_Response___server_md5sum()
x = 'fba85ef2d1dc3a06205281f14b943df8';

function x = sogp_node_Response___type()
x = 'sogp_node/PredictResponse';

function l__ = sogp_node_Response___serializationLength(msg)
l__ =  ...
    + msg.prediction.serializationLength_(msg.prediction) ...
    + 4 + numel(msg.error_msg);

function dat__ = sogp_node_Response___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
msg__.prediction.serialize_(msg__.prediction, seq__, fid__);
fwrite(fid__, numel(msg__.error_msg), 'uint32');
fwrite(fid__, msg__.error_msg, 'uint8');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = sogp_node_Response___deserialize(dat__, fid__)
msg__ = sogp_node_Response();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.prediction = sogp_node_Vector();
msg__.prediction = msg__.prediction.deserialize_(msg__.prediction, fid__);
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.error_msg = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = sogp_node_Response___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

