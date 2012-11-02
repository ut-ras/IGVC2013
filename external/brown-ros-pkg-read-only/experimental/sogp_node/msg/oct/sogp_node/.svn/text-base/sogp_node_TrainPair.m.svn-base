% Auto-generated.  Do not edit!

% msg = sogp_node_TrainPair()
%
% TrainPair message type, fields include:
%   sogp_node_Datapoint predictor
%   sogp_node_Datapoint target

% //! \htmlinclude TrainPair.msg.html
function msg = sogp_node_TrainPair()
persistent pathsadded__
if (isempty (pathsadded__))
    pathsadded__ = 1;
    addpath('/home/iscander/ros/projects/sogp_node/msg/oct/sogp_node');
end


msg = [];
msg.predictor = sogp_node_Datapoint();
msg.target = sogp_node_Datapoint();
msg.md5sum_ = @sogp_node_TrainPair___md5sum;
msg.type_ = @sogp_node_TrainPair___type;
msg.serializationLength_ = @sogp_node_TrainPair___serializationLength;
msg.serialize_ = @sogp_node_TrainPair___serialize;
msg.deserialize_ = @sogp_node_TrainPair___deserialize;

function x = sogp_node_TrainPair___md5sum()
x = '23cafbf003e8c14d0247eb42376c3366';

function x = sogp_node_TrainPair___type()
x = 'sogp_node/TrainPair';

function l__ = sogp_node_TrainPair___serializationLength(msg)
l__ =  ...
    + msg.predictor.serializationLength_(msg.predictor) ...
    + msg.target.serializationLength_(msg.target);

function dat__ = sogp_node_TrainPair___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
msg__.predictor.serialize_(msg__.predictor, seq__, fid__);
msg__.target.serialize_(msg__.target, seq__, fid__);
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = sogp_node_TrainPair___deserialize(dat__, fid__)
msg__ = sogp_node_TrainPair();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.predictor = sogp_node_Datapoint();
msg__.predictor = msg__.predictor.deserialize_(msg__.predictor, fid__);
msg__.target = sogp_node_Datapoint();
msg__.target = msg__.target.deserialize_(msg__.target, fid__);
if( file_created__ )
    fclose(fid__);
end
function l__ = sogp_node_TrainPair___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

