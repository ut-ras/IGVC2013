% Auto-generated.  Do not edit!

% msg = sogp_node_Datapoint()
%
% Datapoint message type, fields include:
% single{} data

% //! \htmlinclude Datapoint.msg.html
function msg = sogp_node_Datapoint()

msg = [];
msg.data = [];
msg.md5sum_ = @sogp_node_Datapoint___md5sum;
msg.type_ = @sogp_node_Datapoint___type;
msg.serializationLength_ = @sogp_node_Datapoint___serializationLength;
msg.serialize_ = @sogp_node_Datapoint___serialize;
msg.deserialize_ = @sogp_node_Datapoint___deserialize;

function x = sogp_node_Datapoint___md5sum()
x = '420cd38b6b071cd49f2970c3e2cee511';

function x = sogp_node_Datapoint___type()
x = 'sogp_node/Datapoint';

function l__ = sogp_node_Datapoint___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.data) * (4);

function dat__ = sogp_node_Datapoint___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.data), 'uint32');
fwrite(fid__, msg__.data(:), 'single');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = sogp_node_Datapoint___deserialize(dat__, fid__)
msg__ = sogp_node_Datapoint();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1, 'uint32=>uint32'));
msg__.data = fread(fid__, size__, 'single');
if( file_created__ )
    fclose(fid__);
end
function l__ = sogp_node_Datapoint___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

