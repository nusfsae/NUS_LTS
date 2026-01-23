function init_params()
% Build your structs from the 5 functions
params = struct();
params.tyre       = outputTyre();
params.chassis    = outputChassis();
params.aero       = outputAero();
params.pwt        = outputPwt();
params.suspension = outputSus();

% Create Bus objects automatically (top-level + nested)
busInfo = Simulink.Bus.createObject(params);

% busInfo.busName is the variable name created in base workspace
tmpBus = evalin('base', busInfo.busName);

% Rename to a stable name
assignin('base', 'ParamsBus', tmpBus);

% Create ONE typed Simulink.Parameter that holds the entire struct
PARAMS = Simulink.Parameter;
PARAMS.Value = params;
PARAMS.DataType = 'Bus: ParamsBus';
PARAMS.CoderInfo.StorageClass = 'Auto';

assignin('base', 'PARAMS', PARAMS);
end
