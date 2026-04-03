function init_params()
% Build parameter struct from your functions
params = struct();
% params.tyre       = outputTyre();
params.tyre       = outputTyre2();
params.chassis    = outputChassis();
params.aero       = outputAero();
params.pwt        = outputPwt();
params.suspension = outputSus();

% Create Bus object for the whole params struct
busInfo = Simulink.Bus.createObject(params);

% Get the generated bus and rename to stable name
ParamsBus = evalin('base', busInfo.busName);
assignin('base', 'ParamsBus', ParamsBus);

% Create typed parameter
PARAMS = Simulink.Parameter;
PARAMS.Value = params;
PARAMS.DataType = 'Bus: ParamsBus';
PARAMS.CoderInfo.StorageClass = 'Auto';

assignin('base', 'PARAMS', PARAMS);
end
