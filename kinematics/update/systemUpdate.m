function dataUpdate = systemUpdate(conditions)
%SYSTEMUPDATE  Selects the system state update function handle based on environment/fault conditions.
%
%   dataUpdate = systemUpdate(conditions)
%   Returns a function handle for updating ship/environment states, automatically selecting the appropriate
%   update logic based on the scenario (e.g., with or without unknown disturbances, faults, etc.).
%
%   Inputs:
%     conditions   - Cell array or string array of environment or fault types (e.g., {'winds','currents','unknownDisturbance'})
%                   Allowed values:
%                     'winds', 'waves', 'currents', 'measureNoise',
%                     'sensorFault', 'actuatorFault', 'communicationFault', 'unknownDisturbance'
%
%   Outputs:
%     dataUpdate   - Function handle, pointing to the update function that implements the required logic.
%                   E.g. @updateUsual, @updateUnDis, etc.
%
%   Usage Example:
%     dataUpdate = systemUpdate();
%     dataUpdate = systemUpdate({'unknownDisturbance'});
%
%   Author: Wenxiang Wu
%   Date:   2025-06-03

if isempty(conditions)
    dataUpdate = @updateUsual;
else
    allowed_strings = ["winds", "waves", "currents", "measureNoise", "sensorFault", "actuatorFault", "communicationFault","unknownDisturbance"];
    invalid_conditions = conditions(~ismember(conditions, allowed_strings));
    if ~isempty(invalid_conditions)
        error('Error: The following conditions are not allowed: %s', strjoin(invalid_conditions, ', '));
    end
    if ismember("unknownDisturbance", conditions)==true
        dataUpdate = @updateUnDis;
    end
end
end

