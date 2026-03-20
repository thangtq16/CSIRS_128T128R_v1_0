classdef EnablePower
%EnablePower Class offering 2 common knobs to class extensions for waveform generation 
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2020-2024 The MathWorks, Inc.

%#codegen

    % Public properties
    properties

        %Enable Flag to turn channel or signal on or off in waveform generation
        % Specify Enable as a logical scalar. This flag determines the
        % presence of this channel or signal in the generated 5G waveform. The
        % default is true.
        Enable (1,1) logical = true;

        %Power Power scaling in dB
        % Specify Power in dB as a real scalar or row vector that expresses the amount by
        % which this channel or signal is scaled. If Power is a vector, it
        % must be the same size as the number of allocated slots within a
        % period. The default is 0 dB.
        Power (1,:) {mustBeNumeric, mustBeReal, mustBeFinite} = 0;
    end

    methods (Access = protected)
        function validatePower(obj,SlotAllocation,Period)
            % By default, Power must be either a scalar or a vector with as
            % many elements as the number of allocated slots in a period.
            if ~isscalar(obj.Power)
                % If an empty period is provided, the number of slots in a
                % period is just the number of allocated slots. Otherwise,
                % it is the number of allocated slots which are less than
                % the period.
                uniqueAllocatedSlots = unique(SlotAllocation);
                if isempty(Period)
                    numSlotsInPeriod = numel(uniqueAllocatedSlots);
                else
                    numSlotsInPeriod = nnz(uniqueAllocatedSlots<Period);
                end
                coder.internal.errorIf(numel(obj.Power)~=numSlotsInPeriod, 'nr5g:nrWaveformGenerator:InvalidPower', numel(obj.Power), numSlotsInPeriod);
            end
        end
    end

end
