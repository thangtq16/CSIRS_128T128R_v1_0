function srsObj = getSRSObject(waveSRS)
%getSRSObject Creates nrSRSConfig object
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   SRSOBJ = getSRSObject(WAVESRS) provides the SRS configuration object
%   nrSRSConfig SRSOBJ, given the input nrWavegenSRSConfig object WAVESRS.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    % Initialize for codegen
    coder.varsize('srsObj',[1,inf],[0,1]);
    if isempty(waveSRS.Period)
        numSRS = 1;
    else
        numSRS = nnz(waveSRS.SlotAllocation < waveSRS.Period(1));
    end
    
    % Get the SRS configuration object with the waveSRS input
    srsTemp = {nrSRSConfig('SRSPeriod',[1 0])};
    
    % Common property names to both waveSRS and srsObj objects
    fnames = {'NumSRSPorts','NumSRSSymbols','SymbolStart','KTC','KBarTC',...
        'CyclicShift','FrequencyStart','NRRC','CSRS','BSRS','BHop',...
        'Repetition','GroupSeqHopping','NSRSID','SRSPositioning',...
        'FrequencyScalingFactor','EnableStartRBHopping','StartRBIndex',...
        'EnableEightPortTDM','CyclicShiftHopping','CyclicShiftHoppingID',...
        'CyclicShiftHoppingSubset','HoppingFinerGranularity','CombOffsetHopping',...
        'CombOffsetHoppingID','CombOffsetHoppingSubset','HoppingWithRepetition'};

    % Assign the values of common properties
    for i = 1:numel(fnames)
        srsTemp{1}.(fnames{i}) = waveSRS.(fnames{i});
    end
    
    srsObj = repmat(srsTemp,1,numSRS);
    
    for srsIdx = 1:numSRS        
        % When Period is empty, set the resource type to 'aperiodic'
        % (transmission triggered by DCI). Otherwise, set resource type to
        % 'periodic'
        if ~isempty(waveSRS.Period)
            srsObj{srsIdx}.ResourceType = 'periodic';
            srsObj{srsIdx}.SRSPeriod = [waveSRS.Period(1) waveSRS.SlotAllocation(srsIdx)];
        else
            srsObj{srsIdx}.ResourceType = 'aperiodic';
            srsObj{srsIdx}.SRSPeriod = [1 0];
        end
    end
end