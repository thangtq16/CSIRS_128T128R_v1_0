function model = makeCDLChannelAntennaArrayStructure(TransmitAntennaArray,ReceiveAntennaArray,TransmitArrayOrientation,ReceiveArrayOrientation,CarrierFrequency)
%makeCDLChannelAntennaArrayStructure make CDL channel structure containing
%antenna arrays information
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2020 The MathWorks, Inc.

%#codegen
    
    model = struct();
    model.NumInputSignals = getNumSignals(TransmitAntennaArray);
    model.NumOutputSignals = getNumSignals(ReceiveAntennaArray);
    
    % Create Tx antenna array internal model
    taa = getAntennaArray(TransmitAntennaArray,TransmitArrayOrientation,CarrierFrequency);    
    txSize = size(taa.ElementPositions,2:6);
    txSubarraySize = size(taa.SubarrayPositions);
    txElemPattern = wireless.internal.channelmodels.makeElementPattern(TransmitAntennaArray,CarrierFrequency);
    model.TransmitAntennaArray = wireless.internal.channelmodels.emptyAntennaArray(txSize,txSubarraySize,txElemPattern);
    
    % Create Rx antenna array internal model
    raa = getAntennaArray(ReceiveAntennaArray,ReceiveArrayOrientation,CarrierFrequency);
    rxSize = size(raa.ElementPositions,2:6);
    rxSubarraySize = size(raa.SubarrayPositions);
    rxElemPattern = wireless.internal.channelmodels.makeElementPattern(ReceiveAntennaArray,CarrierFrequency);
    model.ReceiveAntennaArray = wireless.internal.channelmodels.emptyAntennaArray(rxSize,rxSubarraySize,rxElemPattern);
    
    % Copy transmit and receive arrays structures into the model at last
    model.ReceiveAntennaArray = mergeStructs(model.ReceiveAntennaArray,raa);
    model.TransmitAntennaArray = mergeStructs(model.TransmitAntennaArray,taa);
    
end

function arrayModel = getAntennaArray(array,orientation,fc)
    
    coder.extrinsic('wireless.internal.channelmodels.makeAntennaArray');
    
    if isstruct(array)
        arrayModel = coder.const(wireless.internal.channelmodels.makeAntennaArray(array,orientation));
    else % PhAST array object
        arrayModel = wireless.internal.channelmodels.makeAntennaArrayPhAST(array,orientation,fc);
    end
    
end

function numSignals = getNumSignals(array)
    
    % Number of input/output signals to/from the channel
    if isstruct(array)
        numSignals = prod(array.Size);
    else % PhAST object
        if isa(array, 'phased.internal.AbstractSubarray')
            numSignals = array.getNumSubarrays;            
        else % phased.internal.AbstractArray
            numSignals = array.getNumElements;
        end
    end
    
end

function s1 = mergeStructs(s1,s2)

    f = fieldnames(s2);
    for i=1:length(f)
        s1.(f{i}) = s2.(f{i});
    end
    
end
