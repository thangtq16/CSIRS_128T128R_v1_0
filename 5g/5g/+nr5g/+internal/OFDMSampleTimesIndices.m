function idx = OFDMSampleTimesIndices(ofdminfo,nslot,sampleTimes,numTimeSamples,toffset)
%OFDMSampleTimesIndices Time indices for OFDM perfect channel estimation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   IDX = OFDMSampleTimesIndices(OFDMINFO,INITIALNSLOT,SAMPLETIMES,NUMTIMESAMPLES,TOFFSET)
%   returns the indices IDX of the first dimension (time) of a propagation
%   channel path gains and sample times corresponding to a set of OFDM
%   symbols. The set of OFDM symbols is determined by the NUMTIMESAMPLES
%   and the OFDMINFO inputs.

%   Copyright 2024 The MathWorks, Inc.

%#codegen

    % Establish the total number of slots spanned by sampleTimes
    samplesPerSubframe = ofdminfo.SampleRate * 1e-3;    
    ofdmInfoOutput = nr5g.internal.OFDMInfoOutput(ofdminfo);
    symbolLengths = reshape(ofdmInfoOutput.SymbolLengths,ofdminfo.SymbolsPerSlot,ofdminfo.SlotsPerSubframe).'; % resize to slots by OFDM symbols
    samplesPerSlot = sum(symbolLengths,2);                       % samples per slot in a subframe
    samplesPerSlot = circshift(samplesPerSlot,nslot);            % cyclic shift to account for starting slot number
    numFullSubframes = floor(numTimeSamples/samplesPerSubframe); % number of full subframes in numTimeSamples
    y = rem(numTimeSamples,samplesPerSubframe);                  % remaining number of slots
    nSlots = ofdminfo.SlotsPerSubframe*numFullSubframes;         % account for slots in full subframes in numTimeSamples
    if y>0
        tmp = find(cumsum([0;samplesPerSlot])<=y,1,'last')-1;
        nSlots = tmp(1) + nSlots;
    end

    
    
    % Establish the sample indices at the center of each OFDM
    % symbol across the total number of subframes, taking into
    % consideration the initial slot number, and update the cyclic
    % prefix lengths to span all subframes
    cpLengths = nr5g.internal.OFDMInfoRelativeNSlot(ofdminfo,nslot,nSlots * ofdminfo.SymbolsPerSlot);
    cpLengths = cpLengths(1:nSlots*ofdminfo.SymbolsPerSlot);
    fftLengths = [0 repmat(ofdminfo.Nfft,1,numel(cpLengths)-1)];
    symbolCenters = cumsum(cpLengths + fftLengths) + ofdminfo.Nfft/2;

    % Set the origin of the sample times to zero, and establish the range
    % of possible durations 'T_min'...'T_max' corresponding to the sample
    % times
    sampleTimes = sampleTimes - sampleTimes(1);
    T_cg = mean(diff(sampleTimes));
    Ncs = length(sampleTimes);
    if (Ncs > 1)
        t = [0; sampleTimes + T_cg/2];
    else
        t = sampleTimes;
    end
    symbolCenterTimes = (symbolCenters + toffset)/ ofdminfo.SampleRate;
    idx = sum(symbolCenterTimes>=t,1);
    
end