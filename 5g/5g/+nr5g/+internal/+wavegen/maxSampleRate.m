function maxsr = maxSampleRate(cfgObj)
%maxSampleRate Find maximum sample rate for DL or UL 5G waveform
%   Loops over all carriers and finds FFT size to calculate sample rate.
% 
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2022 The MathWorks, Inc.

%#codegen

% If a custom sample rate is not configured, the default sample rate is
% calculated based on the bandwith requirements of all SCS carriers
if isempty(cfgObj.SampleRate) 
    % The highest SCS carrier is centered in the waveform. Calculate
    % resulting point A offset, so that sample rate allows all carriers
    % to fit in waveform with no aliasing.
    %           PointA    WaveCenter
    %             |           |
    %   15 kHz:   |         |-----#-----|
    %   30 kHz:   |     |-----#-----|
    %             | |------ bwnrb ------|

    carriers = cfgObj.SCSCarriers;
    carrierscs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers,'SubcarrierSpacing', 'double');
    [maxscs, maxIdx] = max(carrierscs);
    pointAToCenter = (carriers{maxIdx}.NSizeGrid/2 + carriers{maxIdx}.NStartGrid)*12*maxscs; % kHz
    
    % Start and size of all SCS carrier grids in kHz
    gstart = 12*carrierscs.*nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers,'NStartGrid','double');
    gsize  = 12*carrierscs.*nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers,'NSizeGrid','double');
    
    % Use the max span of the carrier grids to calculate sample rate
    fmin = -pointAToCenter + gstart;
    fmax = fmin + gsize;
    bw = max(abs([fmin;fmax]),[],'all');
    bwnrb = ceil(2*bw/(12*carrierscs(1)));
    
    % IDFT size is a power-of-2 > 128 with a maximum occupancy of 85%
    nfft = max(power(2,ceil(log2(bwnrb*12/0.85))),128);
    
    % OFDM sample rate given by SCS and IDFT size
    maxsr = carrierscs(1)*1e3*nfft;
else % Custom sample rate
    maxsr = cfgObj.SampleRate(1);
end
