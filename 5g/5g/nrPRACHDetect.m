function [indout,offset,detinfo] = nrPRACHDetect(carrier,prach,waveform,varargin) 
%nrPRACHDetect Detect physical random access channel
%   [INDOUT,OFFSET,DETINFO] = nrPRACHDetect(CARRIER,PRACH,WAVEFORM) detects
%   physical random access channel (PRACH) transmission in the input
%   WAVEFORM and returns the detected PRACH preamble index, INDOUT, the
%   timing offset, OFFSET, and the detection information, DETINFO. The
%   function generates an internal reference waveform for default PRACH
%   preamble indices 0:63. The function then correlates the input WAVEFORM
%   with this internal reference waveform and searches for correlation
%   output peaks that are greater than the default detection threshold. The
%   function uses the position of the strongest peak in the correlator
%   output to determine the detected preamble index and its associated
%   timing offset.
%
%   CARRIER is a carrier configuration object, <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>.
%   Only these object properties are relevant for this function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15, 30, 60, 120, 240, 480, 960)
%   CyclicPrefix      - Cyclic prefix ('normal', 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%
%   PRACH is a PRACH configuration object, <a href="matlab:help('nrPRACHConfig')">nrPRACHConfig</a>.
%   All the object properties are relevant for this function, except for
%   PreambleIndex.
%
%   WAVEFORM is a time-domain waveform, specified as an N-by-P matrix. N is
%   the number of time-domain samples and P is the number of receive
%   antennas. If N is less than the minimum samples needed to analyze this
%   configuration, the function appends zeros at the end of the waveform.
%   If WAVEFORM contains multiple PRACH instances, the function returns the
%   preamble index, INDOUT, and timing offset, OFFSET, related to the PRACH
%   instance with the strongest peak in the correlation.
%
%   Output INDOUT is the PRACH preamble index corresponding to the
%   strongest correlation peak across all the preamble indices in the
%   PreambleIndex name-value argument. If such correlation peak does not
%   exist, INDOUT is empty.
%
%   Output OFFSET is the timing offset of the PRACH waveform from the
%   origin of the input WAVEFORM, returned in samples at the sample rate of
%   WAVEFORM. The timing offset is a real number. The integer part is the
%   sample position of the strongest correlation peak. The fractional part
%   is the fractional delay present in the correlation peak due to the
%   cyclic shift in the frequency domain. When INDOUT is empty, OFFSET is
%   empty.
%
%   DETINFO is a structure containing these fields:
%
%   CorrelationPeaks   - Strongest correlation peak values, where each
%                        value corresponds to an index in the PreambleIndex
%                        name-value argument.
%   DetectionThreshold - Detection threshold that the function uses for
%                        correlation.
%
%   [INDOUT,OFFSET,DETINFO] = nrPRACHDetect(...,NAME,VALUE) specifies
%   additional options as NAME,VALUE arguments to allow control over the
%   detection threshold and the PRACH preamble indices to use in the
%   correlation:
%
%   DetectionThreshold - Specifies the detection threshold as a real number
%                        in the range [0, 1]. When this input is not present
%                        or set to [], the function selects a default value
%                        specific to the PRACH format, LRA value, and
%                        number of receive antennas. For more information,
%                        see <a href="matlab:doc('nrPRACHDetect')">nrPRACHDetect</a>.
%   PreambleIndex      - Specifies the set of PRACH preamble indices within
%                        the cell for the detection as an array of length
%                        between 0 and 64, containing integers in the range
%                        [0, 63]. When this input is not present or set to
%                        [], the function uses the default value of 0:63.
%
%   Example:
%   % Detect a PRACH preamble format 0 with a timing offset of 7 samples.
%   
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 6;
%   prach = nrPRACHConfig;
%   prach.ConfigurationIndex = 27;
%   prach.ZeroCorrelationZone = 1;
%   prach.PreambleIndex = 44;
%   prachGrid = nrPRACHGrid(carrier, prach);
%   prachSymbols = nrPRACH(carrier, prach);
%   prachIndices = nrPRACHIndices(carrier, prach);
%   prachGrid(prachIndices) = prachSymbols;
%   tx = nrPRACHOFDMModulate(carrier, prach, prachGrid);
%   rx = [zeros(7,1); tx]; % delay PRACH
%   [index, offset] = nrPRACHDetect(carrier, prach, rx)
%
%   See also nrPRACHConfig, nrPRACH, nrPRACHIndices, nrPRACHGrid,
%   nrPRACHOFDMInfo, nrPRACHOFDMModulate.

%   Copyright 2019-2022 The MathWorks, Inc.

%   References:
%   [1] Sesia, S., Toufik, I., and Baker, M., "LTE - The UMTS Long Term
%   Evolution: From Theory to Practice", Wiley 2011. Chapter 17.5.2.

%#codegen

    narginchk(3,7);
    
    % Parse and validate inputs
    [prach,indin,threshold,prachDuration] = parseAndValidateInputs(carrier,prach,waveform,varargin{:});

    % Pre-configure empty outputs
    indout = zeros(0,1);
    offset = zeros(0,1);
    detinfo = struct('DetectionThreshold',threshold,'CorrelationPeaks',zeros(0,1));
    coder.varsize('detinfo.CorrelationPeaks',[64,1],[1,0]);
    
    % Store most used PRACH properties for ease of access
    LRA = prach.LRA;
    
    % Extract PRACH-related information, prachInfo, and OFDM-related
    % information for PRACH, ofdmInfo
    allPreambleIndices = 0:63;
    prachInfo = repmat(struct('RootSequence',nan,'CyclicShift',nan,...
                'CyclicOffset',nan,'NumCyclicShifts',nan),length(allPreambleIndices),1);
    for pIdx = allPreambleIndices
        prach.PreambleIndex = pIdx;
        prachInfo(pIdx+1) = nr5g.internal.prach.getSymbolsInfo(prach);
    end
    ofdmOpts = struct('Windowing',0);
    ofdmInfoTmp = nr5g.internal.prach.OFDMInfo(carrier,prach,ofdmOpts);
    ofdmInfo = nr5g.internal.prach.OFDMInfoOutput(ofdmInfoTmp);
    % Define a scaling factor to upsample the PRACH info values to the OFDM sample rate
    sampleScaling = ofdmInfo.SampleRate/(LRA*prach.SubcarrierSpacing*1e3);
    
    % Find set of root sequences required for set of input preamble indices
    u = unique([prachInfo(indin(:)+1).RootSequence],'stable');
    
    % Configure correlator input dimensions
    ncorrs = prachDuration;
    numOFDMSymbPerSlot = numel(ofdmInfo.SymbolLengths);
    symbLoc = prach.SymbolLocation - numOFDMSymbPerSlot*prach.ActivePRACHSlot;
    start = ofdmInfo.OffsetLength + sum(ofdmInfo.SymbolLengths(1:symbLoc));
    if (numOFDMSymbPerSlot > 0) && nr5g.internal.prach.isActive(prach)
        start = start + ofdmInfo.CyclicPrefixLengths(symbLoc+1);
        duration = ofdmInfo.Nfft;
    else
        % PRACH is not active in the current slot
        return
    end
    
    % Extract number of receive antennas and number of unique root sequences
    NRxAnts = size(waveform,2);
    numRootSeq = length(u);

    % Pad the input waveform with zeros, if it has less samples than the
    % minimum amount needed for the correlation
    numSamplesNeeded = start+ncorrs*duration;
    if size(waveform,1)<numSamplesNeeded
        waveform = [waveform; zeros(numSamplesNeeded-size(waveform,1),NRxAnts)];
    end
    
    % Initialize parameters
    c = repmat({zeros(duration,1)},numRootSeq,1);
    preambleidx = zeros(numRootSeq,1);
    corrPeak = zeros(numRootSeq,1);
    detMet = zeros(length(allPreambleIndices),1);

    % Perform a correlation for each distinct root sequence
    for idx = 1:numRootSeq
        % Find first preamble index preambleidx(idx) associated to root sequence u(idx)
        preambleidxTmp = find([prachInfo.RootSequence]==u(idx))-1;
        preambleidx(idx) = preambleidxTmp(1);
        prach.PreambleIndex = preambleidx(idx);
        info = prachInfo(preambleidx(idx)+1);
        
        % Generate reference PRACH sequence
        refWave = nr5g.internal.prach.generateWaveform(carrier,prach,struct('ofdmInfo',ofdmInfoTmp,'symInfo',info));
        refWave = refWave(start+(1:duration),1);
        refWaveFFT = fft(refWave);
        mask = abs(refWaveFFT)>0.1;
        
        % For each receive antenna
        eRx = 0;
        for p = 1:NRxAnts
            % Perform correlation(s) of input on the pth antenna with
            % reference waveform and store result
            cp = zeros(duration,1);
            for k = 1:ncorrs
                rxWave = waveform(start+((k-1)*duration)+(1:duration),p);
                rxWaveFFT = fft(rxWave);

                % Combine the correlation from each OFDM symbol
                % Note that [1] describes the correlation as the power
                % delay profile (PDP). This is because the correlation
                % represents the impulse response of the channel convolved
                % with the autocorrelation of the sequence used. Thus, if
                % the sequence used is good, the correlator output is a
                % good approximation of the impulse response, and thus the
                % PDP.
                cp = cp + abs(ifft(rxWaveFFT.*conj(refWaveFFT))).^2;

                % Compute the energy of the received waveform for the
                % normalization of the correlation peak
                eRx = eRx + sum(abs(ifft(rxWaveFFT.*mask)).^2);
            end
            
            % Combine correlations from each antenna
            if (p==1)
                c{idx} = cp;
            else
                c{idx} = c{idx}+cp;
            end
        end

        % For restricted sets, determine the size of the cyclic offset, in
        % order to deal with the side peaks caused by loss of orthogonality
        % due to Doppler shift. The side peaks are dealt with by combining
        % each interval of length cyclicOffset in the correlation output.
        if prach.RestrictedSet~="UnrestrictedSet"
            cyclicOffset = fix(info.CyclicOffset*sampleScaling);
            x = c{idx};
            if prach.RestrictedSet=="RestrictedSetTypeA"
                c{idx} = (x((1+cyclicOffset):end)+x(1:(end-cyclicOffset)))/sqrt(2);
            else % RestrictedSetTypeB
                c{idx} = (x((1+2*cyclicOffset):end)+x((1+cyclicOffset):(end-cyclicOffset))+x(1:(end-2*cyclicOffset)))/sqrt(3);
            end
        end

        % Normalize the correlation against the energy of the received and
        % reference waveforms. Add eps to the normalization to avoid
        % dividing by 0.
        eRef = sum(abs(refWave).^2);
        normE = eRx.*eRef;
        c{idx} = c{idx}/(normE+eps);

        % Store correlation peak for this root sequence
        corrPeak(idx) = max(c{idx});
        numPreambles = length(preambleidxTmp);
        detMet(preambleidxTmp+1) = repmat(corrPeak(idx),numPreambles,1);
    end
    
    % Determine the length of the zero correlation zone
    zcz = prachInfo(1).NumCyclicShifts*sampleScaling;
    
    % Specify deadzone as the fraction of the timing window at the end of
    % the timing window for one preamble that will be considered as
    % belonging to the next preamble and having a timing offset of zero.
    % This effectively excludes timing offsets greater than (1.0-deadzone)
    % of the maximum. In cases for which noise has caused the correlation
    % peak to be slightly into the previous preamble's timing window,
    % deadzone ensures detection of preambles with low timing offset. The
    % value of deadzone corresponds to the duration of the main lobe of the
    % autocorrelation of the PRACH. Zero is used for the case in which
    % there is only one preamble per correlation.
    if (zcz~=0)
        deadzone = sampleScaling/zcz;
    else
        deadzone = 0;
    end
    
    % Detect preambles. This implementation detects a single preamble index
    % with the strongest correlation across all correlators, provided a
    % detection threshold is exceeded.
    linearidx = 0:duration-1;
    [bestCorr,idx] = max(corrPeak);

    % If the correlation peak for this root sequence exceeds the detection
    % threshold and is the highest maximum value across correlations
    % checked so far, establish the detected preamble index and timing
    % offset
    if (bestCorr>=threshold)
        % Record the correlation peak position. If there are multiple peaks
        % in the correlation, consider the position of the first peak.
        linearidxTmp = find(c{idx}==bestCorr,1); % First index of the correlation peak equal to bestCorr
        maxpos = mod(linearidx(linearidxTmp(1))+(deadzone*zcz),length(c{idx}))-(deadzone*zcz);

        % Establish the preamble index and timing offset from the
        % correlation peak position
        info = prachInfo(preambleidx(idx)+1);
        if (info.NumCyclicShifts==0)
            indout = preambleidx(idx);
            offset = maxpos;
        else
            % Find the set of cyclic shifts v = 0...maxv for this root
            % sequence
            maxv = find([prachInfo.RootSequence]==u(idx),1,'Last')-preambleidx(idx)-1;
            cyclicShift = mod(LRA-[prachInfo(preambleidx(idx)+(1:(maxv+1))).CyclicShift],LRA)*sampleScaling;

            % Establish the set of offsets from the peak correlation
            % position to the set of cyclic shifts for this root sequence
            offsetpos = maxpos-cyclicShift;
            if prach.RestrictedSet~="UnrestrictedSet"
                % For restricted sets, determine the size of the cyclic
                % offset, in order to deal with the side peaks caused by
                % loss of orthogonality due to Doppler shift
                cyclicOffset = info.CyclicOffset*sampleScaling;
                if prach.RestrictedSet=="RestrictedSetTypeA"
                    offsetpos = [offsetpos-cyclicOffset offsetpos offsetpos+cyclicOffset];
                else % RestrictedSetTypeB
                    offsetpos = [offsetpos-2*cyclicOffset offsetpos-cyclicOffset offsetpos offsetpos+cyclicOffset offsetpos+2*cyclicOffset];
                end
            end

            % Find the value of v for the detected preamble and compute the
            % final preamble index
            vdash = find(floor(offsetpos/zcz + deadzone)==0)-1;
            if (~isempty(vdash))
                vdash = vdash(1);
            end
            v = mod(vdash,maxv+1);
            indout = preambleidx(idx)+v;

            % Establish the timing offset from the correlation peak
            % position
            offset = offsetpos(vdash+1);
            offset = max(offset, 0);
        end
    end

    % Populate the output detection information structure
    detinfo.CorrelationPeaks = detMet(indin+1);

    % Remove output values if the detected preamble is not part of the
    % user-defined preamble index range.
    if(~isempty(indout)) && (~any(indout(1)==indin))
        indout = zeros(0,1);
        offset = zeros(0,1);
    end

end

% Parse and validate inputs
function [prach,indin,thres,prachDuration] = parseAndValidateInputs(carrier,prach,waveform,varargin)

    % Validate carrier and PRACH inputs
    fcnName = 'nrPRACHDetect';
    prach = nr5g.internal.prach.validatePRACHAndCarrier(carrier,prach,fcnName);

    % Validate waveform
    validateattributes(waveform,{'numeric'},{'2d','finite','nonnan'},fcnName,'Waveform');

    % Get the number of OFDM symbols in each PRACH preamble
    prachDuration = prach.PRACHDuration;
    % For format C2, the last OFDM symbol in each time occasion is
    % empty, while the first symbol is entirely used by the cyclic
    % prefix. Therefore, the length of the preamble is two symbols
    % shorter than the value of PRACHDuration, as shown in the N_u
    % entry from TS 38.211 Table 6.3.3.1-2.
    if prach.Format=="C2"
        prachDuration = prachDuration - 2;
    end

    % Parse optional inputs
    opts = nr5g.internal.parseOptions(fcnName,...
        {'PreambleIndex','DetectionThreshold'},varargin{:});

    % Get the value of the PRACH preamble indices within the cell for which
    % to search
    if isempty(opts.PreambleIndex)
        indin = 0:63; % Default value
    else
        indin = double(opts.PreambleIndex);
    end

    % Get the value of the detection threshold
    if isempty(opts.DetectionThreshold)
        % The default value of the detection threshold has been empirically
        % determined based on the probability of false alarm and the
        % probability of detection for the conformance tests discussed in
        % TS 38.141-1 Section 8.4.
        NumRx = size(waveform,2); % Number of receive antennas
        thresBaseSharedSpectrum = (0.01/sqrt(NumRx*prachDuration)+0.0005);
        switch prach.LRA
            case 839 % Long preamble
                thres = 0.02/sqrt(NumRx*prachDuration);
            case 139 % Short preamble
                thres = 0.1/sqrt(NumRx*prachDuration);
            case 1151 % Short preamble, shared spectrum
                thres = thresBaseSharedSpectrum;
            otherwise % LRA=571, short preamble, shared spectrum
                % For LRA=571, the threshold value is double that of
                % LRA=1151.
                thres = 2*thresBaseSharedSpectrum;
        end
    else
        thres = double(opts.DetectionThreshold);
    end

end
