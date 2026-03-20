%hSSBurstInfo Synchronization Signal burst (SS burst) information
%   INFO = hSSBurstInfo(BURST,CBW) creates burst information structure INFO
%   given burst configuration structure BURST and the channel bandwidth CBW
%   in MHz.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   See nr5g.internal.wavegen.hSSBurst for a description of the fields of BURST and INFO.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

function info = hSSBurstInfo(burst,channelBandwidth)
    
    if (channelBandwidth==3)
        % Rel-18 3MHz puncturing modifications
        MinBurstNRBs = 12;
        ssbHalfGrid = 6;
        ssbGridSize = 144;
    else
        MinBurstNRBs = 20;
        ssbHalfGrid = 10;
        ssbGridSize = 240;
    end

    % Get starting symbols of SS blocks in the half frame according to TS
    % 38.213 Section 4.1. The number of SS blocks is denoted L
    ssbStartSymbols = nr5g.internal.wavegen.hSSBurstStartSymbols(burst);
    L = length(ssbStartSymbols);
    
    % Get the half frame number of the SS burst and round the frame number
    % between 0-1023
    n_hf = mod(burst.NHalfFrame,2);
    burst.NFrame = mod(burst.NFrame,1024); % safeguard for NFrame >= 1024
    
    % If the SS burst should be inactive in this half frame according to
    % periodicity, zero the TransmittedBlocks bitmap
    offset = 0;
    if ~isscalar(burst.Period)
        offset = burst.Period(2);
    end
    if (mod(burst.NFrame*10 + n_hf*5 - offset, burst.Period(1))~=0)
        burst.TransmittedBlocks(:) = 0;
    end
    
    % Get subcarrier spacing for SS burst
    [burstSCS,k_SSB_units,NCRB_SSB_units,scsBWP0] = nr5g.internal.wavegen.blockPattern2SCS(burst.BlockPattern,burst.SubcarrierSpacingCommon);
    
    % Calculate number of resource blocks in SS burst numerology needed to
    % achieve the appropriate sampling rate. The largest number of resource
    % blocks yielding the appropriate sampling rate is selected, in order
    % to maximize the flexibility of SS burst placement
    nfft = burst.SampleRate / (burstSCS * 1e3);
    burstNRB = min(275, floor(nfft * 0.85 / 12));
    coder.internal.errorIf(burstNRB < MinBurstNRBs, ...
        'nr5g:nrWaveformGenerator:MaxSSBNRBTooSmall', sprintf('%0.2f', burst.SampleRate/1e6),burstSCS,burstNRB);
    
    % create nrCarrierConfig object, as flat nrOFDMInfo signature has
    % extra coder.const requirements in codegen
    c = nrCarrierConfig('SubcarrierSpacing', burstSCS, ...
                        'NSizeGrid', burstNRB);
    ofdmInfo = nrOFDMInfo(c);
    
    % Validate FrequencyPointA
    k_freqPointA = burst.FrequencyPointA / (burst.SubcarrierSpacingCommon * 1e3);
    coder.internal.errorIf(mod(k_freqPointA,1)~=0, ...
        'nr5g:nrWaveformGenerator:PointANotMultiple',  burst.SubcarrierSpacingCommon);
    
    % Calculate NCRB_SSB, k_SSB and FrequencyOffsetSSB
    coder.internal.errorIf(mod(burst.FrequencySSB,5e3)~=0, ...
        'nr5g:nrWaveformGenerator:FreqSSBMul5khz');
    ssbf0 = burst.FrequencySSB - ((12*ssbHalfGrid) * burstSCS * 1e3);
    delta_f = ssbf0 - burst.FrequencyPointA;

    % NCRB_SSB is the CRB, in SubcarrierSpacingCommon SCS, containing the
    % first SSB carrier. NCRB_SSB is expressed in terms of RBs with
    % 'NCRB_SSB_units' SCS (which may be different from
    % SubcarrierSpacingCommon)
    NCRB_SSB = floor(delta_f / (12 * NCRB_SSB_units * 1e3));    

    % Adjust the value of the RB offset NCRB_SSB (TS 38.211 Section 7.4.3):
    % "the centre of subcarrier 0 of resource block NCRB_SSB coincides with
    % the centre of subcarrier 0 of a common resource block with the
    % subcarrier spacing:
    %   * provided by the higher-layer parameter subCarrierSpacingCommon
    %   for operation without shared spectrum channel access in FR1 and
    %   FR2-1; and
    %   * same as the subcarrier spacing of the SS/PBCH block for operation
    %   without shared spectrum access in FR2-2 and for operation with
    %   shared spectrum channel access.
    % 
    % This common resource block overlaps with subcarrier 0 of the first
    % resource block of the SS/PBCH block."
    scsRatio = scsBWP0 / NCRB_SSB_units;
    NCRB_SSB = NCRB_SSB - mod(NCRB_SSB,scsRatio);

    % Remove RB offset from delta_f
    delta_f = delta_f - (NCRB_SSB * 12 * NCRB_SSB_units * 1e3);

    % k_SSB is the offset between subcarrier 0 of CRB NCRB_SSB and the
    % first SSB subcarrier, k_SSB is signalled in the MIB.
    k_SSB = floor(delta_f / (k_SSB_units * 1e3));
    kStep = burstSCS / k_SSB_units;

    % Remove subcarrier offset from delta_f
    delta_f = delta_f - ((k_SSB-mod(k_SSB,kStep)) * k_SSB_units * 1e3);
    
    % frequencyOffsetSSB is the part of the offset between the carrier and
    % SSB locations that is not a multiple of the SSB SCS
    frequencyOffsetSSB = delta_f;
    % For some parameter combinations, the frequency of the first SSB
    % subcarrier (ssbf0) will not be an integer multiple of the SSB SCS. In
    % this case, adjust the SSB frequency to lie on the SSB grid
    delta_f = mod(ssbf0,burstSCS * 1e3);
    ssbf0 = ssbf0 - delta_f;
    
    % Calculate offset to the subcarrier in the resource grid which 
    % represents SSB subcarrier k=0
    k0_offset = burstNRB*12/2 + (ssbf0 / (burstSCS * 1e3));
    gridFreqSpan = burstNRB * 12 / 2 * burstSCS * 1e3;
    coder.internal.errorIf( (k0_offset < 0) || (k0_offset > (burstNRB*12 - ssbGridSize)), ...
        'nr5g:nrWaveformGenerator:SSBNotSpannedByGrid', burstNRB, sprintf('%0.3f', -gridFreqSpan/1e6), sprintf('%0.3f', (gridFreqSpan - (burstSCS*1e3))/1e6), burstSCS, sprintf('%0.3f', (burst.FrequencySSB - (12*ssbHalfGrid) * burstSCS *1e3)/1e6), sprintf('%0.3f', (burst.FrequencySSB + (12*ssbHalfGrid - 1) * burstSCS *1e3)/1e6));
    
    % Create Master Information Block (MIB) bit payload
    if isnumeric(burst.DataSource)
      mib = burst.DataSource(:);
      
    elseif strcmpi(burst.DataSource, 'MIB')
      mib = zeros(24,1);
      SFN = dec2bin(burst.NFrame,10)=='1';
      mib(2:7) = SFN(1:6);
      if (L==64)
          mib(8) = (burst.SubcarrierSpacingCommon == 120);
      else
          mib(8) = (burst.SubcarrierSpacingCommon == 30);
      end
      mib(9:12) = dec2bin(mod(k_SSB,16),4)=='1';
      mib(13) = (burst.DMRSTypeAPosition == 3);
      mib(14:21) = dec2bin(burst.PDCCHConfigSIB1,8)=='1';
      mib(22) = burst.CellBarred;
      mib(23) = burst.IntraFreqReselection;
      
    else
      % PN
      datasource = nr5g.internal.wavegen.hVectorDataSource(burst.DataSource, 24); 
      mib = datasource.getPacket(24);
    end

    % Create information output
    info = struct();
    info.SubcarrierSpacing = burstSCS;
    info.NCRB_SSB = NCRB_SSB;
    info.k_SSB = k_SSB;
    info.FrequencyOffsetSSB = frequencyOffsetSSB;
    info.MIB = mib;
    info.L = L;
    tmp = find(burst.TransmittedBlocks) - 1;
    info.SSBIndex = tmp;
    if (L==4)
        tmp2 = mod(tmp,4);
        info.i_SSB = tmp2;
        info.ibar_SSB = tmp2 + 4*n_hf;
    else
        info.i_SSB = mod(tmp,8);
        info.ibar_SSB = mod(tmp,8);
    end
    info.SampleRate = ofdmInfo.SampleRate;
    info.Nfft = double(ofdmInfo.Nfft);
    info.NRB = burstNRB;
    info.CyclicPrefix = 'Normal';
    info.OccupiedSubcarriers = k0_offset + (1:ssbGridSize).';
    
    a = 1:4;
    b = ssbStartSymbols(logical(burst.TransmittedBlocks));
    info.OccupiedSymbols = nr5g.internal.wavegen.addRowAndColumn(a, b');
    info.Windowing = ofdmInfo.Windowing;
end