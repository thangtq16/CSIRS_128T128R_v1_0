function meas = nrSSBMeasurements(ssbGrid,nCellID,varargin)
%nrSSBMeasurements SSB-based physical layer measurements
%   MEAS = nrSSBMeasurements(SSBGRID,NCELLID) returns physical layer
%   measurements based on synchronization signal block (SSB), as defined in
%   TS 38.215 Sections 5.1.1 and 5.1.3, for received synchronization signal
%   grid SSBGRID and physical layer cell identity NCELLID. The returned
%   structure MEAS contains the reference signal received power (RSRP),
%   received signal strength indicator (RSSI), and reference signal
%   received quality (RSRQ). The RSRP measurement is based on the secondary
%   synchronization signal (SSS) in the SSBGRID.
%
%   The RSSI measurement bandwidth is 20 resource blocks and
%   measurement time resources are 4 OFDM symbols.
%
%   SSBGRID is a complex double or single 240-by-4-by-R array that consists
%   of a single synchronization signal (SS) block that is received across multiple receive antennas.
%   R is the number of receive antennas.
%
%   NCELLID represents the physical layer cell identity and is an integer
%   from 0 to 1007.
%
%   MEAS is a structure with the fields:
%   RSRPPerAntenna - Column vector of RSRP values in dBm relative to 1
%                    milliwatt in 1 Ohm. Each row corresponds to a receive
%                    antenna.
%   RSSIPerAntenna - Column vector of RSSI values in dBm relative to 1
%                    milliwatt in 1 Ohm. Each row corresponds to a receive
%                    antenna.
%   RSRQPerAntenna - Column vector of RSRQ values in dB. Each row
%                    corresponds to a receive antenna.
%
%   MEAS = nrSSBMeasurements(SSBGRID,NCELLID,IBARSSB) also specifies
%   IBARSSB to include the physical broadcast channel (PBCH) demodulation
%   reference signal (DM-RS) in the RSRP measurement.
%   
%   IBARSSB represents the time-dependent part of the DM-RS scrambling
%   initialization based on SS/PBCH block index and half-frame number and
%   is an integer from 0 to 7.
%
%   MEAS = nrSSBMeasurements(...,NAME=VALUE) specifies additional option as
%   NAME=VALUE pair to enable the phase correction:
%
%   'EnablePhaseCorrection' - 0 to disable the phase correction (default)
%                             1 to enable the phase correction
%
%   % Example: Calculate physical layer measurements of an SS block
%     
%   % Generate an SS block
%   ssblock = complex(zeros([240 4]));
%     
%   % Create and set the PSS for a given cell identity
%   ncellid = 17;
%   pssSymbols = nrPSS(ncellid);
%   ssblock(nrPSSIndices) = pssSymbols;
%     
%   % Create and set the SSS for a given cell identity
%   sssSymbols = nrSSS(ncellid);
%   ssblock(nrSSSIndices) = sssSymbols;
%     
%   % Create and set the PBCH given random codewords, cell identity, and
%   % scrambling sequence phase
%   cw = randi([0 1],864,1);
%   v = 0;
%   pbchSymbols = nrPBCH(cw,ncellid,v);
%   pbchIndices = nrPBCHIndices(ncellid);
%   ssblock(pbchIndices) = pbchSymbols;
%     
%   % Create and set the PBCH DM-RS given cell identity and the time-
%   % dependent part of DM-RS scrambling initialization
%   ibarssb = 0;
%   dmrsSymbols = nrPBCHDMRS(ncellid,ibarssb);
%   dmrsIndices = nrPBCHDMRSIndices(ncellid);
%   ssblock(dmrsIndices) = dmrsSymbols;
%    
%   % Apply power scaling to SS block given Es/N0 (SNR) in dB and N0 in dBm
%   EsN0dB = 10;
%   N0dBm = -94.65;
%   N0 = 10^((N0dBm-30)/10);
%   EsN0 = 10^(EsN0dB/10);
%   Es = EsN0*N0;
%   ssblock = ssblock*sqrt(Es);
%     
%   % OFDM modulate SS block
%   scs = 15;
%   initialNSlot = 0;
%   [txWaveform,info] = nrOFDMModulate(ssblock,scs,initialNSlot);
%     
%   % Apply awgn noise
%   noise = sqrt(N0/(2*info.Nfft)).*randn(size(txWaveform),'like',1j);
%   rxWaveform = txWaveform + noise;
%     
%   % OFDM demodulate waveform
%   nrb = 20;
%   rxSSBGrid = nrOFDMDemodulate(rxWaveform,nrb,scs,initialNSlot);
%     
%   % Measure SS block signal quality based on SSS
%   meas1 = nrSSBMeasurements(rxSSBGrid,ncellid)
%     
%   % Measure SS block signal quality based on SSS and PBCH DM-RS 
%   meas2 = nrSSBMeasurements(rxSSBGrid,ncellid,ibarssb)
%   
%   See also nrSSS, nrPBCHDMRS, nrCSIRSMeasurements.

% Copyright 2021-2024 The MathWorks, Inc.
    
%#codegen
    narginchk(2,5)
    persistent sssInd

    fcnName = 'nrSSBMeasurements';
    validateattributes(ssbGrid,{'double','single'},{'3d','size',...
        [240 4 NaN],'finite'},fcnName,'SSBGRID');
    
    % Define persistent inputs
    if isempty(sssInd)
        sssInd = nrSSSIndices;         
    end

    % Generate SSS symbols
    refSSS=nrSSS(nCellID);
    
    refPBCHDMRS = zeros(0,1);
    pbchDMRSInd = zeros(0,1,'uint32');
    firstoptarg = 1;
    % Generate PBCH DM-RS reference if ibarssb is present and non-empty
    if any(nargin == [3 5])
        if ~isempty(varargin{1})
            refPBCHDMRS = nrPBCHDMRS(nCellID,varargin{1});
            pbchDMRSInd = nrPBCHDMRSIndices(nCellID);
            numSSS = numel(sssInd);
            numPBCHDMRS = numel(pbchDMRSInd);
        end
        firstoptarg = 2;
    end
    % Parse options
    enablePhaseCorrection = false;
    if any(nargin == [4 5])
        opts = nr5g.internal.parseOptions(fcnName,{'EnablePhaseCorrection'},varargin{firstoptarg:end});
        enablePhaseCorrection = opts.EnablePhaseCorrection;
    end
    
    numRx = size(ssbGrid,3); % Number of receive antennas
    meas = struct('RSRPPerAntenna',zeros(numRx,1),...
        'RSSIPerAntenna',zeros(numRx,1),...
        'RSRQPerAntenna',zeros(numRx,1));

    % Generate the reference grid for the channel estimation
    rsIndices = [sssInd; pbchDMRSInd];
    rsSymbols = [refSSS; refPBCHDMRS];
    refGrid = zeros([240 4],like=1i);
    refGrid(rsIndices) = rsSymbols;
    % Perform the phase correction, if enabled
    if any(nargin == [4 5]) && enablePhaseCorrection
        % Perform the channel estimation and extract the phase information
        H = nrChannelEstimate(ssbGrid,refGrid);
        theta = unwrap(angle(H));
        % Perform the phase compensation
        ssbGrid = ssbGrid .* exp(-1i*theta);
    end

    for rxAntIdx = 1:numRx
        ssblock = ssbGrid(:,:,rxAntIdx);
        % RSSI - Linear average of power per OFDM Symbol
        meas.RSSIPerAntenna(rxAntIdx) = sum(abs(ssblock(:)).^2)/4;
        
        % RSRP - Use reference symbols to get most accurate power readings
        meas.RSRPPerAntenna(rxAntIdx) = abs(mean(ssblock(sssInd).*conj(refSSS))).^2;
        % Include PBCH DM-RS in RSRP measurement if ibarssb is present and
        % non-empty
        if any(nargin == [3 5]) && ~isempty(varargin{1})
            rsrpDMRS = abs(mean(ssblock(pbchDMRSInd).*conj(refPBCHDMRS))).^2;
            meas.RSRPPerAntenna(rxAntIdx) = (meas.RSRPPerAntenna(rxAntIdx)*numSSS ...
                + rsrpDMRS*numPBCHDMRS)/(numSSS+numPBCHDMRS);
        end  
    end
    % RSRQ = N*RSRP/RSSI where N is the number of resource blocks used in
    % the RSSI measurement. For an SS block, N will always be 20.
    meas.RSRQPerAntenna = 20.*meas.RSRPPerAntenna./meas.RSSIPerAntenna;

    meas.RSRQPerAntenna(isnan(meas.RSRQPerAntenna)) = 0;

    meas.RSRPPerAntenna = 10*log10(meas.RSRPPerAntenna) + 30;
    meas.RSSIPerAntenna = 10*log10(meas.RSSIPerAntenna) + 30;
    meas.RSRQPerAntenna = 10*log10(meas.RSRQPerAntenna);

end