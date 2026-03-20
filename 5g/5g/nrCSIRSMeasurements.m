function meas = nrCSIRSMeasurements(carrier,csirs,grid,varargin)
%nrCSIRSMeasurements CSI-RS-based physical layer measurements
%   MEAS = nrCSIRSMeasurements(CARRIER,CSIRS,GRID) returns physical layer
%   measurements based on the channel state information reference signal
%   (CSI-RS), as defined in TS 38.215 Sections 5.1.2 and 5.1.4, for carrier
%   configuration parameters CARRIER, CSI-RS configuration parameters
%   CSIRS, and received grid GRID. The returned structure MEAS contains the
%   reference signal received power (RSRP), received signal strength
%   indicator (RSSI), and reference signal received quality (RSRQ).
%
%   CARRIER is an <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> object.
%   This function uses only these object properties:
%
%   SubcarrierSpacing     - Subcarrier spacing in kHz
%   CyclicPrefix          - Cyclic prefix type
%   NSlot                 - Absolute slot number
%   NFrame                - Absolute system frame number
%   NSizeGrid             - Size of the carrier resource grid in terms of
%                           number of resource blocks (RBs)
%   NStartGrid            - Starting RB index of the carrier resource grid
%                           relative to common resource block 0 (CRB 0)
%
%   CSIRS is an <a href="matlab:help('nrCSIRSConfig')">nrCSIRSConfig</a> object.
%   This function uses only these object properties:
%
%   CSIRSType             - CSI-RS resource type: 'zp' or 'nzp'
%   The RSRP for a resource with CSIRSType='zp' is -Inf dBm (0
%   milliwatts).
%
%   CSIRSPeriod           - CSI-RS slot periodicity and offset
%   The RSRP for a resource with CSIRSPeriod='off' is -Inf dBm (0
%   milliwatts).
%
%   RowNumber             - Row number corresponding to a CSI-RS
%                           resource, as defined in TS 38.211
%                           Table 7.4.1.5.3-1
%   Density               - CSI-RS resource frequency density
%   SymbolLocations       - Time-domain locations of a CSI-RS resource
%   SubcarrierLocations   - Frequency-domain locations of a CSI-RS
%                           resource
%   NumRB                 - Number of RBs allocated for a CSI-RS resource
%   RBOffset              - Starting RB index of CSI-RS allocation
%                           relative to carrier resource grid
%   NID                   - Scrambling identity
%
%
%   GRID is a K-by-L-by-R array of resource elements, for one slot
%   across all receive antennas. K is the number of subcarriers, L
%   is the number of OFDM symbols, and R is the number of
%   receive antennas. L is 14 for normal cyclic prefix and 12 for extended
%   cyclic prefix.
%
%   MEAS is a structure with the fields:
%   RSRPPerAntenna - Matrix of RSRP values in dBm relative
%                    to 1 milliwatt in 1 Ohm. Each row corresponds to a
%                    receive antenna and the columns correspond to the
%                    CSI-RS resources specified in the input CSIRS.
%   RSSIPerAntenna - Matrix of RSSI values in dBm relative
%                    to 1 milliwatt in 1 Ohm. Each row corresponds to a
%                    receive antenna and the columns correspond to the
%                    CSI-RS resources specified in the input CSIRS.
%   RSRQPerAntenna - Matrix of RSRQ values in dB. Each row
%                    corresponds to a receive antenna and the columns
%                    correspond to the CSI-RS resources specified in the
%                    input CSIRS.
%
%   MEAS = nrCSIRSMeasurements(...,NAME=VALUE) specifies additional option
%   as NAME=VALUE pair to enable the phase correction:
%
%   'EnablePhaseCorrection' - 0 to disable the phase correction (default)
%                             1 to enable the phase correction
%
%   % Example: Calculate physical layer measurements of a CSI-RS resource
%
%   % Create carrier configuration object
%   carrier = nrCarrierConfig;
%
%   % Create CSI-RS configuration object
%   csirs = nrCSIRSConfig;
%   csirs.RowNumber = 1;
%   csirs.Density = 'three';
%   csirs.SymbolLocations = 6;
%   csirs.SubcarrierLocations = 0;
%
%   % Generate CSI-RS symbols and indices for the specified configurations
%   ind = nrCSIRSIndices(carrier,csirs);
%   sym = nrCSIRS(carrier,csirs);
%
%   % Initialize the carrier resource grid for one slot
%   ports = csirs.NumCSIRSPorts;
%   txGrid = nrResourceGrid(carrier,ports);
%
%   % Map the CSI-RS symbols
%   txGrid(ind) = sym;
%
%   % Perform OFDM modulation
%   txWaveform = nrOFDMModulate(carrier,txGrid);
%
%   % Apply power scaling to the transmitted waveform
%   EsdBm = -50;
%   rxWaveform = txWaveform * sqrt(10^((EsdBm-30)/10));
%
%   % Perform OFDM demodulation
%   rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
%
%   % Perform CSI-RS measurements
%   meas = nrCSIRSMeasurements(carrier,csirs,rxGrid)
%
%   See also nrCarrierConfig, nrCSIRSconfig, nrSSBMeasurements.

%   Copyright 2019-2024 The MathWorks, Inc.

%#codegen
narginchk(3,5)

% Validate inputs
fcnName = 'nrCSIRSMeasurements';
validateattributes(carrier,{'nrCarrierConfig'},{'scalar'}, ...
    fcnName,'CARRIER');
validateattributes(csirs,{'nrCSIRSConfig'},{'scalar'}, ...
    fcnName,'CSIRS');
validateattributes(grid,{'double','single'},{'3d', ...
    'finite','size',[double(carrier.NSizeGrid)*12 ...
    carrier.SymbolsPerSlot NaN]},fcnName,'GRID');
% Parse options
enablePhaseCorrection = false;
if nargin > 3
    opts = nr5g.internal.parseOptions(fcnName,{'EnablePhaseCorrection'},varargin{:});
    enablePhaseCorrection = opts.EnablePhaseCorrection;
end

% Validate CSI-RS Config
csirsInfo = validateConfig(csirs);

% Get the number of CSI-RS resources
numCSIRSREs = numel(csirsInfo.CSIRSType);

% Number of receive antennas
numRx = size(grid,3);

% Initialize output structure for measurements
meas = struct('RSRPPerAntenna',zeros(numRx,numCSIRSREs),...
    'RSSIPerAntenna',zeros(numRx,numCSIRSREs),...
    'RSRQPerAntenna',zeros(numRx,numCSIRSREs));

% Reference indices and symbols generation
[refIndLin,info] = nrCSIRSIndices(carrier,csirs,'OutputResourceFormat','cell');
refSymCell = nrCSIRS(carrier,csirs,'OutputResourceFormat','cell');
for resIdx = 1:numCSIRSREs % Loop over all CSI-RS resources
    numPorts = csirsInfo.NumCSIRSPorts(resIdx);
    refSymPerResourceTmp = refSymCell{resIdx == info.ResourceOrder};
    
    if any(refSymPerResourceTmp) % Checks if NZP CSI-RS resource is present and non-empty
        refSymPerResource = reshape(refSymPerResourceTmp,[],numPorts);
        refIndPerResourceTmp = refIndLin{resIdx == info.ResourceOrder};
        refIndPerResource = reshape(refIndPerResourceTmp,[],numPorts);
        l_0 = double(csirsInfo.SymbolLocations{resIdx}(1));
        csirsSymIndices = l_0 + info.LPrime{resIdx} + 1; % 1-based symbol indices
        N = csirsInfo.NumRB(resIdx);
        rbOffset = csirsInfo.RBOffset(resIdx);

        % Estimate the channel and extract the phase information for all
        % receive antennas, if phase correction is enabled
        if nargin > 3 && enablePhaseCorrection
            Hest = nrChannelEstimate(carrier,grid,refIndPerResource,refSymPerResource);
            Hest_MeanAcrossPorts = mean(Hest,4);
            phases = unwrap(angle(Hest_MeanAcrossPorts(:,csirsSymIndices,:)));
        end

        ports = min(numPorts,2); % Port 3000 or ports 3000 and 3001
        refSym = reshape(refSymPerResource(:,1:ports),[],1);
        refInd = double(refIndPerResource(:,1:ports)) - (0:ports-1)* ...
            double(carrier.NSizeGrid)*12*carrier.SymbolsPerSlot; % Port 3000 or ports 3000 and 3001

        for rxAntIdx = 1:numRx % Loop over all receive antennas            
            gridRxAnt = grid(:,:,rxAntIdx);
            % Perform the phase correction, if enabled
            if nargin > 3 &&  enablePhaseCorrection
                theta = phases(:,:,rxAntIdx);
                gridRxAnt(:,csirsSymIndices,:) = gridRxAnt(:,csirsSymIndices,:).*exp(-1i*theta);
            end

            % Extract the received CSI-RS symbols using locally generated
            % CSI-RS indices
            rxSym = reshape(gridRxAnt(refInd),[],1);

            % Calculate CSI-RS physical layer measurements
            meas.RSRPPerAntenna(rxAntIdx,resIdx) = abs(mean(rxSym.*conj(refSym))*ports)^2;

            % For RSSI measurement, generate the indices of all resource elements in OFDM symbols containing CSI-RS resource for single port (port 3000)
            numCSIRSSym = numel(csirsSymIndices);
            rssiIndices = repmat((1:N*12).' + rbOffset*12,1,numCSIRSSym) + ...
                repmat((csirsSymIndices - 1)*12*double(carrier.NSizeGrid),12*N,1); % 1-based, linear carrier-oriented indices

            % Extract the modulation symbols using the indices, which
            % corresponds to RSSI measurement
            rssiSym = gridRxAnt(rssiIndices);
            meas.RSSIPerAntenna(rxAntIdx,resIdx) = sum(abs(rssiSym(:)).^2)/numCSIRSSym;
            meas.RSRQPerAntenna(rxAntIdx,resIdx) = N*meas.RSRPPerAntenna(rxAntIdx,resIdx)/meas.RSSIPerAntenna(rxAntIdx,resIdx);
        end
    end
end

meas.RSRPPerAntenna(isnan(meas.RSRPPerAntenna)) = 0;
meas.RSSIPerAntenna(isnan(meas.RSSIPerAntenna)) = 0;
meas.RSRQPerAntenna(isnan(meas.RSRQPerAntenna)) = 0;

meas.RSRPPerAntenna = 10*log10(meas.RSRPPerAntenna) + 30;
meas.RSSIPerAntenna = 10*log10(meas.RSSIPerAntenna) + 30;
meas.RSRQPerAntenna = 10*log10(meas.RSRQPerAntenna);

end