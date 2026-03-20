classdef nrDLCarrierConfig < nr5g.internal.wavegen.CarrierConfigBase
%nrDLCarrierConfig 5G downlink waveform configuration
%   CFG = nrDLCarrierConfig creates a configuration object for a
%   single-component-carrier 5G downlink waveform. This object contains
%   parameters defining the frequency range, channel bandwidth, cell
%   identity, waveform duration (in subframes), SCS carriers, bandwidth
%   parts, SS burst, CORESET, search spaces, PDCCH, PDSCH (as well as their
%   DM-RS and PT-RS), and CSI-RS.
%
%   CFG = nrDLCarrierConfig(Name,Value) creates a 5G downlink waveform
%   configuration object with the specified property Name set to the
%   specified Value. You can specify additional name-value pair
%   arguments in any order as (Name1,Value1,...,NameN,ValueN).
%
%   nrDLCarrierConfig methods:
%   
%   openInGenerator     - Open this downlink carrier configuration in the 5G Waveform Generator 
%
%   nrDLCarrierConfig properties:
%
%   Label               - Alphanumeric description for this downlink
%                         carrier configuration object (default 'Downlink carrier 1')
%   FrequencyRange      - Frequency range ('FR1' (default) or 'FR2')
%   ChannelBandwidth    - Channel bandwidth in MHz (default 50)
%   NCellID             - Physical layer cell identity (0...1007)
%                         (default 1)
%   NumSubframes        - Number of subframes (default 10)
%   InitialNSubframe    - Initial subframe number (default 0)
%   WindowingPercent    - Percentage of windowing relative to FFT length (default 0)
%   SampleRate          - Sample rate of the OFDM modulated waveform (default [])
%   CarrierFrequency    - Carrier frequency in Hz (default 0)
%   SCSCarriers         - Configuration of SCS carrier(s) (default {nrSCSCarrierConfig})
%   BandwidthParts      - Configuration of bandwidth part(s) (default {nrWavegenBWPConfig})
%   SSBurst             - Configuration of SS burst (SS blocks containing PSS, SSS, PBCH)
%                         (default nrWavegenSSBurstConfig)
%   CORESET             - Configuration of CORESET (default {nrCORESETConfig})
%   SearchSpaces        - Configuration of search space(s) (default {nrSearchSpaceConfig})
%   PDCCH               - Configuration of PDCCH channel(s) (default {nrWavegenPDCCHConfig})
%   PDSCH               - Configuration of PDSCH channel(s) (default {nrWavegenPDSCHConfig})
%   CSIRS               - Configuration of CSI-RS signal(s) (default {nrWavegenCSIRSConfig('Enable',0)})
%
%   Example 1:
%   % Create a configuration for a single-numerology (15 kHz), single-user
%   % downlink 5G waveform with no CSI-RS; then generate the waveform.
%
%   cfg = nrDLCarrierConfig('FrequencyRange', 'FR1', 'ChannelBandwidth', 40, 'NumSubframes', 20);
%   cfg.SCSCarriers{1}.NSizeGrid = 100;    % default SCS is 15 kHz
%   cfg.BandwidthParts{1}.NStartBWP = cfg.SCSCarriers{1}.NStartGrid + 10;
%   cfg.SSBurst.BlockPattern = 'Case A'; % 15 kHz
%   cfg.CORESET{1}.Duration = 3;
%   cfg.CORESET{1}.FrequencyResources = [1 1 1 1];
%   cfg.SearchSpaces{1}.NumCandidates = [8 4 0 0 0];
%   cfg.PDCCH{1}.AggregationLevel = 2;
%   cfg.PDCCH{1}.AllocatedCandidate = 4;
%   cfg.PDSCH{1}.Modulation = '16QAM';
%   cfg.PDSCH{1}.TargetCodeRate = 658/1024;
%   cfg.PDSCH{1}.DMRS.DMRSTypeAPosition = 3;
%   cfg.PDSCH{1}.EnablePTRS = true;
%   cfg.PDSCH{1}.PTRS.TimeDensity = 2;
%   cfg.CSIRS{1}.RowNumber = 4;
%   cfg.CSIRS{1}.RBOffset = 10;
%
%   waveform = nrWaveformGenerator(cfg);
%
%   Example 2:
%   % Create a configuration for a mixed-numerology, multi-user 5G downlink
%   % waveform; then generate the waveform.
%
%   % SCS Carriers:
%   scscarriers = {nrSCSCarrierConfig('SubcarrierSpacing', 15, 'NStartGrid', 10, 'NSizeGrid', 100), ...
%                  nrSCSCarrierConfig('SubcarrierSpacing', 30, 'NStartGrid', 0, 'NSizeGrid', 70)};
%   % Bandwidth parts:
%   bwp = {nrWavegenBWPConfig('BandwidthPartID', 0, 'SubcarrierSpacing', 15, 'NStartBWP', 10, 'NSizeBWP', 80), ...
%          nrWavegenBWPConfig('BandwidthPartID', 1, 'SubcarrierSpacing', 30, 'NStartBWP', 0, 'NSizeBWP', 60)};
%   % SS burst:
%   ssburst = nrWavegenSSBurstConfig('BlockPattern', 'Case A'); % 15 kHz
%   % Control (CORESET/Search space/PDCCH):
%   coreset = {nrCORESETConfig('CORESETID', 1, 'FrequencyResources', [1 1 1 1 1 0 0 0 0 0 1], 'Duration', 3), ...
%              nrCORESETConfig('CORESETID', 2, 'FrequencyResources', [0 0 0 0 0 0 0 0 1 1])};
%   ss = {nrSearchSpaceConfig('SearchSpaceID', 1, 'CORESETID', 1, 'StartSymbolWithinSlot', 4), ...
%         nrSearchSpaceConfig('SearchSpaceID', 2, 'CORESETID', 2, 'NumCandidates', [8 8 4 0 0])};
%   pdcch = {nrWavegenPDCCHConfig('SearchSpaceID', 1, 'BandwidthPartID', 0, 'RNTI', 1, 'DMRSScramblingID', 1), ...
%            nrWavegenPDCCHConfig('SearchSpaceID', 2, 'BandwidthPartID', 1, 'RNTI', 2, 'DMRSScramblingID', 2, 'AggregationLevel', 4)};
%   % PDSCH:
%   pdsch = {nrWavegenPDSCHConfig('BandwidthPartID', 0, 'Modulation', '16QAM', 'RNTI', 1, 'NID', 1), ...
%            nrWavegenPDSCHConfig('BandwidthPartID', 1, 'Modulation', 'QPSK', 'RNTI', 2, 'NID', 2, 'PRBSet', 50:59)};
%   % CSI-RS:
%   % In nrWavegenCSIRSConfig, CSI-RS is enabled by default.
%   csirs = {nrWavegenCSIRSConfig('BandwidthPartID', 0, 'RowNumber', 2, 'RBOffset', 10), ...
%           nrWavegenCSIRSConfig('BandwidthPartID', 1, 'Density', 'one', 'RowNumber', 4, 'NumRB', 5)};
%
%   % Combine everything together:
%   cfg = nrDLCarrierConfig('FrequencyRange', 'FR1', 'ChannelBandwidth', 40, 'NumSubframes', 20);
%   cfg.SCSCarriers = scscarriers;
%   cfg.BandwidthParts = bwp;
%   cfg.SSBurst = ssburst;
%   cfg.CORESET = coreset;
%   cfg.SearchSpaces = ss;
%   cfg.PDCCH = pdcch;
%   cfg.PDSCH = pdsch;
%   cfg.CSIRS = csirs;
%
%   % Generate waveform:
%   waveform = nrWaveformGenerator(cfg);
%
%   See also nrWaveformGenerator, nrSCSCarrierConfig, nrWavegenBWPConfig,
%   nrWavegenSSBurstConfig, nrCORESETConfig, nrSearchSpaceConfig,
%   nrWavegenPDCCHConfig, nrWavegenPDSCHConfig, nrPDSCHDMRSConfig,
%   nrPDSCHPTRSConfig, nrWavegenCSIRSConfig, nrULCarrierConfig.

%   Copyright 2020-2024 The MathWorks, Inc.

%#codegen

    properties
        %WindowingPercent Percentage of windowing relative to FFT length
        % Specify WindowingPercent as a scalar or eight-element or 
        % six-element vector of real doubles in the range [0 50] or as []. 
        % This property configures the number of time-domain samples over
        % which windowing and overlapping of OFDM symbols is applied, as a
        % percentage of the FFT length. A scalar value establishes the same
        % windowing for all combinations of subcarrier spacing and cyclic
        % prefix. If set to [], a default value is automatically selected
        % based on other parameters (see <a href="matlab:doc('nrOFDMModulate')">nrOFDMModulate</a>). 
        % If WindowingPercent is the eight-element vector [w1 w2 w3 w4 w5 w6 w7 w8],
        % then w1% is the windowing percentage for the 15 kHz carrier, 
        % w2% for 30 kHz, w3% for 60 kHz and normal cyclic prefix, w4% for
        % 60 kHz and extended cyclic prefix, w5% for 120 kHz, w6% for 240 kHz,
        % w7% for 460 kHz, w8% for 960 kHz. If WindowingPercent has six
        % elements then w6 applies to 240 kHz, 460 kHz, and 960 kHz.
        % The default is 0.
        WindowingPercent = 0;
        
        %SSBurst Configuration of SS burst
        % Specify SSBurst as a scalar <a href="matlab:
        % help('nrWavegenSSBurstConfig')">nrWavegenSSBurstConfig</a> object.
        % This property configures the synchronization signal (SS) burst
        % and blocks. The default is nrWavegenSSBurstConfig.
        SSBurst           = nrDLCarrierConfig.getDefault('SSBurst');
        
        %CORESET Configuration of CORESET
        % Specify CORESET as a cell array of <a href="matlab:
        % help('nrCORESETConfig')">nrCORESETConfig</a> objects. This
        % property specifies different CORESET configurations that may be used
        % by multiple search spaces and PDCCH. The default is {nrCORESETConfig}.
        CORESET           = nrDLCarrierConfig.getDefault('CORESET');
        
        %SearchSpaces Configuration of search spaces
        % Specify SearchSpaces as a cell array of <a href="matlab:
        % help('nrSearchSpaceConfig')">nrSearchSpaceConfig</a> objects.
        % This property specifies different SearchSpace configurations that
        % link to a CORESET and may be used by multiple PDCCH. The default
        % is {nrSearchSpaceConfig}.
        SearchSpaces      = nrDLCarrierConfig.getDefault('SearchSpaces');
        
        %PDCCH Configuration of PDCCH
        % Specify PDCCH as a cell array of <a href="matlab:
        % help('nrWavegenPDCCHConfig')">nrWavegenPDCCHConfig</a> objects.
        % This property configures different physical downlink control
        % channels (PDCCH) and the associated DM-RS signals. The default
        % is {nrWavegenPDCCHConfig}.
        PDCCH             = nrDLCarrierConfig.getDefault('PDCCH');
        
        %PDSCH Configuration of PDSCH
        % Specify PDSCH as a cell array of <a href="matlab:
        % help('nrWavegenPDSCHConfig')">nrWavegenPDSCHConfig</a> objects.
        % This property configures different physical downlink shared
        % channels (PDSCH) as well as their DM-RS and PT-RS signals. The
        % default is {nrWavegenPDSCHConfig}.
        PDSCH             = nrDLCarrierConfig.getDefault('PDSCH');
        
        %CSIRS Configuration of CSIRS
        % Specify CSIRS as a cell array of <a href="matlab:
        % help('nrWavegenCSIRSConfig')">nrWavegenCSIRSConfig</a> objects. This
        % property configures different channel state information reference
        % signals (CSI-RS). The default value is {nrWavegenCSIRSConfig('Enable',0)},
        % which disables the CSI-RS.
        CSIRS             = nrDLCarrierConfig.getDefault('CSIRS');
    end
    
    properties (Hidden)
        CustomPropList = {'Label', 'FrequencyRange', 'ChannelBandwidth', 'NCellID', 'NumSubframes', 'InitialNSubframe', ...
            'WindowingPercent', 'SampleRate', 'CarrierFrequency', ...
            'SCSCarriers',  'BandwidthParts', ...
            'SSBurst', 'CORESET', 'SearchSpaces', ...
            'PDCCH', 'PDSCH', 'CSIRS'};
    end
    
    methods
        % Constructor
        function obj = nrDLCarrierConfig(varargin)
            
            ssb = nr5g.internal.parseProp('SSBurst', ...
                nrDLCarrierConfig.getDefault('SSBurst'),varargin{:});
            crst = nr5g.internal.parseProp('CORESET', ...
                nrDLCarrierConfig.getDefault('CORESET'),varargin{:});
            ss = nr5g.internal.parseProp('SearchSpaces', ...
                nrDLCarrierConfig.getDefault('SearchSpaces'),varargin{:});
            pdcch = nr5g.internal.parseProp('PDCCH', ...
                nrDLCarrierConfig.getDefault('PDCCH'),varargin{:});
            pdsch = nr5g.internal.parseProp('PDSCH', ...
                nrDLCarrierConfig.getDefault('PDSCH'),varargin{:});
            csirs = nr5g.internal.parseProp('CSIRS', ...
                nrDLCarrierConfig.getDefault('CSIRS'),varargin{:});
            
            obj@nr5g.internal.wavegen.CarrierConfigBase( ...
                'SSBurst', ssb, ...
                'CORESET', crst, ...
                'SearchSpaces', ss, ...
                'PDCCH', pdcch, ...
                'PDSCH', pdsch, ...
                'CSIRS', csirs, ...
                'Label', 'Downlink carrier 1', ...
                varargin{:});
        end
        
        % Self-validate and set properties
        function obj = set.WindowingPercent(obj,val)
            prop = 'WindowingPercent';
            
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 8],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                
                % If Windowing is a vector, it must have 6 or 8 elements
                coder.internal.errorIf(~any(numel(temp)==[1 6 8]), ...
                    'nr5g:nrWaveformGenerator:InvalidWindowingVector', 'downlink', 6, 8);
                
                validateattributes(temp,{'numeric'},...
                    {'real','nonnegative', '<=', 50},...
                    [class(obj) '.' prop],prop);
            end
            
            obj.WindowingPercent = temp;
        end
        
        function obj = set.SSBurst(obj,val)
            validateattributes(val,{'nrWavegenSSBurstConfig'},{'scalar'},[class(obj) '.SSBurst'], 'SSBurst');
            obj.SSBurst = val;
        end
        
        function obj = set.CORESET(obj,val)
            validateCellObjProp(obj, 'CORESET', {'nrCORESETConfig'}, val);
            obj.CORESET = val;
        end
        
        function obj = set.SearchSpaces(obj,val)
            validateCellObjProp(obj, 'SearchSpaces', {'nrSearchSpaceConfig'}, val);
            obj.SearchSpaces = val;
        end
        
        function obj = set.PDCCH(obj,val)
            validateCellObjProp(obj, 'PDCCH', {'nrWavegenPDCCHConfig'}, val);
            obj.PDCCH = val;
        end
        
        function obj = set.PDSCH(obj,val)
            validateCellObjProp(obj, 'PDSCH', {'nrWavegenPDSCHConfig'}, val);
            obj.PDSCH = val;
        end
        
        function obj = set.CSIRS(obj,val)
            validateCellObjProp(obj, 'CSIRS', {'nrWavegenCSIRSConfig'}, val);
            obj.CSIRS = val;
        end
        
        function validateConfig(obj)
            
            % Call nrCarrierConfigBase validator
            validateConfig@nr5g.internal.wavegen.CarrierConfigBase(obj);
            
            %% SS Burst
            if obj.SSBurst.Enable
                % validate SSB object
                validateConfig(obj.SSBurst);
                
                % Make sure that BlockPattern is compatibile with FrequencyRange
                fr1 = strcmp(obj.FrequencyRange, 'FR1');
                blockPattern = obj.SSBurst.BlockPattern;
                fr1BlockPattern = any(strcmpi(blockPattern, {'Case A', 'Case B', 'Case C'}));
                coder.internal.errorIf(xor(fr1,fr1BlockPattern),'nr5g:nrWaveformGenerator:InvalidBlockPattern');
                
                % Verify that an SCS carrier exists for the SSBurst
                burstSCS = nr5g.internal.wavegen.blockPattern2SCS(blockPattern);
                carriers = obj.SCSCarriers;
                carrierSCS = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers, 'SubcarrierSpacing');
                coder.internal.errorIf(~any(carrierSCS==burstSCS), ...
                    'nr5g:nrWaveformGenerator:SSBNotInCarrier', burstSCS(1), blockPattern);
                
                % Verify that the SSB can fit in its SCS carrier
                burstCarrier = carriers{carrierSCS==burstSCS};                
                validateSSBurstFreqLocation(burstCarrier,obj.SSBurst,obj.ChannelBandwidth);                
            end
            
            %% CORESET
            % Validate each CORESET
            for idx = 1:numel(obj.CORESET)
                validateConfig(obj.CORESET{idx});
            end
            
            % Unique IDs
            csetID = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(obj.CORESET, 'CORESETID');
            uniqueIDs = unique(csetID);
            coder.internal.errorIf(numel(obj.CORESET)~=numel(uniqueIDs), ...
                'nr5g:nrWaveformGenerator:IDNotUnique', 'CORESET');
            
            %% Search spaces
            % Validate each Search Space
            for idx = 1:numel(obj.SearchSpaces)
                validateConfig(obj.SearchSpaces{idx});
            end
            
            % Unique IDs
            ssID = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(obj.SearchSpaces, 'SearchSpaceID');
            uniqueIDs = unique(ssID);
            coder.internal.errorIf(numel(obj.SearchSpaces)~=numel(uniqueIDs), ...
                'nr5g:nrWaveformGenerator:IDNotUnique', 'search spaces');
            
            % Search space must link to an existing CORESET
            for idx1 = 1:numel(obj.SearchSpaces)
                linkExists = false;
                for idx2 = 1:numel(obj.CORESET)
                    if obj.CORESET{idx2}.CORESETID == obj.SearchSpaces{idx1}.CORESETID
                        linkExists = true;
                    end
                end
                coder.internal.errorIf(~linkExists, ...
                    'nr5g:nrWaveformGenerator:SSInvalidCORESET', obj.SearchSpaces{idx1}.SearchSpaceID, obj.SearchSpaces{idx1}.CORESETID);
            end
            
            %% PDCCH
            % PDCCH must link to an existing Search Space
            
            % initialization for codegen of subsequent checks
            % CORESET
            propsCSET = {'CORESETID', 'Duration', 'FrequencyResources','RBOffset'};
            csetinit = nrCORESETConfig;
            for idx = 1:length(propsCSET)
                coreset.(propsCSET{idx}) = csetinit.(propsCSET{idx});
            end
            coder.varsize('coreset.FrequencyResources',[1 45],[0 1]);
            coder.varsize('coreset.RBOffset',[1 1],[1 1]);

            % SearchSpace
            propsSS = {'SearchSpaceID', 'CORESETID', 'StartSymbolWithinSlot', 'SlotPeriodAndOffset', 'Duration', 'NumCandidates'};
            ssinit = nrSearchSpaceConfig;
            for idx = 1:length(propsSS)
                searchSpace.(propsSS{idx}) = ssinit.(propsSS{idx});
            end
            % BWP
            propsBWP = {'NStartBWP', 'NSizeBWP', 'CyclicPrefix', 'SubcarrierSpacing'};
            bwpinit = nrWavegenBWPConfig;
            for idx = 1:length(propsBWP)
                bwp.(propsBWP{idx}) = bwpinit.(propsBWP{idx});
            end
            coder.varsize('bwp.CyclicPrefix',[1,8],[0,1]);
            
            for idx1 = 1:numel(obj.PDCCH)
                if ~obj.PDCCH{idx1}.Enable
                    % no validation for disabled PDCCH
                    continue
                end
                linkExists = false;
                for idx2 = 1:numel(obj.SearchSpaces)
                    if obj.SearchSpaces{idx2}.SearchSpaceID == obj.PDCCH{idx1}.SearchSpaceID
                        linkExists = true;
                        for idx = 1:length(propsSS)
                            searchSpace.(propsSS{idx}) = obj.SearchSpaces{idx2}.(propsSS{idx});
                        end
                    end
                end
                coder.internal.errorIf(~linkExists, ...
                    'nr5g:nrWaveformGenerator:PDCCHInvalidLink', idx1, 'search space', obj.PDCCH{idx1}.SearchSpaceID);
                
                % PDCCH must link to an existing Bandwidth Part
                linkExists = false;
                for idx2 = 1:numel(obj.BandwidthParts)
                    if obj.BandwidthParts{idx2}.BandwidthPartID == obj.PDCCH{idx1}.BandwidthPartID
                        linkExists = true;
                        for idx = 1:length(propsBWP)
                            bwp.(propsBWP{idx}) = obj.BandwidthParts{idx2}.(propsBWP{idx});
                        end
                    end
                end
                coder.internal.errorIf(~linkExists, ...
                    'nr5g:nrWaveformGenerator:PDCCHInvalidLink', idx1, 'bandwidth part', obj.PDCCH{idx1}.BandwidthPartID);
                
                % Validate each PDCCH
                for idx2 = 1:numel(obj.CORESET)
                    if obj.CORESET{idx2}.CORESETID == searchSpace.CORESETID
                        for idx = 1:length(propsCSET)
                            coreset.(propsCSET{idx}) = obj.CORESET{idx2}.(propsCSET{idx});
                        end
                    end
                end
                validateConfig(obj.PDCCH{idx1}, coreset, searchSpace, bwp.NStartBWP, bwp.NSizeBWP);
                % PDCCH validation makes sure that search spaces is within a Normal-CP slot. Check for Extended CP:
                if strcmp(bwp.CyclicPrefix, 'extended')
                    symbperslot = 12;
                    if searchSpace.StartSymbolWithinSlot + coreset.Duration > symbperslot
                        coder.internal.warning('nr5g:nrWaveformGenerator:SSBeyondSlot', ...
                            searchSpace.SearchSpaceID, searchSpace.StartSymbolWithinSlot, ...
                            coreset.CORESETID, coreset.Duration, symbperslot);
                    end
                end
                
                % Make sure that the PDCCH slot allocation falls within its
                % SearchSpace slot Allocation
                pdcchSlots        = nr5g.internal.wavegen.expandbyperiod(obj.PDCCH{idx1}.SlotAllocation, obj.PDCCH{idx1}.Period, obj.NumSubframes, bwp.SubcarrierSpacing, obj.InitialNSubframe);
                ssOffset = searchSpace.SlotPeriodAndOffset(2);
                searchSpaceSlots  = nr5g.internal.wavegen.expandbyperiod(...
                    ssOffset:(ssOffset+searchSpace.Duration-1), searchSpace.SlotPeriodAndOffset(1), ...
                    obj.NumSubframes, bwp.SubcarrierSpacing,obj.InitialNSubframe);
                pdcchSlotsNotInSS = pdcchSlots(~ismember(pdcchSlots, searchSpaceSlots));
                coder.internal.errorIf(~isempty(pdcchSlotsNotInSS), ...
                    'nr5g:nrWaveformGenerator:PDCCHNotInSS', idx1, searchSpace.SearchSpaceID);

                % Validate PDCCH MIMO precoding configuration
                validateMIMOPrecoding(obj.PDCCH{idx1},'PDCCH',1,idx1);

            end
            
            %% PDSCH
            % Make sure all PDSCH link to a valid bandwidth part
            checkResource2BWPLinks(obj, obj.PDSCH, 'PDSCH');
            
            for idx = 1:numel(obj.PDSCH)
                % validate each PDSCH
                if obj.PDSCH{idx}.Enable
                    % no validation for disabled PDSCH
                    validateConfig(obj.PDSCH{idx});
                    
                    % Make sure all CORESET defined in ReservedCORESET exist
                    resCORESET = obj.PDSCH{idx}.ReservedCORESET(:);
                    coder.internal.errorIf(any(~ismember(resCORESET, csetID),1), ...
                        'nr5g:nrWaveformGenerator:InvalidReservedCORESET', idx);

                    % Validate PDSCH MIMO precoding configuration
                    validateMIMOPrecoding(obj.PDSCH{idx},'PDSCH',obj.PDSCH{idx}.NumLayers,idx);
                end
            end
            
            %% CSI-RS
            % CSI-RS must link to an existing Bandwidth Part
            checkResource2BWPLinks(obj, obj.CSIRS, 'CSI-RS');

            for idx = 1:numel(obj.CSIRS)
                if obj.CSIRS{idx}.Enable
                    % Validate CSIRS MIMO precoding configuration
                    validateMIMOPrecoding(obj.CSIRS{idx},'CSIRS',max(obj.CSIRS{idx}.NumCSIRSPorts),idx);
                end
            end
        end
    end

    methods (Static, Access = protected)
        % Default values of SSBurst, CORESET, SearchSpaces, PDSCH, PDCCH, and CSIRS properties.
        function out = getDefault(propName)
            switch propName
                case 'SSBurst'
                    out = nrWavegenSSBurstConfig;
                case 'CORESET'
                    out = {nrCORESETConfig};
                case 'SearchSpaces'
                    out = {nrSearchSpaceConfig};
                case 'PDSCH'
                    out = {nrWavegenPDSCHConfig};
                case 'PDCCH'
                    out = {nrWavegenPDCCHConfig};
                case 'CSIRS'
                    out = {nrWavegenCSIRSConfig('Enable',0)};
            end
        end
    end

end

% Validate the location of the SS burst in its SCS carrier
function validateSSBurstFreqLocation(carrier,burst,channelBandwidth)

    % Error if SSB cannot fit in carrier  
    nStartGrid = carrier.NStartGrid;
    nSizeGrid = carrier.NSizeGrid;
    minGridSize = 20 - (channelBandwidth == 3)*5;
    coder.internal.errorIf(nSizeGrid < minGridSize,'nr5g:nrWaveformGenerator:SSBCarrierTooSmall',carrier.SubcarrierSpacing,minGridSize);

    % If NCRBSSB is empty, the SSB is centered in its SCS
    % carrier and no further validation is required. Otherwise,
    % verify that the SSB does not exceed the frequency range
    % of its associated SCS carrier.
    if ~isempty(burst.NCRBSSB)
    
        % Get SCS associated to SS burst, KSSB and NCRBSSB
        [burstSCS,KSSBscs,NCRBSSBscs] = nr5g.internal.wavegen.blockPattern2SCS(burst.BlockPattern,burst.SubcarrierSpacingCommon);
    
        % (NCRBSSB <=> SSB) units conversion factor (>1)
        crbScaling = burstSCS / NCRBSSBscs;
    
        % Determine the lowest CRB overlapping with the SSB from NCRBSSB and
        % KSSB offsets
        NCRBSSB = double(burst.NCRBSSB(1));
        KSSB = double(burst.KSSB);
        KSSBOffset = (KSSB * KSSBscs / burstSCS) / 12;  % KSSB offset in RB of the SSB SCS
        SSBCRB0 = NCRBSSB / crbScaling + KSSBOffset;    % Lowest CRB overlapping with the SSB

        % Error if the SS burst is out of the limits of its SCS carrier
        if (SSBCRB0 < nStartGrid) || ((SSBCRB0+20) > (nStartGrid+nSizeGrid))
    
            % Calculate the min and max NCRBSSB for this configuration
            minNCRB = ceil((nStartGrid - KSSBOffset) * crbScaling);
            maxNCRB = floor((nStartGrid + nSizeGrid - 20 - KSSBOffset) * crbScaling);
    
            % If the min NCRBSSB allowed is larger than the max NCRBSSB, it is
            % due to KSSB, so recalculate range with KSSB = 0.
            if minNCRB > maxNCRB
                minNCRB = nStartGrid * crbScaling;
                maxNCRB = (nStartGrid + nSizeGrid - 20) * crbScaling;
                KSSB = 0;
            end

            % Calculate NCRBSSB to center the SS burst in its SCS carrier
            centerNCRB = round((minNCRB + maxNCRB)/2);

            coder.internal.error('nr5g:nrWaveformGenerator:SSBOutOfLimits',burstSCS,minNCRB,maxNCRB,KSSB,centerNCRB);
            
        end
    end
end
