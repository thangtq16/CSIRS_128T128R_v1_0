classdef (Abstract) CarrierConfigBase < comm.internal.ConfigBase
    %CarrierConfigBase Class containing properties common between nrDLCarrierConfig and nrULCarrierConfig
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    %   Copyright 2020-2024 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %Label Custom alphanumeric label
        % Specify Label as a character array or string scalar. Use this
        % property to assign a description to this carrier configuration
        % object.
        Label = '';

        %FrequencyRange Frequency range
        % Specify the frequency range as 'FR1' (default) or 'FR2'.
        % Frequency range 1 corresponds to frequencies from 410 MHz to
        % 7.125 GHz, while frequency range 2 corresponds to frequencies
        % from 24.25 GHz to 52.6 GHz (FR2-1) and from 52.6 GHz to 71 GHz (FR2-2)
        FrequencyRange    = 'FR1';

        %ChannelBandwidth Channel bandwidth
        % Specify the channel bandwidth, in MHz, as one of
        % {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100} when
        % FrequencyRange is 'FR1' and one of {50, 100, 200, 400, 800, 1600,
        % 2000} when FrequencyRange is 'FR2'. The default is 50.
        ChannelBandwidth  = 50;

        %NCellID Physical layer cell identity number
        % Specify the physical layer cell identity as a nonnegative real
        % scalar integer that is not greater than 1007. The default is 1.
        NCellID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NCellID, 1007)} = 1;

        %NumSubframes Number of subframes
        % Specify the duration of the 5G waveform in subframes (multiples
        % of 1 ms). The default is 10 subframes, which corresponds to one
        % frame.
        NumSubframes (1,1) {mustBeNumeric, mustBeInteger, mustBePositive} = 10;

        %InitialNSubframe Initial subframe number
        % Specify the initial subframe number in the 5G waveform (0-based).
        % The default is 0.
        InitialNSubframe (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative} = 0;

        %SampleRate Sample rate of the OFDM modulated waveform
        % Specify the sample rate as a positive real number or []. This
        % property configures the desired sample rate of the OFDM modulated
        % waveform. If set to [], the object uses the value corresponding
        % to the maximum default sample rate across the configured SCS
        % carriers, as given by <a href="matlab:help('nrOFDMInfo')"
        % >nrOFDMInfo</a>. The default is [].
        SampleRate        = [];

        %CarrierFrequency Carrier frequency in Hz
        % Specify the carrier frequency as a scalar real number. This
        % property configures the carrier frequency in Hz, denoted f_0 in
        % TS 38.211 Section 5.4. This property is used for symbol phase
        % compensation before OFDM modulation, not for upconversion. The
        % default is 0.
        CarrierFrequency (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0;

        %SCSCarriers Configuration of SCS carriers
        % Specify SCSCarriers as a cell array of <a
        % href="matlab:help('nrSCSCarrierConfig')">nrSCSCarrierConfig</a>
        % objects. This property determines the subcarrier spacing and grid
        % size of each numerology. Each nrSCSCarrierConfig object in the
        % cell array must contain a unique SubcarrierSpacing property
        % value. The default is {nrSCSCarrierConfig}.
        SCSCarriers       = {nrSCSCarrierConfig};

        %BandwidthParts Configuration of bandwidth parts
        % Specify BandwidthParts as a cell array of <a
        % href="matlab:help('nrWavegenBWPConfig')">nrWavegenBWPConfig</a>
        % objects. This property configures different bandwidth parts. The
        % default is {nrWavegenBWPConfig}.
        BandwidthParts    = {nrWavegenBWPConfig};
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        FrequencyRange_Values = {'FR1', 'FR2'};
        CBW_FR1_Options = [5 10 15 20 25 30 35 40 45 50 60 70 80 90 100];
        CBW_FR2_Options = [50 100 200 400 800 1600 2000];
    end

    methods
        % Constructor
        function obj = CarrierConfigBase(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
        end

        % Self-validate and set properties
        function obj = set.Label(obj,val)
            prop = 'Label';
            validateattributes(val, {'char', 'string'}, {'scalartext'}, ...
                [class(obj) '.' prop], prop);
            obj.(prop) = convertStringsToChars(val);
        end

        function obj = set.FrequencyRange(obj,val)
            prop = 'FrequencyRange';
            val = validatestring(val,obj.FrequencyRange_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

        function obj = set.ChannelBandwidth(obj,val)
            prop = 'ChannelBandwidth';
            validateattributes(val, {'numeric'}, {'scalar', 'integer'}, ...
                [class(obj) '.' prop], prop);
            versusFR = false;
            validateCBW(obj,val,versusFR);
            obj.(prop) = val;
        end

        function obj = set.SampleRate(obj,val)
            prop = 'SampleRate';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'}, ...
                    {'scalar','real','positive','integer'},[class(obj) '.' prop], prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.SCSCarriers(obj,val)
            prop = 'SCSCarriers';
            validateattributes(val,{'cell'},{'nonempty'},[class(obj) '.' prop],prop);
            for idx = 1:numel(val)
                validateattributes(val{idx},{'nrSCSCarrierConfig'},{'scalar'},[class(obj) '.' prop],prop);
            end
            obj.SCSCarriers = val;
        end

        function obj = set.BandwidthParts(obj,val)
            validateCellObjProp(obj, 'BandwidthParts', {'nrWavegenBWPConfig'}, val);
            obj.BandwidthParts = val;
        end

        function openInGenerator(obj)
            %openInGenerator Open carrier configuration in 5G Waveform Generator
            %   Open the input carrier configuration in the 5G Waveform
            %   Generator app.

            wirelessWaveformGenerator.internal.openInGenerator(obj);
        end

        % Validate configuration
        function validateConfig(obj)

            %% Channel bandwidth
            versusFR = true;
            validateCBW(obj,obj.ChannelBandwidth,versusFR);

            %% SCS carriers:
            % Ensure all SCS carriers belong to this FR
            isDownlink = isa(obj,'nrDLCarrierConfig');
            if strcmp(obj.FrequencyRange, 'FR1')
                minSCS = 15;
                maxSCS = 60;
            else
                minSCS = 60;
                maxSCS = 960; % Extended to 960 kHz in Rel-17 FR2-2
            end

            % Also, make sure that Channel Bandwidth can fit all carriers. The
            % highest SCS carrier is centered and this causes a point A offset.
            carrierSCS = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(obj.SCSCarriers, 'SubcarrierSpacing');
            [maxscs, maxIdx] = max(carrierSCS);
            pointAToCenter = (obj.SCSCarriers{maxIdx}.NSizeGrid/2 + obj.SCSCarriers{maxIdx}.NStartGrid)*12*maxscs;

            used = false(1, 7);  % All transmission numerologies, mu = 0...6, 15 kHz - 960 kHz
            % Also, only one carrier-object per SCS:
            for idx = 1:numel(obj.SCSCarriers)
                scs = obj.SCSCarriers{idx}.SubcarrierSpacing;
                if isDownlink
                    coder.internal.errorIf(any(scs<minSCS) || any(scs>maxSCS), ...
                        'nr5g:nrWaveformGenerator:InvalidSCSDL', idx, scs, obj.FrequencyRange);
                else % nrULCarrierConfig
                    coder.internal.errorIf(any(scs<minSCS) || any(scs>maxSCS) || any(scs==240), ...   % 240 kHz is allowed only for the SS burst and thus not for uplink transmission
                        'nr5g:nrWaveformGenerator:InvalidSCSUL', idx, scs, obj.FrequencyRange);
                end
                % Only one carrier per SCS
                thisIdx = 1 + log2(scs/15);  % Turn SCS value into a 1-based index
                coder.internal.errorIf(used(thisIdx), ...
                    'nr5g:nrWaveformGenerator:OneCarrierPerSCS', scs);   % Error is already used
                used(thisIdx) = true;   % Otherwise mark SCS as used

                threeMhzCheck = (obj.ChannelBandwidth == 3 && scs ~= 15);
                coder.internal.errorIf(threeMhzCheck,'nr5g:nrWaveformGenerator:Invalid3MHzSCS');

                if scs ~= 240
                    %  Make sure that a grid does not have more RBs than
                    %  the maximum allowed by the FR and BW combination
                    maxNRB = nr5g.internal.wavegen.getNumRB(obj.FrequencyRange, scs, obj.ChannelBandwidth);
                    coder.internal.errorIf(obj.SCSCarriers{idx}.NSizeGrid > maxNRB(1), ...    % (1) indexing for codegen
                        'nr5g:nrWaveformGenerator:GridTooLarge', obj.SCSCarriers{idx}.NSizeGrid, scs, maxNRB(1), obj.ChannelBandwidth, scs);

                    % Make sure that carrier fits in the channel, despite
                    % the point A offset due to the centering of the
                    % highest-SCS carrier.
                    gstart = obj.SCSCarriers{idx}.NStartGrid*12*scs;
                    gsize = obj.SCSCarriers{idx}.NSizeGrid*12*scs;

                    gb = nr5g.internal.wavegen.getGuardband(obj.FrequencyRange,obj.ChannelBandwidth,scs);

                    % Lower and upper bounds of SCS carrier resource grid
                    % in frequency (kHz)
                    fmin = -pointAToCenter + gstart;
                    fmax = fmin + gsize;

                    % Bandwidth occupied by this SCS carrier
                    occupiedBandwidth = 2*max(abs([fmin fmax]))*1e3;

                    % Consider guard bands in the validation against the
                    % channel bandwidth except if undefined
                    if isnan(gb)
                        gb = 0;
                    end
                    maxBw = (obj.ChannelBandwidth - 2*gb)*1e6;
                    coder.internal.errorIf(occupiedBandwidth > maxBw, ...
                        'nr5g:nrWaveformGenerator:CBWTooSmall');

                    % If SampleRate is custom, verify that it is sufficient
                    % to cover the bandwidth spanned by this SCS carrier
                    if ~isempty(obj.SampleRate)
                        coder.internal.errorIf(occupiedBandwidth > obj.SampleRate(1), ...
                            'nr5g:nrWaveformGenerator:SampleRateTooSmall',obj.SampleRate(1),occupiedBandwidth)
                    end
                end
            end

            %% Bandwidth parts:
            % All IDs must be unique
            bwpID = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(obj.BandwidthParts, 'BandwidthPartID');
            uniqueIDs = unique(bwpID);
            coder.internal.errorIf(numel(obj.BandwidthParts)~=numel(uniqueIDs), ...
                'nr5g:nrWaveformGenerator:IDNotUnique', 'bandwidth parts');

            % Checks for each BWP
            for idx = 1:numel(obj.BandwidthParts)
                % the SCS value must also be legitimate for this FR:
                bwpObj = obj.BandwidthParts{idx};
                scs = bwpObj.SubcarrierSpacing;

                % 240 kHz is only for the SS Burst
                coder.internal.errorIf(scs == 240, ...
                    'nr5g:nrWaveformGenerator:InvalidSCSBWP');

                % BWP must link to an existing SCS carrier
                thisIdx = 1 + log2(scs/15);
                coder.internal.errorIf(~used(thisIdx), ...
                    'nr5g:nrWaveformGenerator:BWP2SCSLinkBroken', bwpObj.BandwidthPartID, scs);

                % BWP must lie within its SCS carrier
                idx2 = find(carrierSCS==scs, 1); % now guaranteed one such exists
                scsObj = obj.SCSCarriers{idx2(1)};
                coder.internal.errorIf(bwpObj.NStartBWP<scsObj.NStartGrid, ...
                    'nr5g:nrWaveformGenerator:BWPStartLessThanSCS', bwpObj.BandwidthPartID, bwpObj.NStartBWP, scsObj.NStartGrid);
                coder.internal.errorIf((bwpObj.NStartBWP+bwpObj.NSizeBWP)>(scsObj.NStartGrid+scsObj.NSizeGrid), ...
                    'nr5g:nrWaveformGenerator:BWPExtendsBeyondGrid', bwpObj.BandwidthPartID, bwpObj.NStartBWP, bwpObj.NSizeBWP, scsObj.NStartGrid, scsObj.NSizeGrid);
            end
        end
    end

    methods(Access = protected)
        function checkResource2BWPLinks(obj, resources, prop)
            for idx1 = 1:numel(resources)
                linkExists = false;
                if resources{idx1}.Enable
                    % No check for disabled channels/signals
                    for idx2 = 1:numel(obj.BandwidthParts)
                        if obj.BandwidthParts{idx2}.BandwidthPartID == resources{idx1}.BandwidthPartID
                            linkExists = true;
                        end
                    end
                    coder.internal.errorIf(~linkExists, ...
                        'nr5g:nrWaveformGenerator:CHNotInBWP', prop, idx1, resources{idx1}.BandwidthPartID);
                end
            end
        end

        function validateCellObjProp(obj, prop, classes, val)
            % Classes is a cell array of character vectors
            validateattributes(val,{'cell'},{},[class(obj) '.' prop],prop);
            for idx = 1:numel(val)
                validateattributes(val{idx},classes,{'scalar'},[class(obj) '.' prop],prop);
            end
        end
    end

end

function validateCBW(obj,val,versusFR)

    cbw_FR1 = obj.CBW_FR1_Options;
    cbw_FR2 = obj.CBW_FR2_Options;
    if (matlab.internal.feature("5g_r18_3MHz"))
        cbw_FR1 = [3 cbw_FR1];
    end
    if (versusFR)
        fr1 = strcmp(obj.FrequencyRange, 'FR1');
        if fr1
            flag = ~any(val == cbw_FR1);
        else
            flag = ~any(val == cbw_FR2);
        end
    else
        flag = ~any(val == union(cbw_FR1, cbw_FR2));
    end
    coder.internal.errorIf(flag,'nr5g:nrWaveformGenerator:InvalidCBW',val);

end
