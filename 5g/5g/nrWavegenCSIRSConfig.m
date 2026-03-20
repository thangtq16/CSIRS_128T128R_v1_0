classdef nrWavegenCSIRSConfig < nr5g.internal.nrCSIRSConfigBase & ...
                                nr5g.internal.wavegen.MIMOPrecodingConfig
    %nrWavegenCSIRSConfig CSI-RS configuration for 5G waveform generation
    %   CSIRS = nrWavegenCSIRSConfig creates a Channel State Information Reference
    %   Signal (CSI-RS) configuration object for single or multiple CSI-RS
    %   resources. This object contains the properties related to TS 38.211
    %   Section 7.4.1.5. By default, the object defines an NZP-CSI-RS
    %   resource configured for 2 antenna ports with CDM type equal to
    %   FD-CDM2 and density equal to 1 (corresponding to the row number 3
    %   in TS 38.211 Table 7.4.1.5.3-1).
    %
    %   CSIRS = nrWavegenCSIRSConfig(Name,Value) creates a CSI-RS configuration
    %   object CSIRS with the specified property Name set to the specified
    %   Value. You can specify additional name-value arguments in any order
    %   as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrWavegenCSIRSConfig properties (configurable):
    %
    %   Enable                - Logical turning this signal on or off (default true)
    %   Label                 - Alphanumeric description for this CSI-RS
    %   Power                 - Power scaling in dB
    %   BandwidthPartID       - ID of bandwidth part containing this CSI-RS
    %   CSIRSType             - CSI-RS type ('nzp' (default), 'zp')
    %   CSIRSPeriod           - CSI-RS slot periodicity (Tcsi-rs) and
    %                           offset (Toffset)
    %                           ('on' (default), 'off', [Tcsi-rs Toffset])
    %   RowNumber             - Row number corresponding to a CSI-RS
    %                           resource as defined in TS 38.211
    %                           Table 7.4.1.5.3-1 (1...18) (default 3)
    %   Density               - CSI-RS resource frequency density
    %                           ('one' (default), 'three', 'dot5even', 'dot5odd')
    %   SymbolLocations       - Time-domain locations of a CSI-RS resource
    %                           (default 0)
    %   SubcarrierLocations   - Frequency-domain locations of a CSI-RS
    %                           resource (default 0)
    %   NumRB                 - Number of resource blocks (RBs) allocated
    %                           for a CSI-RS resource (1...275) (default 52)
    %   RBOffset              - Starting RB index of CSI-RS allocation
    %                           relative to carrier resource grid
    %                           (0...274) (default 0)
    %   NID                   - Scrambling identity (0...1023) (default 0)
    %
    %   Note that each element of CSIRSType configures one CSI-RS resource.
    %
    %   Example 1:
    %   %  Create nrWavegenCSIRSConfig object with its default properties.
    %
    %   csirs = nrWavegenCSIRSConfig
    %
    %   Example 2:
    %   %  Create nrWavegenCSIRSConfig object with CSIRSType as 'zp'.
    %
    %   csirs = nrWavegenCSIRSConfig('CSIRSType','zp')
    %
    %   Example 3:
    %   %  Generate CSI-RS specific configuration object for 3 resources
    %   %  (ZP, NZP, ZP) with row numbers [5 3 8].
    %
    %   csirs = nrWavegenCSIRSConfig('CSIRSType',{'zp','nzp','zp'},...
    %       'CSIRSPeriod',{'on','on','off'},'RowNumber',[5 3 8],...
    %       'Density',{'one','dot5odd','one'},'SymbolLocations',{6,10,9},...
    %       'SubcarrierLocations',{0,0,[0 4]})
    %
    %   See also nrDLCarrierConfig, nrWaveformGenerator.

    %   Copyright 2020-2024 The MathWorks, Inc.

    %#codegen

    properties
        %Enable Flag to turn channel or signal on or off in waveform generation
        % Specify Enable as a logical scalar. This flag determines the
        % presence of this channel or signal in the generated 5G waveform. The
        % default is true.
        Enable (1,1) logical = true;

        %Power Power scaling in dB
        % Specify Power in dB as a real scalar that expresses the amount by
        % which this channel or signal is scaled. The default is 0 dB.
        Power (1,1) {mustBeNumeric, mustBeReal, mustBeFinite} = 0;

        %Label Custom alphanumeric label
        % Specify Label as a character array or string scalar. Use this
        % property to assign a description to this CSI-RS object. The default
        % is 'CSIRS1'.
        Label = 'CSIRS1';

        %BandwidthPartID ID of bandwidth part containing this CSI-RS
        % Specify BandwidthPartID as a nonnegative integer scalar to link this
        % CSI-RS instance with one of the bandwidth parts defined in the
        % BandwidthParts property of <a href="matlab:help('nrDLCarrierConfig.BandwidthParts')"
        % >nrDLCarrierConfig</a>. The default is 1.
        BandwidthPartID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative} = 1;
    end

    properties (Hidden)
        CustomPropList = {'Enable'; 'Label'; 'Power'; 'BandwidthPartID';  'CSIRSType'; ...
            'CSIRSPeriod'; 'RowNumber'; 'Density'; 'SymbolLocations'; ...
            'SubcarrierLocations'; 'NumRB'; 'RBOffset'; ...
            'AntennaMapping'; 'PrecodingMatrix'; 'NID'; 'NumCSIRSPorts'};
    end

    properties (Hidden = true, Dependent = true)
        NumColumns
        Wpa
    end

    methods
        function obj = nrWavegenCSIRSConfig(varargin)
            obj@nr5g.internal.nrCSIRSConfigBase(varargin{:});
        end

        function obj = set.Label(obj,val)
            %set.NStartBWP Set the start of BWP resource grid
            prop = 'Label';
            validateattributes(val, {'char', 'string'}, {'scalartext'}, ...
                [class(obj) '.' prop], prop);
            obj.(prop) = convertStringsToChars(val);
        end

        function numCols = get.NumColumns(obj)
            numCols = obj.getNumCols(max(obj.NumCSIRSPorts));
        end

        function P = get.Wpa(obj)

            P = obj.calculatePrecodeAndMapMatrix(max(obj.NumCSIRSPorts));

        end

    end
end
