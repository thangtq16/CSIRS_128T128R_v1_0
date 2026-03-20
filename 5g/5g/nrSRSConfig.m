classdef nrSRSConfig < nr5g.internal.srs.nrSRSConfigBase
    %nrSRSConfig Sounding reference signal (SRS) configuration object
    %   CFGSRS = nrSRSConfig creates an SRS configuration object. The
    %   object contains parameters of the SRS defined in TS 38.211 Section
    %   6.4.1.4. The default nrSRSConfig object represents a single-port,
    %   single-symbol configuration that places the SRS at the end of the
    %   slot. This configuration is narrowband and without frequency
    %   hopping (BHop>=BSRS).
    %
    %   CFGSRS = nrSRSConfig(Name,Value) creates an SRS configuration object
    %   with the specified property Name set to the specified Value. You
    %   can specify additional name-value arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrSRSConfig properties (configurable):
    %
    %   NumSRSPorts              - Number of antenna ports (1 (default), 2, 4, 8)
    %   SymbolStart              - First SRS symbol in a slot (0...13) (default 13)
    %   NumSRSSymbols            - Number of consecutive SRS symbols (1 (default), 2, 4, 8, 10, 12, 14) 
    %   ResourceType             - Resource type ('periodic'(default), 'semi-persistent', 'aperiodic')
    %   SRSPeriod                - Slot periodicity and offset ('on' (default), 'off', [Tsrs Toffset])
    %   FrequencyStart           - Frequency position of the SRS in PRBs (0...271) (default 0)
    %   NRRC                     - Frequency offset in blocks of 4 RBs (0...67) (default 0)
    %   CSRS                     - Bandwidth configuration index C_SRS (0...63) (default 0) 
    %   BSRS                     - Bandwidth configuration index B_SRS (0...3) (default 0) 
    %   BHop                     - Frequency hopping index B_hop (0...3) (default 0)
    %   Repetition               - Repetition factor (1 (default), 2, 4, 5, 6, 7, 8, 10, 12, 14)
    %   KTC                      - Transmission comb number (2 (default), 4, 8)
    %   KBarTC                   - Transmission comb offset (0...KTC-1) (default 0)
    %   FrequencyScalingFactor   - Scaling for partial frequency sounding (1 (default), 2, 4)
    %   StartRBIndex             - Partial frequency sounding block (0...3) (default 0)
    %   EnableStartRBHopping     - Enable hopping for partial frequency sounding (false (default), true)
    %   CyclicShift              - Cyclic shift number offset (0...11) (default 0)    
    %   GroupSeqHopping          - Group or sequence hopping ('neither' (default), 'groupHopping', 'sequenceHopping')
    %   NSRSID                   - Scrambling identity (0...65535) (default 0)
    %   SRSPositioning           - Enable SRS for positioning (false (default), true)
    %   EnableEightPortTDM       - Enable 8-port time division multiplexing (false (default), true)
    %   CyclicShiftHopping       - Enable cyclic shift hopping (false (default), true)
    %   CyclicShiftHoppingID     - Cyclic shift hopping identity (default 0)
    %   CyclicShiftHoppingSubset - Cyclic shift hopping subset (default [])
    %   HoppingFinerGranularity  - Enable cyclic shift hopping finer granularity (false (default), true)
    %   CombOffsetHopping        - Enable comb offset hopping (false (default), true)
    %   CombOffsetHoppingID      - Comb offset hopping identity (default 0)
    %   CombOffsetHoppingSubset  - Comb offset hopping subset (default [])
    %   HoppingWithRepetition    - Enable comb offset hopping with repetition (false (default), true)
    %   
    %   nrSRSConfig properties (read-only):
    %
    %   NRB                     - Maximum number of RBs allocated to the SRS
    %   NRBPerTransmission      - Number of RBs allocated to the SRS in an OFDM symbol.
    %
    %   Constant properties: 
    %   
    %   BandwidthConfigurationTable - Table containing the SRS configuration
    %                                 parameters m_SRS and N in TS 38.211
    %                                 Table 6.4.1.4.3-1.
    %   SubcarrierOffsetTable       - Table containing subcarrier offsets
    %                                 per OFDM symbol and comb number for
    %                                 SRS user positioning specified in 
    %                                 TS 38.211 Table 6.4.1.4.3-2.
    %   StartRBHoppingTable         - Table containing frequency hopping
    %                                 offset indices specified in TS 38.211
    %                                 Table 6.4.1.4.3-3.
    %
    % Example:
    % % Create an SRS configuration object with 4 OFDM symbols.
    %
    % srs = nrSRSConfig('NumSRSSymbols',4,'SymbolStart',8);
    %
    % See also nrSRS, nrSRSIndices.
    
    % Copyright 2019-2023 The MathWorks, Inc.
        
    %#codegen
    
    % Public, tunable properties
    properties
        %SRSPeriod Slot periodicity and offset
        %   Specify the slot periodicity and offset values of the SRS. The
        %   possible options are ('on', 'off', [Tsrs Toffset]). When this
        %   property is set to 'on', then the SRS is present in all slots.
        %   When it is set to 'off', the resource is absent in all slots.
        %   For explicit values of SRSPeriod [Tsrs Toffset], the slot
        %   periodicity (Tsrs) and offset (Toffset) values are considered
        %   to determine the presence of the SRS in a given slot according
        %   to TS 38.211 Section 6.4.1.4.4. Tsrs must be one of 1, 2, 4, 5,
        %   8, 10, 16, 20, 32, 40, 64, 80, 160, 320, 640, 1280, 2560; and
        %   Toffset must be lower than Tsrs. SRSPeriod = 'on' is equivalent
        %   to SRSPeriod = [1 0]. The default value of SRSPeriod is 'on'.
        SRSPeriod = 'on';
        
        %ResourceType Time-domain behavior of SRS
        %   Specify the time-domain behavior of the SRS. The value must be
        %   one of {'periodic', 'semi-persistent', 'aperiodic'}. Downlink
        %   control information (DCI) triggers aperiodic SRS transmissions.
        %   For an 'aperiodic' resource type, SRSPeriod determines the
        %   periodicity and offset of the DCI triggering signal. Set
        %   ResourceType to 'aperiodic' to disable inter-slot frequency
        %   hopping. The default value is 'periodic'.
        ResourceType = nrSRSConfig.getDefault('ResourceType');
    end
    
    % Read-only properties
    properties (SetAccess = private)
        
        %NRB Number of RBs allocated to the SRS
        %   Number of RBs allocated to the SRS transmission. When frequency
        %   hopping is enabled, NRB denotes the hopping bandwidth or number
        %   of RBs over which the SRS signal hops across multiple time slots.
        NRB;
        
        %NRBPerTransmission Number of RBs allocated to the SRS at each OFDM symbol 
        % When frequency hopping is enabled, NRBPerTransmission specifies
        % the allocated bandwidth in RBs at each SRS transmission.
        NRBPerTransmission;
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        SRSPeriod_Options        = [1,2,4,5,8,10,16,20,32,40,64,80,160,320,640,1280,2560];
        SRSPeriod_CharOpt        = {'on','off'};
        ResourceType_Values      = {'periodic','semi-persistent','aperiodic'};
    end
    
    % Constant public properties
    properties (Constant)
        %BandwidthConfigurationTable SRS Bandwidth configuration table
        % Table containing the SRS configuration parameters m_SRS and N in
        % TS 38.211 Table 6.4.1.4.3-1.
        BandwidthConfigurationTable = getBandwidthConfigTable;

        %SubcarrierOffsetTable Subcarrier offset for SRS user positioning
        % Table containing subcarrier offsets per OFDM symbol and
        % transmission comb number for SRS user positioning specified in TS
        % 38.211 Table 6.4.1.4.3-2.
        SubcarrierOffsetTable = getOffsetKTable;

        %StartRBHoppingTable Frequency hopping offset table
        % Table containing frequency hopping offset indices for partial
        % frequency sounding specified in TS 38.211 Table 6.4.1.4.3-3.
        StartRBHoppingTable = getStartRBHoppingOffsetTable;
    end
    
    properties (Hidden)
    CustomPropList = {'NumSRSPorts'; 'SymbolStart'; 'NumSRSSymbols'; ...
         'ResourceType'; 'SRSPeriod'; 'FrequencyStart'; 'NRRC'; 'CSRS'; ...
         'BSRS'; 'BHop'; 'Repetition'; 'KTC'; 'KBarTC'; 'FrequencyScalingFactor'; ...
         'StartRBIndex'; 'EnableStartRBHopping'; 'CyclicShift'; 'GroupSeqHopping'; ...
         'NSRSID'; 'SRSPositioning'; 'EnableEightPortTDM'; 'CyclicShiftHopping';...
         'CyclicShiftHoppingID';'CyclicShiftHoppingSubset';'HoppingFinerGranularity';...
         'CombOffsetHopping';'CombOffsetHoppingID';'CombOffsetHoppingSubset';...
         'HoppingWithRepetition';...
         'NRB'; 'NRBPerTransmission'; 'BandwidthConfigurationTable'; ...
         'SubcarrierOffsetTable';'StartRBHoppingTable'};
    end
    
    methods
        
        % Constructor
        function obj = nrSRSConfig(varargin)
            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.srs.nrSRSConfigBase(...
                'ResourceType',nrSRSConfig.getDefault('ResourceType'),...
                varargin{:}); % Set variable-size properties for codegen compatibility
        end
        
        % Self-validate and set properties
        function obj = set.SRSPeriod(obj,val)
            propName = 'SRSPeriod';
            validateattributes(val,{'numeric','char','string'},...
                         {'nonempty'},[class(obj) '.' propName],propName);
            
            if ischar(val) || (isstring(val) && isscalar(val))
                % Character array or string scalar
                temp = validatestring(val,obj.SRSPeriod_CharOpt,[class(obj) '.' propName],propName);
                obj.(propName) = ''; % For codegen compatibility
            elseif isnumeric(val)
                validateattributes(val,{'numeric'},...
                    {'integer','vector','numel',2},[class(obj) '.' propName],propName);
                if ~any(val(1) == obj.SRSPeriod_Options)
                    coder.internal.error('nr5g:nrSRS:InvalidSlotPeriodicity',val(1));
                end
                if val(2) >= val(1) || val(2) < 0
                    coder.internal.error('nr5g:nrSRS:InvalidSlotOffset',val(2),val(1));
                end
                temp = val;
            end
            obj.(propName) = temp;
        end
        
        function obj = set.ResourceType(obj,val)
            propName = 'ResourceType';
            validateattributes(val,{'char','string'},...
                         {'nonempty'},[class(obj) '.' propName],propName);
            val = validatestring(val,obj.ResourceType_Values,[class(obj) '.' propName],propName);
            obj.(propName) = ''; % For codegen compatibility
            obj.(propName) = val;
        end
        
        % Read-only properties getters
        function val = get.NRB(obj)
            if obj.BHop < obj.BSRS % Frequency hopping cases
                val = nr5g.internal.srs.SRSBandwidthConfiguration(obj.CSRS,obj.BHop);
            else
                val = nr5g.internal.srs.SRSBandwidthConfiguration(obj.CSRS,obj.BSRS);
            end
        end
        
        function val = get.NRBPerTransmission(obj)
            val = nr5g.internal.srs.SRSBandwidthConfiguration(obj.CSRS,obj.BSRS)/double(obj.FrequencyScalingFactor);
        end

        % Validate config object and return a structure with SRS parameters
        function out = validateConfig(srs)

                validateConfig@nr5g.internal.srs.nrSRSConfigBase(srs);

                out = struct();
                out.NumSRSPorts = double(srs.NumSRSPorts);
                out.SymbolStart = double(srs.SymbolStart);
                out.NumSRSSymbols = double(srs.NumSRSSymbols);
                out.ResourceType = srs.ResourceType;
                if isnumeric(srs.SRSPeriod)
                    out.SRSPeriod = double(srs.SRSPeriod);
                else
                    out.SRSPeriod = srs.SRSPeriod;
                end                
                out.FrequencyStart = double(srs.FrequencyStart);
                out.NRRC = double(srs.NRRC);
                out.CSRS = double(srs.CSRS);
                out.BSRS = double(srs.BSRS);
                out.BHop = double(srs.BHop);
                out.Repetition = double(srs.Repetition);
                out.KTC = double(srs.KTC);
                out.KBarTC = double(srs.KBarTC);
                out.FrequencyScalingFactor = double(srs.FrequencyScalingFactor);
                out.StartRBIndex = double(srs.StartRBIndex);
                out.EnableStartRBHopping = srs.EnableStartRBHopping;
                out.CyclicShift = double(srs.CyclicShift);
                out.GroupSeqHopping = srs.GroupSeqHopping;
                out.NSRSID = double(srs.NSRSID);
                out.SRSPositioning = srs.SRSPositioning;
                out.EnableEightPortTDM = srs.EnableEightPortTDM;
                out.CyclicShiftHopping = srs.CyclicShiftHopping;
                out.CyclicShiftHoppingID = double(srs.CyclicShiftHoppingID);
                out.CyclicShiftHoppingSubset = double(srs.CyclicShiftHoppingSubset);
                out.HoppingFinerGranularity = srs.HoppingFinerGranularity;
                out.CombOffsetHopping = srs.CombOffsetHopping;
                out.CombOffsetHoppingID = double(srs.CombOffsetHoppingID);
                out.CombOffsetHoppingSubset = double(srs.CombOffsetHoppingSubset);
                out.HoppingWithRepetition = srs.HoppingWithRepetition;

        end
    end
    
    methods (Static, Access = private)
        % Default values of variable-size char properties. This allows to
        % localize the defaults required for codegen compatibility
        function out = getDefault(propName)
            switch propName
                case 'ResourceType'
                    out = 'periodic';
            end
        end
    end
end

%% Local functions
% Get bandwidth configuration table for MSRS
function t = getBandwidthConfigTable
    % Create TS 38.211 Table 6.4.1.4.3-1: SRS bandwidth configuration
    
    confTableArray = nr5g.internal.srs.SRSBandwidthConfiguration;
    
    columnNames = ["C_SRS", "m_SRS_0", "N_0", "m_SRS_1", "N_1", "m_SRS_2", "N_2", "m_SRS_3", "N_3"];
    
    % Package array in a table 
    t = array2table(confTableArray,"VariableNames",columnNames);
    t.Properties.VariableNames = columnNames;
    t.Properties.Description = 'TS 38.211 Table 6.4.1.4.3-1: SRS bandwidth configuration';
end

function t = getOffsetKTable
    % Create TS 38.211 Table 6.4.1.4.3-2: The offset k_offset for SRS as a function of K_TC and l'
    
    confTableArray = nr5g.internal.srs.SRSOffsetK;
    
    columnNames = ["K_TC", "NSRS_symb = 1", "NSRS_symb = 2", "NSRS_symb = 4", "NSRS_symb = 8", "NSRS_symb = 12"];
     
    % Package array into a table 
    t = cell2table(confTableArray,"VariableNames",columnNames);
    t.Properties.Description = 'TS 38.211 Table 6.4.1.4.3-2: The offset k_offset for SRS as a function of K_TC and l''';
end

function t = getStartRBHoppingOffsetTable
    % Create TS 38.211 Table 6.4.1.4.3-3

    confTableArray = nr5g.internal.srs.SRSStartRBHoppingOffset;

    columnNames = ["kBarHop", "PF = 1", "PF = 2", "PF = 4"];
    
    % Package array into a table 
    t = cell2table(confTableArray,"VariableNames",columnNames);
    t.Properties.Description = 'TS 38.211 Table 6.4.1.4.3-3: The quantity khop as a function of kBarhop''';

end
