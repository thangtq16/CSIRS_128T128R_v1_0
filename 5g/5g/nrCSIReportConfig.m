classdef nrCSIReportConfig < nr5g.internal.BWPSizeStart & comm.internal.ConfigBase
    %nrCSIReportConfig CSI report configuration object
    %   CSIREPORTCFG = nrCSIReportConfig creates a channel state
    %   information (CSI) report configuration object, CSIREPORTCFG. This
    %   object contains the properties related to TS 38.214 Section 5.2. By
    %   default, the object defines a CSI reporting configuration with the
    %   codebook type set to 'type1SinglePanel'.
    %
    %   CSIREPORTCFG = nrCSIReportConfig(Name=Value) creates a CSI report
    %   configuration object with the specified property Name set to the
    %   specified Value. You can specify additional name-value arguments in
    %   any order as (Name1=Value1,...,NameN=ValueN).
    %
    %   See also nrCSIRSConfig, nrCarrierConfig.

    %   Copyright 2024 The MathWorks, Inc.

    %#codegen
    properties
        %CQITable CQI table
        %   Specify the CQI table as a character array or a string scalar.
        %   It must be one of {'table1','table2','table3','table4-r17'},
        %   as defined in TS 38.214 Tables 5.2.2.1-2 through 5.2.2.1-4. The
        %   default value is "table1".
        CQITable = "table1"

        %CodebookType The type of codebook
        %    Specify the codebook type according to which the CSI
        %    parameters must be computed. It must be a character array or a
        %    string scalar. It must be one of {'type1SinglePanel',
        %    'type1MultiPanel','type2','eType2'}. In case of
        %    "type1SinglePanel", the PMI computation is performed using TS
        %    38.214 Tables 5.2.2.2.1-1 to 5.2.2.2.1-12. In case of
        %    "type1MultiPanel", the PMI computation is performed using TS
        %    38.214 Tables 5.2.2.2.2-1 to 5.2.2.2.2-6. In case of "type2"
        %    the computation is performed according to TS 38.214 Section
        %    5.2.2.2.3. In case of "eType2" the computation is performed
        %    according to TS 38.214 Section 5.2.2.2.5. The default value is
        %    "type1SinglePanel".
        CodebookType = "type1SinglePanel"

        %PanelDimensions Antenna panel configuration
        %     Specify the antenna panel configuration as a three-element
        %     row vector in the form of [Ng N1 N2]. Ng represents the
        %     number of antenna panels, N1 represents the number of antenna
        %     elements in horizontal direction and N2 represents the number
        %     of antenna elements in vertical direction. When CodebookType
        %     is specified as "type1SinglePanel" or "type2", or "eType2",
        %     Ng is 1 and valid combinations of [N1 N2] are defined in TS
        %     38.214 Table 5.2.2.2.1-2. This is not applicable when the
        %     number of CSI-RS ports is less than or equal to 2. When
        %     CodebookType is specified as "type1MultiPanel", valid
        %     combinations of [Ng N1 N2] are defined in TS 38.214 Table
        %     5.2.2.2.2-1.
        PanelDimensions (1,3) {mustBeNumeric, mustBeInteger, mustBePositive} = [1 2 1]

        %CQIFormatIndicator Mode of CQI reporting
        %   Specify the mode of CQI reporting as a character array or a
        %   string scalar. It must be one of {'wideband','subband'}. The
        %   default value is "wideband".
        CQIFormatIndicator = "wideband"

        %PMIFormatIndicator Mode of PMI reporting
        %   Specify the mode of PMI reporting as a character array or a
        %   string scalar. It must be one of {'wideband','subband'}. The
        %   default value is "wideband".
        PMIFormatIndicator = "wideband"

        %SubbandSize Subband size
        %   Specify the subband size for CQI or PMI reporting as a positive
        %   scalar integer. It must be one of two possible subband sizes
        %   based on the BWP size, as defined in TS 38.214 Table 5.2.1.4-2.
        %   It is applicable only when either CQIFormatIndicator or
        %   PMIFormatIndicator are provided as "subband" and the size of
        %   BWP is greater than or equal to 24 PRBs.
        SubbandSize (1,1) {mustBeNumeric, mustBeMember(SubbandSize, [4 8 16 32])} = 4

        %PRGBundleSize Precoding resource block group (PRG) size
        %   Specify the PRG size for CQI calculation as a scalar positive
        %   integer. It must be one of {2, 4} and it is provided by the
        %   higher-layer parameter pdsch-BundleSizeForCSI. Empty ([]) is
        %   also supported to represent that this is not configured by
        %   higher layer parameters. This is applicable to the CSI report
        %   quantity cri-RI-i1-CQI when the CodebookType is
        %   "type1SinglePanel", as defined in TS 38.214 Section 5.2.1.4.2.
        %   This report quantity expects only the i1 set of PMI to be
        %   reported as part of CSI parameters irrespective of PMI
        %   reporting mode. If PRGBundleSize is not configured as empty,
        %   the CQI values are computed according to the configured
        %   CQIFormatIndicator, as defined in TS 38.214 Section 5.2.1.4.2.
        %   The default value is [].
        PRGBundleSize = []

        %CodebookMode Codebook mode
        %   Specify the codebook mode as scalar positive integer. The value
        %   must be one of {1, 2}.
        %   - When CodebookType is specified as
        %     "type1SinglePanel", this is applicable only if the
        %     number of transmission layers is 1 or 2 and number of CSI-RS
        %     ports is greater than 2
        %   - When CodebookType is specified as
        %     "type1MultiPanel", this is applicable for all the
        %     number of transmission layers and the CodebookMode value 2 is
        %     applicable only for the panel configurations with Ng value 2
        %   This is not applicable for CodebookType "type2" or
        %   "eType2". The default value is 1.
        CodebookMode (1,1) {mustBeNumeric, mustBeMember(CodebookMode, [1 2])} = 1

        %CodebookSubsetRestriction Codebook subset restriction
        %   Specify the codebook subset restriction as a binary vector
        %   (right-msb).
        %   - When the CodebookType is specified as
        %     "type1SinglePanel" or "type1MultiPanel" and the number of
        %     CSI-RS ports is greater than 2, the number of elements in the
        %     input vector must be N1*N2*O1*O2, where N1 and N2 are panel
        %     configurations obtained from PanelDimensions property and O1
        %     and O2 are the respective discrete Fourier transform (DFT)
        %     oversampling factors obtained from TS.38.214 Table
        %     5.2.2.2.1-2 for "type1SinglePanel" codebook type or TS.38.214
        %     Table 5.2.2.2.2-1 for "type1MultiPanel" codebook type. When
        %     the number of CSI-RS ports is 2 with the codebook type as
        %     "type1SinglePanel", the number of elements in the input
        %     vector must be 6, as defined in TS 38.214 Section 5.2.2.2.1
        %    - When CodebookType is specified as
        %      "type2" or "eType2", this is a bit vector which is obtained
        %      by concatenation of two bit vectors [B1 B2]. B1 is a bit
        %      vector of 11 bits (right-msb) when N2 of the panel
        %      dimensions is greater than 1 and 0 bits otherwise. B2 is a
        %      combination of 4 bit vectors, each with 2*N1*N2 number of
        %      elements. B1 denotes 4 sets of beam groups for which
        %      restriction is applicable. When CodebookType is specified as
        %      "type2", B2 denotes the maximum allowable amplitude for each
        %      of the DFT vectors in each of the respective beam groups
        %      denoted by B1. When CodebookType is specified as "eType2",
        %      B2 denotes the maximum average coefficient amplitude for
        %      each of the DFT vectors in each of the respective beam
        %      groups denoted by B1. The default value is empty ([]), which
        %      means there is no codebook subset restriction.
        CodebookSubsetRestriction = []

        %I2Restriction I2 values restriction in a codebook
        %   Specify the i2 values restriction as a binary vector. The
        %   number of elements in the input vector must be 16. First
        %   element of the input binary vector corresponds to i2 as 0,
        %   second element corresponds to i2 as 1, and so on. Binary value
        %   1 indicates that the precoding matrix associated with the
        %   respective i2 is unrestricted and 0 indicates that the
        %   precoding matrix associated with the respective i2 is
        %   restricted. For a precoding matrices codebook, if the number of
        %   possible i2 values are less than 16, then only the required
        %   binary elements are considered and the trailing extra elements
        %   in the input vector are ignored. This is applicable only when
        %   the number of CSI-RS ports is greater than 2 and the
        %   CodebookType is specified as "type1SinglePanel". The default
        %   value is empty ([]), which means there is no i2 restriction.
        I2Restriction = []

        %RIRestriction Rank indicator (RI) restriction
        %   Specify the restricted rank set as a binary vector. When
        %   CodebookType is specified as "type1SinglePanel", the number of
        %   elements in the imput vector must be 8. When CodebookType is
        %   specified as "type1MultiPanel" or "eType2", the number of
        %   elements in the input vector must be 4. When the CodebookType
        %   is specified as "type2", the number of elements in the input
        %   vector must be 2. In all the cases, the first element
        %   corresponds to rank 1, second element corresponds to rank 2,
        %   and so on. The binary value 0 represents that the corresponding
        %   rank is restricted and the binary value 1 represents that the
        %   corresponding rank is unrestricted. The default value is empty
        %   ([]), which means there is no rank restriction.
        RIRestriction = []

        %NumberOfBeams Number of beams
        %   Specify the number of beams to be considered in the beam group
        %   as a scalar positive integer. This is applicable only
        %   when the CodebookType is specified as "type2". The value must
        %   be one of {2, 3, 4}. The default value is 2.
        NumberOfBeams (1,1) {mustBeNumeric, mustBeMember(NumberOfBeams, [2, 3, 4])} = 2

        %SubbandAmplitude Flag to enable the subband amplitudes reporting
        %   Specify the subband amplitudes reporting as a logical scalar.
        %   When set to true amplitudes are reported for each subband and
        %   not reported when set to false. The value must be one of {true,
        %   false}. This is applicable when CodebookType is specified
        %   as "type2" and PMIFormatIndicator is "subband". The default
        %   value is false.
        SubbandAmplitude (1,1) {mustBeNumericOrLogical} = false

        %PhaseAlphabetSize Range of phase amplitudes
        %   Specify the phase alphabet size as a scalar which represents
        %   the range of the phases that are to be considered for the
        %   computation of PMI i2 indices. This is applicable only
        %   when the CodebookType is specified as "type2". The value must
        %   be one of {4, 8}. The value 4 represents the phases
        %   corresponding to QPSK and the value 8 represents the phases
        %   corresponding to 8-PSK. The default value is 4. This is
        %   not a configurable parameter for "eType2" and it is fixed as
        %   16, which corresponds to 16-PSK.
        PhaseAlphabetSize (1,1) {mustBeNumeric, mustBeMember(PhaseAlphabetSize, [4, 8])} = 4

        %ParameterCombination Parameter combination index
        %   Specify the parameter combination index as a positive scalar
        %   integer in the range 1...8. This is applicable when
        %   CodebookType is specified as "eType2". This parameter defines
        %   the number of beams and two other parameters as defined in TS
        %   38.214 Table 5.2.2.2.5-1. The default value is 1.
        ParameterCombination (1,1) {mustBeNumeric, mustBeMember(ParameterCombination, 1:8)} = 1

        %NumberOfPMISubbandsPerCQISubband Number of PMI subbands per CQI subband
        %   Specify the number of PMI subbands within one CQI subband as a
        %   positive scalar integer. It must be either 1 or 2. This
        %   is applicable when CodebookType is specified as "eType2". The
        %   default value is 1.
        NumberOfPMISubbandsPerCQISubband (1,1) {mustBeNumeric, mustBeMember(NumberOfPMISubbandsPerCQISubband, 1:2)} = 1
    end

    % Constant properties
    properties(Constant)
        %Tables Structure containing the tables related to CSI parameters
        %computation:
        %       - SinglePanelConfigurations   - TS 38.214 Table 5.2.2.2.1-2
        %       - MultiPanelConfigurations    - TS 38.214 Table 5.2.2.2.2-1
        %       - EnhancedType2Configurations - TS 38.214 Table 5.2.2.2.5-1

        Tables = struct("SinglePanelConfigurations", getSinglePanelConfigurations, ...    % TS 38.214 Table 5.2.2.2.1-2
                        "MultiPanelConfigurations", getMultiPanelConfigurations, ...      % TS 38.214 Table 5.2.2.2.2-1
                        "EnhancedType2Configurations", eType2ParameterConfigurationTable);% TS 38.214 Table 5.2.2.2.5-1
    end % Constant properties

    % Hidden properties
    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'NSizeBWP','NStartBWP','CQITable','CodebookType',...
            'PanelDimensions','CQIFormatIndicator','PMIFormatIndicator','SubbandSize','PRGBundleSize',...
            'CodebookMode','CodebookSubsetRestriction','I2Restriction','RIRestriction',...
            'NumberOfBeams','SubbandAmplitude','PhaseAlphabetSize','ParameterCombination','NumberOfPMISubbandsPerCQISubband','Tables'};
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        CQITable_Values = {'table1','table2','table3','table4-r17'};
        % ThangTQ23_128T128R_eTypeII_Rel19: added 'eTypeII-r19' (TS 38.214 §5.2.2.2.5a)
        CodebookType_Values = {'type1SinglePanel','type1MultiPanel','type2','eType2','typeI-SinglePanel-r19','eTypeII-r19'};
        CQIFormatIndicator_Values = {'wideband','subband'};
        PMIFormatIndicator_Values = {'wideband','subband'};
    end

    methods
        %Default constructor
        function obj = nrCSIReportConfig(varargin)
            % Get the value of NStartBWP from the name-value pairs
            nStartBWP = nr5g.internal.parseProp("NStartBWP",[],varargin{:});
            % Get the value of NSizeBWP from the name-value pairs
            nSizeBWP = nr5g.internal.parseProp("NSizeBWP",[],varargin{:});
            % Get the value of PRGBundleSize from the name-value pairs
            prgBundleSize = nr5g.internal.parseProp("PRGBundleSize",[],varargin{:});
            % Get the value of CodebookSubsetRestriction from the name-value pairs
            codebookSubsetRestriction = nr5g.internal.parseProp("CodebookSubsetRestriction",[],varargin{:});
            % Get the value of I2Restriction from the name-value pairs
            i2Restriction = nr5g.internal.parseProp("I2Restriction",[],varargin{:});
            % Get the value of RIRestriction from the name-value pairs
            riRestriction = nr5g.internal.parseProp("RIRestriction",[],varargin{:});

            obj@comm.internal.ConfigBase(...
                "NStartBWP", nStartBWP, ...
                "NSizeBWP", nSizeBWP, ...
                "PRGBundleSize", prgBundleSize,...
                "CodebookSubsetRestriction", codebookSubsetRestriction,...
                "I2Restriction", i2Restriction,...
                "RIRestriction", riRestriction,varargin{:});
        end
    end

    methods (Access = protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            inactive = false;
            switch prop
                case {"NumberOfBeams", "PhaseAlphabetSize", "SubbandAmplitude"}
                    inactive = ~strcmpi(obj.CodebookType,"type2");
                case {"ParameterCombination", "NumberOfPMISubbandsPerCQISubband"}
                    % ThangTQ23_128T128R_eTypeII_Rel19: eTypeII-r19 uses same params
                    inactive = ~(strcmpi(obj.CodebookType,"eType2") || strcmpi(obj.CodebookType,"eTypeII-r19"));
                case {"SubbandSize"}
                    inactive = ~(strcmpi(obj.PMIFormatIndicator,"subband") || strcmp(obj.CQIFormatIndicator,"subband"));
                case {"PRGBundleSize","I2Restriction"}
                    inactive = ~strcmpi(obj.CodebookType,"type1SinglePanel");
            end
        end
    end

    methods
        % Self-validate and set properties
        function obj = set.CQITable(obj,val)
            prop = 'CQITable';
            temp = validatestring(val,obj.CQITable_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = temp;
        end

        function obj = set.CodebookType(obj,val)
            prop = 'CodebookType';
            temp = validatestring(val,obj.CodebookType_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = temp;
        end

        function obj = set.CQIFormatIndicator(obj,val)
            prop = 'CQIFormatIndicator';
            temp = validatestring(val,obj.CQIFormatIndicator_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = temp;
        end

        function obj = set.PMIFormatIndicator(obj,val)
            prop = 'PMIFormatIndicator';
            temp = validatestring(val,obj.PMIFormatIndicator_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = temp;
        end

        function obj = set.PRGBundleSize(obj,val)
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize("temp",[1 1],[1 1]);
            prop = 'PRGBundleSize';
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'real','scalar'},[class(obj) '.' prop],prop);
                coder.internal.errorIf(~any(temp(1) == [2 4]),"nr5g:nrCSIReportConfig:InvalidPRGBundleSize",sprintf('%g',double(temp(1))));
            end
            obj.(prop) = temp;
        end

        function obj = set.CodebookSubsetRestriction(obj,val)
            % To allow codegen for varying length in a single function script
            temp = val;
            prop = 'CodebookSubsetRestriction';
            coder.varsize("temp",[Inf Inf],[1 1]); % It is of variable length based on the codebook type
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric','logical'},{'vector','binary'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.I2Restriction(obj,val)
            % To allow codegen for varying length in a single function script
            temp = val;
            prop = 'I2Restriction';
            coder.varsize("temp",[16 16],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric','logical'},{'vector','binary','numel',16},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.RIRestriction(obj,val)
            % To allow codegen for varying length in a single function script
            temp = val;
            prop = 'RIRestriction';
            coder.varsize("temp",[8 8],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric','logical'},{'vector','binary'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function val = get.NumberOfBeams(obj)
            val = obj.NumberOfBeams;
            if strcmpi(obj.CodebookType,'eType2') || strcmpi(obj.CodebookType,'eTypeII-r19')
                % Provide the value of NumberOfBeams for enhanced type 2
                % codebooks based on ParameterCombination, as defined in
                % TS 38.214 Table 5.2.2.2.5-1.
                % ThangTQ23_128T128R_eTypeII_Rel19: eTypeII-r19 uses same table
                configTable = eType2ParameterConfigurationTable;
                val = configTable{obj.ParameterCombination,2};
            end
        end

        function val = get.PhaseAlphabetSize(obj)
            val = obj.PhaseAlphabetSize;
            if strcmpi(obj.CodebookType,'eType2') || strcmpi(obj.CodebookType,'eTypeII-r19')
                % Return the value of PhaseAlphabetSize for enhanced type 2
                % codebooks as 16, as defined in TS 38.214 Section
                % 5.2.2.2.5.
                % ThangTQ23_128T128R_eTypeII_Rel19: eTypeII-r19 fixed at 16-PSK
                val = 16;
            end
        end

        function obj = validateConfig(obj)
            % Validate the number of elements of RIRestriction based on the codebook type
            temp = obj.RIRestriction;
            switch obj.CodebookType
                case "type1SinglePanel"
                    maxRank = 8;
                case "type2"
                    maxRank = 2;
                otherwise % "type1MultiPanel", "eType2", or "eTypeII-r19"
                    % ThangTQ23_128T128R_eTypeII_Rel19: max RI=4 per §5.2.2.2.5a
                    maxRank = 4;
            end
            riRestrictionLen = numel(temp);
            if riRestrictionLen
                coder.internal.errorIf(riRestrictionLen ~= maxRank,...
                    "nr5g:nrCSIReportConfig:InvalidRIRestriction",...
                    obj.CodebookType,numel(temp),maxRank);
            else
                % Update the RIRestriction property with all ones to
                % consider no restriction, in case of empty
                obj.RIRestriction = ones(1,maxRank);
            end

            % Update the I2Restriction property with all ones to consider
            % no restriction, in case of empty
            temp = obj.I2Restriction;
            if isempty(temp)
                obj.I2Restriction = ones(1,16);
            end
        end

    end
end
function configTable = eType2ParameterConfigurationTable
    % Returns the parameter configuration table as per TS 38.214
    % Table 5.2.2.2.5-1.
    % Each column represents the following:
    %   Column 1: ParamCombination_R16
    %   Column 2: L
    %   Column 3: Pv_1or2Layers
    %   Column 4: Pv_3or4Layers
    %   Column 5: Beta

    configTableEntries = ...
        {
            1   2     1/4       1/8     1/4;
            2   2     1/4       1/8     1/2;
            3   4     1/4       1/8     1/4;
            4   4     1/4       1/8     1/2;
            5   4     1/4       1/4     3/4;
            6   4     1/2       1/4     1/2;
            7   6     1/4       NaN     1/2;
            8   6     1/4       NaN     3/4};
    configTable = cell2table(configTableEntries,'VariableNames',{'ParamCombination_R16','L','Pv_1or2Layers','Pv_3or4Layers','Beta'});
    configTable.Properties.Description = 'TS 38.214 Table 5.2.2.2.5-1: Codebook parameter configurations for L, Beta, and Pv';
end

function panelConfigTable = getSinglePanelConfigurations
    % Supported panel configurations and oversampling factors
    % for single-panel codebooks, as defined in
    % TS 38.214 Table 5.2.2.2.1-2
    % Each column represents the following:
    %   Column 1: Number of CSI-RS antenna ports
    %   Column 2: (N1,N2)
    %   Column 3: (O1,O2)

    panelConfigEntries = ...
        {...
        %  Number of
        %   CSI-RS    (N1,N2)   (O1,O2)
        %   ports
            4          [2 1]    [4 1];
            8          [2 2]    [4 4];
            8          [4 1]    [4 1];
            12         [3 2]    [4 4];
            12         [6 1]    [4 1];
            16         [4 2]    [4 4];
            16         [8 1]    [4 1];
            24         [4 3]    [4 4];
            24         [6 2]    [4 4];
            24         [12 1]   [4 1];
            32         [4 4]    [4 4];
            32         [8 2]    [4 4];
            32         [16 1]   [4 1];
            % --- Rel-19 Refined Type I Single-Panel (TS 38.214 Table 5.2.2.2.1a-1) ---
            % ThangTQ23_128T128R_Rel19
            48         [8 3]    [4 4];
            48         [6 4]    [4 4];
            48         [16 2]   [4 4];
            64         [8 4]    [4 4];
            128        [16 4]   [4 4];
            128        [8 8]    [4 4]};
    panelConfigTable = cell2table(panelConfigEntries,'VariableNames',{'Number of CSI-RS antenna ports','(N1,N2)','(O1,O2)'});
    panelConfigTable.Properties.Description = 'TS 38.214 Table 5.2.2.2.1-2 (Rel-15/16) + Table 5.2.2.2.1a-1 (Rel-19): Supported configurations of (N1,N2) and (O1,O2)';
end

function panelConfigTable = getMultiPanelConfigurations
    % Supported panel configurations and oversampling factors
    % for multi-panel codebooks, as defined in TS 38.214 Table
    % 5.2.2.2.2-1
    %   Column 1: Number of CSI-RS antenna ports
    %   Column 2: (Ng,N1,N2)
    %   Column 3: (O1,O2)

    panelConfigEntries = ...
        {
        % Number of
        % CSI-RS      (Ng,N1,N2)    (O1,O2)
        % ports
            8         [2 2 1]       [4 1];
            16        [2 4 1]       [4 1];
            16        [4 2 1]       [4 1];
            16        [2 2 2]       [4 4];
            32        [2 8 1]       [4 1];
            32        [4 4 1]       [4 1];
            32        [2 4 2]       [4 4];
            32        [4 2 2]       [4 4]};
    panelConfigTable = cell2table(panelConfigEntries,'VariableNames',{'Number of CSI-RS antenna ports','(Ng,N1,N2)','(O1,O2)'});
    panelConfigTable.Properties.Description = 'TS 38.214 Table 5.2.2.2.1-2: Supported configurations of (Ng,N1,N2) and (O1,O2)';
end