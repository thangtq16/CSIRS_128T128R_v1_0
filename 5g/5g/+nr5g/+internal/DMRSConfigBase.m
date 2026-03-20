classdef (Abstract) DMRSConfigBase < comm.internal.ConfigBase
    %DMRSConfigBase Base object for DM-RS configuration object
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    % Copyright 2019-2024 The MathWorks, Inc.

    %#codegen

    properties (Abstract)
        % Abstract properties, specifically so that derived classes can supply their own overriding MATLAB help text
        DMRSConfigurationType (1,1) {mustBeMember(DMRSConfigurationType, [1 2])};
        NumCDMGroupsWithoutData (1,1) {mustBeMember(NumCDMGroupsWithoutData, [1 2 3])};
    end

    properties (Abstract,Constant,Access=protected)
        % Derived classes must define the frequency domain OCC vectors (each column is a different FD cover)
        FDOCCTable (4,4);
    end

    properties (Dependent, Hidden)
        %Ports The active DM-RS ports configured, based on the DM-RS port set and
        % the number of layers
        Ports;
    end

    properties (Hidden)
        %NLayers Number of transmission layers in the associated shared channel
        % transmission
        NLayers = 1;
    end

    properties

        %DMRSTypeAPosition Position of first DM-RS OFDM symbol in a slot
        %   Specify the position of first DM-RS OFDM symbol in a slot, as a
        %   scalar positive integer. The value must be one of {2, 3},
        %   provided by higher-layer parameter dmrs-TypeA-Position. This
        %   property is applicable only when the physical shared channel
        %   mapping type is configured to be 'A'. The default value is 2.
        DMRSTypeAPosition (1,1) {mustBeNumeric,mustBeMember(DMRSTypeAPosition,[2 3])} = 2;

        %DMRSAdditionalPosition Maximum number of DM-RS additional positions
        %   Specify the maximum number of DM-RS additional positions as a
        %   scalar nonnegative integer. The value must be one of {0, 1, 2,
        %   3}, provided by higher-layer parameter dmrs-AdditionalPosition.
        %   The default value is 0.
        DMRSAdditionalPosition (1,1) {mustBeNumeric,mustBeMember(DMRSAdditionalPosition,[0 1 2 3])} = 0;

        %DMRSLength Number of consecutive DM-RS OFDM symbols
        %   Specify the number of consecutive OFDM symbols carrying DM-RS
        %   as a scalar positive integer. The value must be one of {1, 2}.
        %   The value of 1 implies single-symbol DM-RS and the value of 2
        %   implies double-symbol DM-RS. The default value is 1.
        DMRSLength (1,1) {mustBeNumeric,mustBeMember(DMRSLength,[1 2])} = 1;

        %CustomSymbolSet Custom DM-RS symbol set (0-based)
        %   Specify the custom set of DM-RS OFDM symbol locations as a
        %   vector of nonnegative integers. Use these values to override
        %   the DM-RS symbol locations defined in the TS 38.211 DM-RS
        %   tables. Note that the values provided here are treated as
        %   single symbols. The default value is [], which corresponds to
        %   no override of the standard defined values.
        CustomSymbolSet = [];

        %DMRSPortSet DM-RS antenna ports
        %   Specify the DM-RS antenna ports as a vector of integers. The
        %   valid set of port numbers depends on the DM-RS configuration
        %   type, the DM-RS duration, and whether enhanced DM-RS multiplexing
        %   is enabled. The default value is [], when the port indices are
        %   the first N valid ports, where N = NumLayers as defined in the
        %   associated shared channel configuration.
        DMRSPortSet = [];

        % NIDNSCID DM-RS scrambling identities (NID^0 and NID^1)
        %   Specify the DM-RS scrambling identities (NID^0 and NID^1). The
        %   property can be a two-element vector ([NID^0, NID^1]), a scalar
        %   (NID^0=NID^1), or empty ([]). The values must be in range 0...65535.
        %   When the property is empty, the DM-RS scrambling identity is equal
        %   to <a href="matlab:help('nrCarrierConfig/NCellID')">NCellID</a> of nrCarrierConfig.
        %   The default value is [].
        NIDNSCID = [];

        %NSCID DM-RS scrambling initialization (nSCID)
        %   Specify the DM-RS scrambling initialization as a scalar
        %   nonnegative integer. The value must be one of {0, 1}. The
        %   default value is 0.
        NSCID (1,1) {mustBeNumeric, mustBeMember(NSCID,[0 1])} = 0;

        %DMRSEnhancedR18 Enhanced DM-RS multiplexing
        %   Specify the use of enhanced DM-RS multiplexing. Setting this
        %   property to 1 enables an extended set of DM-RS antenna ports
        %   for MU-MIMO use. The actual port numbers selected depend on
        %   other configuration parameters. The default value is 0.
        DMRSEnhancedR18 (1,1) logical = 0;

    end

    properties (SetAccess = private)
        %CDMGroups CDM group number(s) corresponding to each port
        %   A row vector of CDM group number(s) corresponding to each port,
        %   depending on the DM-RS configuration type.
        CDMGroups;

        %DeltaShifts Delta shift(s) corresponding to each CDM group
        %   A row vector of delta shift(s) corresponding to each CDM group,
        %   depending on the DM-RS configuration type.
        DeltaShifts;

        %FrequencyWeights Frequency weights (w_f) corresponding to each port
        %   A matrix of frequency weights to be applied to the DM-RS
        %   symbols, depending on the DM-RS configuration type and the port
        %   set. Each column corresponds to the weights to be applied for
        %   that port.
        FrequencyWeights;

        %TimeWeights Time weights (w_t) corresponding to each port
        %   A matrix of time weights to be applied to the DM-RS symbols,
        %   depending on the DM-RS configuration type and the port set.
        %   Each column corresponds to the weights to be applied for that
        %   port.
        TimeWeights;

        %DMRSSubcarrierLocations Subcarrier locations in a resource block for each port
        %   A matrix of subcarrier locations of DM-RS symbols in a resource
        %   block for each port. Each column corresponds to the subcarrier
        %   locations for that port.
        DMRSSubcarrierLocations;

        %CDMLengths CDM lengths in frequency and time domain
        %   A two-element row vector [FD TD] specifying the length of
        %   FD-CDM and TD-CDM despreading. The values depend on the
        %   frequency and time masks to be applied to the CDM groups.
        CDMLengths;
    end

    properties (Transient,Access=private)
        AllValidPortNumbers;   % Set of all valid port numbers for this DM-RS configuration

        % For code generation, the data type of this property needs to
        % remain constant as it is accessed by updateSupportedPortNumbers
        % during object construction
        DMRSLengthInternal = 1;
    end

    properties (Abstract,Transient,Access=protected)
        % For code generation, the data type of this property needs to
        % remain constant as it is accessed by updateSupportedPortNumbers
        % during object construction
        DMRSConfigurationTypeInternal;
    end

    methods

        function obj = DMRSConfigBase(varargin)
            %DMRSConfigBase Construct a DMRSConfigBase object
            %   Set the property values from any name-value pairs input to
            %   the object
            obj@comm.internal.ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow properties to be var-size in codegen
                obj.DMRSPortSet = nr5g.internal.parseProp('DMRSPortSet',[],varargin{:});
                obj.CustomSymbolSet = nr5g.internal.parseProp('CustomSymbolSet',[],varargin{:});
                obj.NIDNSCID = nr5g.internal.parseProp('NIDNSCID',[],varargin{:});
            end
            obj = updateSupportedPortNumbers(obj);
        end

        function obj = set.DMRSLength(obj,value)
            obj.DMRSLength = value;
            obj.DMRSLengthInternal = double(value); %#ok<MCSUP>
            obj = updateSupportedPortNumbers(obj);
        end

        function obj = set.DMRSEnhancedR18(obj,value)
            obj.DMRSEnhancedR18 = logical(value);
            obj = updateSupportedPortNumbers(obj);
        end

        function obj = set.CustomSymbolSet(obj,val)
            prop = 'CustomSymbolSet';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp, {'numeric'},...
                    {'vector','integer','nonnegative','nonempty','<=',13},...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.DMRSPortSet(obj,val)
            prop = 'DMRSPortSet';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp, {'numeric'},...
                    {'vector','integer','nonnegative','nonempty','<=',23},...   % 0-7 for type 1, and 0-11 for type 2 , and these get doubled for eType 1 (0-15)/eType 2 (0-23)
                    [class(obj) '.' prop], prop);
            end
            obj.(prop) = temp;
        end

        function obj = set.NIDNSCID(obj,val)
            prop = 'NIDNSCID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[2 2],[1 1])
            % Check array length
            coder.internal.errorIf(~any(length(temp) == [0 1 2]),...
                'nr5g:DMRSConfigBase:InvalidNIDNSCIDLength');
            % Check values
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp, {'numeric'},...
                    {'integer','nonnegative','<=',65535},...
                    [class(obj) '.' prop], prop);
            end
            obj.(prop) = temp;
        end

        % Dependent property
        function val = get.Ports(obj)
            % The antenna ports configured for the shared channel. It is equal to
            % DMRSPortSet or derived from NLayers (if DMRSPortSet is empty)
            if ~isempty(obj.DMRSPortSet)
                val = reshape(double(obj.DMRSPortSet),1,[]);   % Return them as a row always
            else
                % Given that this property getter must always succeed, get the closest valid set of DM-RS ports
                % for the Nlayers input and DM-RS configuration at the time
                % validports = getValidPortNumbers(obj);
                validports = obj.AllValidPortNumbers;
                val = validports(1:min(obj.NLayers,end));
            end
        end

        % Read-only properties
        function val = get.CDMGroups(obj)
            % The CDM group number(s) for each port
            val = reshape(...
                mod(fix(double(obj.Ports)/2),double(obj.DMRSConfigurationType)+1),1,[]);
        end

        % Summary comments on the relationship between ports numbers and
        % associated properties, as used in the properties below
        %
        % Stepping through the port numbers
        % - Cycle through FD covers (FDD part of CDM group)
        % - Cycle through shift deltas (each group by FDD part)
        % - Cycle through TD covers (same, but now double symbol TD part)
        % Then do the same with the enhanced second pair of FD covers for CDM groups

        function val = get.DeltaShifts(obj)
            % The delta shift for the CDM group number of each port number
            val = double(obj.DMRSConfigurationType)*obj.CDMGroups;  % Multiply the CDM group numbers by 1 or 2 to give the associated delta shift
        end

        function val = get.FrequencyWeights(obj)
            %  The frequency weights for each port, where each weight vector is a column (port) in the returned matrix
            fdocclen = 2 + 2*double(obj.DMRSEnhancedR18);  % FD OCC table length is 2 or 4
            maxports = 2*[8 12];                           % Number of ports for R18 type 1 and type 2 (R15 is half this number)
            tablesplit = maxports(obj.DMRSConfigurationType)/(1+(fdocclen==4));  % Use top half of table if above split
            fdoccindices = mod(obj.Ports,2) + 2*mod(fix(obj.Ports/tablesplit),2);
            val = obj.FDOCCTable(1:fdocclen,1+fdoccindices);
        end

        function val = get.TimeWeights(obj)
            % The time weights for each port, where each weight pair is a column (port) in the returned matrix
            ports = obj.Ports;
            val = ones(2,numel(ports));
            nportsallgroups = 2+2*double(obj.DMRSConfigurationType);            % Number of ports in a single set of all CDM groups: 4 ports for type 1 (CDM = 0,1) or 6 ports for type 2 (CDM = 0,1,2)
            doubleSymbolOnlyPorts = logical(mod(fix(ports/nportsallgroups),2)); % Identify ports where the second weight of the TD cover must be a -1 (therefore double symbol DM-RS only)
            val(2,doubleSymbolOnlyPorts) = -1;
        end

        function val = get.DMRSSubcarrierLocations(obj)
            % The subcarrier locations of each port in a resource block
            deltaShifts = obj.DeltaShifts;
            if obj.DMRSConfigurationType == 1
                val = repmat((0:2:10)',1,numel(deltaShifts))+repmat(deltaShifts,6,1);   % Type 1: 6 DM-RS symbols per PRB
            else
                val = repmat([0 1 6 7]',1,numel(deltaShifts))+repmat(deltaShifts,4,1);  % Type 2: 4 DM-RS symbols per PRB
            end
        end

        function val = get.CDMLengths(obj)
            % The CDM lengths [FD, TD]

            if ~obj.DMRSEnhancedR18
                % Initialize CDM lengths for each CDM group
                cdmgroups = obj.CDMGroups;
                cdmLen = zeros(numel(cdmgroups),2);
                % Get the number of unique frequency weights and time weights
                % applied for each CDM group, to determine lengths of FD-CDM
                % and TD-CDM despreading
                freqWeights = sum(obj.FrequencyWeights);
                timeWeights = sum(obj.TimeWeights);
                for gi = 1:numel(cdmgroups)
                    cdmLen(gi,1) = numel(unique(freqWeights(cdmgroups == cdmgroups(gi))));
                    cdmLen(gi,2) = numel(unique(timeWeights(cdmgroups == cdmgroups(gi))));
                end
                val = max(cdmLen,[],1);
            else
                val = [4 2];
            end

            % Overwrite the value of TD with 1, when the DM-RS length is 1
            % or when custom DM-RS symbol set is not empty and in use
            if ~isempty(obj.CustomSymbolSet) || obj.DMRSLength == 1
                val(end) = 1;
            end
        end

    end

    methods (Access=public)

        function validateConfig(obj)
            % This method validates inter-dependencies between properties

            % If DMRSPortSet was provided then check that it is valid against
            % those defined by the rest of the configuration
            if ~isempty(obj.DMRSPortSet)
                [txt,validportstxt] = checkPorts(obj,obj.DMRSPortSet);
                flag = ~isempty(txt);
                coder.internal.errorIf(flag,['nr5g:',class(obj),':InvalidDMRSPortSet'],...
                    txt,validportstxt,double(obj.DMRSConfigurationType),double(obj.DMRSLength),double(obj.DMRSEnhancedR18));
            else
                % Otherwise check whether the number of ports in the derived port set is equal to the number of layers input
                numDMRSPorts = numel(obj.Ports);
                flag = (obj.NLayers ~= numDMRSPorts);
                if flag
                    [~,validportstxt] = checkPorts(obj,obj.Ports);
                    coder.internal.errorIf(flag,['nr5g:',class(obj),':InvalidDMRSAndNLayers'],...
                        obj.NLayers,validportstxt,double(obj.DMRSConfigurationType),double(obj.DMRSLength),double(obj.DMRSEnhancedR18),numDMRSPorts);
                end
            end
        end

    end

    methods (Access=public, Hidden)

        function validatePTRSPortCompatible(obj,ptrsobj)

            % Possible combinations of auto/custom PT-RS and auto/custom DM-RS
            % and possible imcompatibilities which can arise. This function
            % assumes that the DM-RS configuration is fully valid and self-consistent
            %
            % Auto PT-RS, Auto DM-RS Cases
            % - Auto PT-RS, Auto DM-RS - Len1       - Always good - PT-RS is always 0
            % - Auto PT-RS, Auto DM-RS - Len2       - Always good - PT-RS is always 0
            %
            % Auto PT-RS, Custom DM-RS Cases
            % - Auto PT-RS, Custom DM-RS - Len1     - Always good - DM-RS assumed consistent with Len1 therefore must be good
            % - Auto PT-RS, Custom DM-RS - Len2     - Check CUSTOM DM-RS (the PT-RS used) that the first DM-RS is PT-RS compatible (since Len2, it may not be)
            %
            % Custom PT-RS, Auto DM-RS Cases
            % - Custom PT-RS, Auto DM-RS - Len1     - Check CUSTOM PT-RS is one of the AUTO DM-RS - Since Len1, all DM-RS will be good so just need to check that PT-RS is one of these DM-RS
            % - Custom PT-RS, Auto DM-RS - Len2     - Check CUSTOM PT-RS is one of the _PT-RS compatible_ AUTO DM-RS (there will be some) - Since Len2, DM-RS _may_ contain incompatible ports even though auto (for larger layering), so need to check the custom PT-RS
            %
            % Custom PT-RS, Custom DM-RS Cases
            % - Custom PT-RS, Custom DM-RS - Len1   - Check CUSTOM PT-RS is one of the CUSTOM DM-RS - Since Len1, all DM-RS will be good so just need to check that PT-RS is one of these DM-RS
            % - Custom PT-RS, Custom DM-RS - Len2   - Check CUSTOM DM-RS for some PT-RS compatibility in the first place
            %                                         Check CUSTOM PT-RS is one of the _PT-RS compatible_ CUSTOM DM-RS (there may be none, as above) - Since enL2, custom DM-RS _may_ contain incompatible ports so need to check the custom PT-RS
            
            autodmrs = isempty(obj.DMRSPortSet);
            autoptrs = isempty(ptrsobj.PTRSPortSet);
            
            if obj.DMRSLength == 2
            
                % A valid DM-RS configuration with DMRSLength == 2 may specify 
                % DM-RS port which are not compatible with PT-RS, so first assess 
                % these base DM-RS ports for suitability with the PT-RS
        
                if autoptrs
                   % If auto PT-RS selection then the first DM-RS port is used for
                   % PT-RS so that first port requires checking for PT-RS compatibility
                   testports = obj.Ports(1);
                else
                   % If custom PT-RS selection then all DM-RS ports first need checking 
                   % that at least one of them is PT-RS compatible. If compatible
                   % DM-RS do exist then a later check will establish that the 
                   % any custom PT-RS are selecting from this particular set
                   testports = obj.Ports;
                end  
                
                % For this DM-RS configuration, the set of all PT-RS compatible DM-RS ports is equivalent 
                % to the subset when DMRS length = 1
                dmrsduration = 1;
                [validportset, validportsettxt] = getAllValidPortNumbers(obj,dmrsduration);  % This is the full set of PT-RS compatible ports for this overall DM-RS config
                suitabledmrs = findmembers(testports,validportset);
                % Check that that there are ANY compatible DM-RS; this could only happen if using CUSTOM DM-RS
                if isempty(suitabledmrs)
                    eptxt = num2text(testports);
                    if autoptrs
                        coder.internal.error('nr5g:nrPXSCHConfig:CustomDMRSAreNotAutoPTRSCompatible',eptxt,validportsettxt);
                    else
                        coder.internal.error('nr5g:nrPXSCHConfig:CustomDMRSAreNotCustomPTRSCompatible',eptxt,validportsettxt);
                    end  
                end
            else
                % A valid DM-RS configuration with DMRSLength == 1 will always have PT-RS compatible DM-RS
                suitabledmrs = obj.Ports;
            end

            % A set of valid, PT-RS compatible DM-RS has been identified so lastly we now need to check that any 
            % custom PT-RS are members of this set
            if ~autoptrs
                validptrs = findmembers(ptrsobj.PTRSPortSet,suitabledmrs);
                if isempty(validptrs)
                    errptrstxt = num2text(ptrsobj.PTRSPortSet);
                    errdmrstxt = num2text(suitabledmrs);
                    errfulldmrstxt = num2text(obj.Ports);
                    if autodmrs
                        coder.internal.error('nr5g:nrPXSCHConfig:CustomPTRSNotInAutoDMRS',errptrstxt,errfulldmrstxt,errdmrstxt);
                    else
                        coder.internal.error('nr5g:nrPXSCHConfig:CustomPTRSNotInCustomDMRS',errptrstxt,errfulldmrstxt,errdmrstxt);
                    end  
                end         
            end

        end

    end

    methods (Access=protected)

        function [validports, validportstxt] = getAllValidPortNumbers(obj,duration)
            % Get a list of the valid DM-RS antenna port numbers for this configuration,
            % with optional overriding of the DM-RS symbol duration/length

            % Handle any overriding of the object's DM-RS single/double symbol duration
            if nargin < 2
                dmrsduration = obj.DMRSLengthInternal;
            else
                dmrsduration = duration;
            end

            % TS 38.211 DM-RS antenna ports table, per DM-RS multiplexing, duration and configuration type
            %
            % DM-RS multiplexing     DM-RS duration                    Supported antenna ports p
            %                                              Configuration type 1            Configuration type 2
            % Basic                single-symbol DM-RS         1000 - 1003                     1000 - 1005
            %                      double-symbol DM-RS         1000 - 1007                     1000 - 1011
            % Enhanced             single-symbol DM-RS    1000 - 1003, 1008 - 1011      1000 - 1005, 1012 - 1017
            %                      double-symbol DM-RS         1000 - 1015                     1000 - 1023

            supportedportcombo =    { { { 0:3, 0:5} , {0:7, 0:11} } ,             { { [0:3 8:11], [0:5 12:17]} , {0:15, 0:23} }};
            supportedportcomboTXT = { { {'[0:3]','[0:5]'}, {'[0:7]', '[0:11]'} }, { {'[0:3, 8:11]', '[0:5, 12:17]'} , {'[0:15]','[0:23]'} }};
            validports    = supportedportcombo{ 1+obj.DMRSEnhancedR18 }{ dmrsduration }{ obj.DMRSConfigurationTypeInternal };
            validportstxt = supportedportcomboTXT{ 1+obj.DMRSEnhancedR18 }{ dmrsduration }{ obj.DMRSConfigurationTypeInternal };

        end

        function obj = updateSupportedPortNumbers(obj)
            % Update the cache of support port numbers for this configuration,
            % where dependencies are on,
            %   obj.DMRSConfigurationTypeInternal
            %   obj.DMRSEnhancedR18
            %   obj.DMRSLengthInternal
            obj.AllValidPortNumbers = obj.getAllValidPortNumbers();
        end

        function [invalidportstxt,validportstxt] = checkPorts(obj,testports,duration)
            % Report whether 'testports' are members of the set of valid DM-RS antenna port numbers
            % for this configuration, with optional overriding of the DM-RS symbol duration/length

            % Handle any overriding of the object's DM-RS single/double symbol duration
            if nargin < 3
                dmrsduration = obj.DMRSLength;
            else
                dmrsduration = duration;
            end

            % Get the valid DM-RS port numbers for this configuration
            [validports, validportstxt] = getAllValidPortNumbers(obj,dmrsduration);

            % Test for testports not being part of the valid port set
            [~,notmembers] = findmembers(testports,validports);
            invalidportstxt = num2text(notmembers);
        end

    end

end

% File local functions

function [validmembers,notvalidmembers] = findmembers(testports,validportset)
    % Lightweight port set member testing

    % Create a simple vector 'bitmap' to test for testports which are part
    % and not part of the overall valid port set
    validportmap = zeros(1,24);
    validportmap(validportset+1) = 1;   % Mark the valid set with a 1
    testports = testports(:)';          % Ensure the testports are also a row
    validportmap(testports+1) = validportmap(testports+1) + 2; % Combine in the test ones
    % 3: testports values which are within the valid port set
    % 2: testports values which are outside the valid port set
    % 1: valid ports not containing any testports                     
    % 0: not valid ports not containing any testports
    validmembers = find(validportmap == 3)-1;      % Find the valid test ports (test ports in the valid set)
    notvalidmembers = find(validportmap == 2)-1;   % Find the invalid test ports (test ports not in the valid set)
end

function numtxt = num2text(numvec)
    % Create a comma separated text list of any invalid port numbers (code generation compatible text creation)
    numtxt = '';
    for i = 1:length(numvec)
        numtxt = [numtxt, sprintf('%s,',string(numvec(i)))]; %#ok<AGROW>
    end
    numtxt = numtxt(1:end-1); % Remove final trailing ',' character from the list
end
