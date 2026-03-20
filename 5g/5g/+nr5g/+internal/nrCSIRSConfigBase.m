classdef (Abstract) nrCSIRSConfigBase < comm.internal.ConfigBase
%nrCSIRSConfigBase Class offering properties common between nrCSIRSConfig and nrWavegenCSIRSConfig
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2020-2024 The MathWorks, Inc.

%#codegen


    % Public, tunable properties
    properties
        %CSIRSType CSI-RS type
        %   Specify the CSI-RS type of one or more CSI-RS resource
        %   configurations in terms of zero-power or non-zero-power using
        %   the options 'zp' and 'nzp' respectively. In case of single
        %   CSI-RS resource, this property is specified as a character
        %   array or string scalar. In case of multiple CSI-RS resources,
        %   this property is specified as a cell array or a string array of
        %   length equals to the number of CSI-RS resources to be
        %   configured. Note that this property decides the number of
        %   CSI-RS resources configured based on the number of values
        %   provided. The default is 'nzp'.
        CSIRSType = 'nzp';

        %CSIRSPeriod Slot periodicity and offset of CSI-RS resource(s)
        %   Specify the slot periodicity and offset values of one or more
        %   CSI-RS resource configurations. The possible options are
        %   {'on'(default),'off',[Tcsi-rs Toffset]}. When this property is
        %   set to 'on' for a CSI-RS resource, then the resource is present
        %   in all slots. When this property is set to 'off' for a CSI-RS
        %   resource, then the resource is absent in all slots. When this
        %   property is set to [Tcsi-rs Toffset], then explicit slot
        %   periodicity (Tcsi-rs) and offset (Toffset) values are
        %   considered to check the presence of the resource in the
        %   specified slot. The possible values of Tcsi-rs are
        %   [4,5,8,10,16,20,32,40,64,80,160,320,640]. For a particular
        %   Tcsi-rs, the value of Toffset is in the range 0...Tcsi-rs-1. It
        %   is provided by the higher-layer parameter
        %   CSI-ResourcePeriodicityAndOffset or slotConfig in the
        %   CSI-RS-CellMobility IE. Multiple CSI-RS resources can be
        %   configured with a single CSI-RS period, or from a cell array or
        %   a string array of CSI-RS period values where each value
        %   corresponds to a separate resource. The default is 'on'.
        CSIRSPeriod = 'on';

        %RowNumber Row number(s) from CSI-RS locations table
        %   Specify the row number as a scalar or vector of positive
        %   integers in the range 1 to 18 for one or more CSI-RS resource
        %   configurations as defined in TS 38.211 Table 7.4.1.5.3-1. The
        %   length of provided input vector must be equal to the number of
        %   CSI-RS resources configured. The default is 3.
        RowNumber = 3;

        %Density Frequency density of CSI-RS resource(s)
        %   Specify the frequency density of one or more CSI-RS resource
        %   configurations. The possible options are
        %   {'one','three','dot5even','dot5odd'}. In comparison to TS
        %   38.211 Table 7.4.1.5.3-1, the option 'one' corresponds to rho
        %   as 1, 'three' corresponds to rho as 3, 'dot5even' corresponds
        %   to rho as 0.5 with even RB allocation with respect to common
        %   resource block 0 (CRB 0) and 'dot5odd' corresponds to rho as
        %   0.5 with odd RB allocation with respect to CRB 0. It is
        %   provided by the higher-layer parameter density in the
        %   CSI-RS-ResourceMapping IE or the CSI-RS-CellMobility IE. In
        %   case of single CSI-RS resource, this property is specified as a
        %   character array or string scalar. In case of multiple CSI-RS
        %   resources, this property is specified as a cell array or a
        %   string array of length equals to the number of CSI-RS resources
        %   configured. The default is 'one'.
        Density = 'one';

        %SymbolLocations OFDM symbol locations of CSI-RS resource(s)
        %   Specify the OFDM symbol locations of one or more CSI-RS
        %   resource configurations within a slot. It can be either a
        %   scalar (l_0) or a vector containing 2 elements ([l_0,l_1]),
        %   where l_0 and l_1 are provided by the higher-layer parameters
        %   firstOFDMSymbolInTimeDomain (0...13) and
        %   firstOFDMSymbolInTimeDomain2 (2...12) respectively in the
        %   CSI-RS-ResourceMapping IE or CSI-RS-Resource-Mobility IE. In
        %   case of single CSI-RS resource, this property is specified as a
        %   vector. In case of multiple CSI-RS resources, this property is
        %   specified as a cell array of length equals to the number of
        %   CSI-RS resources configured. Note that for a CSI-RS resource,
        %   l_1 value is required only for the rows 13,14,16 and 17 in
        %   TS 38.211 Table 7.4.1.5.3-1. The default is 0.
        SymbolLocations = 0;

        %SubcarrierLocations Subcarrier locations of CSI-RS resource(s)
        %   Specify the subcarrier locations (k_i values in TS 38.211
        %   Table 7.4.1.5.3-1) of one or more CSI-RS resource
        %   configurations within an RB. For a CSI-RS resource, possible
        %   lengths of subcarrier locations are {1,2,3,4,6}. In case of
        %   single CSI-RS resource, this property is specified as a vector.
        %   In case of multiple CSI-RS resources, this property is
        %   specified as a cell array of length equals to the
        %   number of CSI-RS resources configured. The default is 0.
        SubcarrierLocations = 0;

        %NumRB Number of RBs allocated for CSI-RS resource(s)
        %   Specify the number of RBs across which CSI-RS resource spans
        %   for one or more CSI-RS resource configurations. It can be
        %   either a scalar or vector of positive integers in the range
        %   1...275. It is provided by the higher-layer parameter nrofRBs
        %   in CSI-FrequencyOccupation IE or nrofPRBs in
        %   CSI-RS-CellMobility IE. Nominal values of this property are
        %   multiples of 4 in the range 24...275. The length of provided
        %   input vector must be either 1 or the number of CSI-RS resources
        %   configured. In case of scalar input, the provided value is
        %   considered for all CSI-RS resources. The default is 52.
        NumRB = 52;

        %RBOffset Starting RB index of CSI-RS resource(s) allocation
        %   Specify the RB index where CSI-RS resource starts relative to
        %   carrier resource grid for one or more CSI-RS resource
        %   configurations. It can be either a scalar or vector of
        %   non-negative integers in the range 0...274. The length of
        %   provided input vector must be either 1 or the number of CSI-RS
        %   resources configured. In case of scalar input, the provided
        %   value is considered for all CSI-RS resources. The default is 0.
        RBOffset = 0;

        %NID Scrambling identity of CSI-RS resource(s)
        %   Specify the scrambling identities of one or more CSI-RS
        %   resource configurations. Scrambling identity corresponding to a
        %   CSI-RS resource must be a non-negative integer in the range
        %   0...1023. It is provided by the higher-layer parameter
        %   scramblingID in NZP-CSI-RS-Resource IE or
        %   sequenceGenerationConfig in CSI-RS-ResourceConfigMobility IE.
        %   Number of scrambling identities specified must be either 1 or
        %   the number of CSI-RS resources configured. In case of scalar
        %   input, the provided value is considered for all CSI-RS
        %   resources. When a mix of ZP and NZP CSI-RS resources are
        %   configured, scrambling identity corresponding to a ZP-CSI-RS
        %   resource is ignored as it is not required. Note that this
        %   property is not displayed when all the resources configured are
        %   ZP-CSI-RS. The default is 0.
        NID = 0;

    end

    % Read-only properties
    properties (Dependent = true, SetAccess = private)
        %NumCSIRSPorts Number of CSI-RS specific antenna ports
        %   The number of antenna ports specific to one or more CSI-RS
        %   resource configuration(s). This property is read-only and is
        %   updated based on the property RowNumber.
        NumCSIRSPorts;
    end
  
    % Constant, hidden properties
    properties (Constant,Hidden)

        CSIRSType_Values        = {'zp','nzp'};
        CSIRSPeriod_CharOptions =  {'on','off'};
        SlotPeriodicity_Options = [4,5,8,10,16,20,32,40,64,80,160,320,640];
        % The following range of values are considered as specified in
        % TS 38.211 Table 7.4.1.5.3-1
        Density_Values          = {'one','three','dot5even','dot5odd'};
        SCLocations_Lengths     = [1 2 3 4 6];
        SymbolLocations_Lengths = [1 2];
        Ports_Options           = [1 1 2 4 4 8 8 8 12 12 16 16 24 24 24 32 32 32];
    end

    methods

        % Constructor
        function obj = nrCSIRSConfigBase(varargin)
            % Support name-value pair arguments when constructing object
            obj@comm.internal.ConfigBase(varargin{:});
        end

        % Self-validate and set properties
        function obj = set.CSIRSType(obj,val)
            prop = 'CSIRSType';
            validateattributes(val,{'cell','char','string'},...
                {'nonempty'},[class(obj) '.' prop], prop);

            if ischar(val) || (isstring(val) && isscalar(val))
                % Character array or string scalar
                % Single CSI-RS resource
                temp = validatestring(val,obj.CSIRSType_Values,[class(obj) '.' prop],prop);
                obj.(prop) = '';
            else
                % Cell array or string array
                % Multiple CSI-RS resources

                % Convert string array to cell array
                tempVal = convertStringsToChars(val);
                temp = coder.nullcopy(cell(1,numel(tempVal)));
                for idx = 1:numel(tempVal)
                    temp{idx} = validatestring(tempVal{idx},obj.CSIRSType_Values,[class(obj) '.' prop],prop);
                end
            end
            obj.(prop) = temp;
        end

        function obj = set.CSIRSPeriod(obj,val)
            prop = 'CSIRSPeriod';
            validateattributes(val,{'numeric','cell','char','string'},...
                {'nonempty'},[class(obj) '.' prop],prop);

            if ischar(val) || (isstring(val) && isscalar(val))
                % Character array or string scalar
                % Single CSI-RS resource
                temp = validatestring(val,obj.CSIRSPeriod_CharOptions,[class(obj) '.' prop],prop);
                obj.(prop) = '';
            elseif isnumeric(val)
                % Vector
                % Single CSI-RS resource
                validateattributes(val,{'numeric'},...
                    {'vector','integer','nonnegative','numel',2},[class(obj) '.' prop],prop);
                if ~any(val(1) == obj.SlotPeriodicity_Options)
                    coder.internal.error('nr5g:nrCSIRS:InvalidSlotPeriodicity',val(1));
                end
                if val(2) >= val(1)
                    coder.internal.error('nr5g:nrCSIRS:InvalidSlotOffset',val(2),val(1));
                end
                temp = val;
            elseif isstring(val) && numel(val) > 1
                % String array
                % Multiple CSI-RS resources
                temp = coder.nullcopy(cell(1,numel(val)));
                for idx = 1:numel(val)
                    temp{idx} = validatestring(val{idx},obj.CSIRSPeriod_CharOptions,[class(obj) '.' prop],prop);
                end
            else
                % Cell array
                % Multiple CSI-RS resources
                temp = coder.nullcopy(cell(1,numel(val)));
                for idx = 1:numel(val)
                    validateattributes(val{idx},...
                        {'numeric','char','string'},{},[class(obj) '.' prop],prop);
                    if isnumeric(val{idx})
                        validateattributes(val{idx},{'numeric'},...
                            {'vector','integer','nonnegative','numel',2},[class(obj) '.' prop],prop);
                        if ~any(val{idx}(1) == obj.SlotPeriodicity_Options)
                            coder.internal.error('nr5g:nrCSIRS:InvalidSlotPeriodicity',val{idx}(1));
                        end
                        if val{idx}(2) >= val{idx}(1)
                            coder.internal.error('nr5g:nrCSIRS:InvalidSlotOffset',val{idx}(2),val{idx}(1));
                        end
                        temp{idx} = val{idx};
                    else
                        temp{idx} = validatestring(val{idx},obj.CSIRSPeriod_CharOptions,[class(obj) '.' prop],prop);
                    end
                end
            end
            obj.(prop) = temp;
        end

        function obj = set.RowNumber(obj,val)
            prop = 'RowNumber';
            validateattributes(val,{'numeric'},...
                {'vector','integer','positive','<=',18},[class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

        function obj = set.Density(obj,val)
            prop = 'Density';
            validateattributes(val,{'cell','char','string'},...
                {'nonempty'},[class(obj) '.' prop],prop);
            if ischar(val) || (isstring(val) && isscalar(val))
                % Character array or string scalar
                % Single CSI-RS resource
                temp = validatestring(val,obj.Density_Values,[class(obj) '.' prop],prop);
                obj.(prop) = '';
            else
                % Cell array or string array
                % Multiple CSI-RS resources

                % Convert string array to cell array
                tempVal = convertStringsToChars(val);
                temp = coder.nullcopy(cell(1,numel(tempVal)));
                for idx = 1:numel(tempVal)
                    temp{idx} = validatestring(tempVal{idx},obj.Density_Values,[class(obj) '.' prop],prop);
                end
            end
            obj.(prop) = temp;
        end

        function obj = set.SymbolLocations(obj,val)
            prop = 'SymbolLocations';
            validateattributes(val,{'numeric','cell'},{'nonempty'},...
                [class(obj) '.' prop],prop);
            if iscell(val)
                inpClass = class(val{1});
            else
                inpClass = class(val);
            end
            if isnumeric(val)
                % Vector
                % Single CSI-RS resource
                validateattributes(val,{'numeric'},...
                    {'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
                if ~any(numel(val) == obj.SymbolLocations_Lengths)
                    coder.internal.error('nr5g:nrCSIRS:InvalidSymbolLocationsLen',numel(val));
                end
                obj.(prop) = zeros(0,0,inpClass);
            else
                % Cell array
                % Multiple CSI-RS resources
                for idx = 1:numel(val)
                    validateattributes(val{idx},{'numeric'},...
                        {'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
                    if ~any(numel(val{idx}) == obj.SymbolLocations_Lengths)
                        coder.internal.error('nr5g:nrCSIRS:InvalidSymbolLocationsLen',numel(val{idx}));
                    end
                end
                obj.(prop) = {zeros(0,0,inpClass)};
            end
            obj.(prop) = val;
        end

        function obj = set.SubcarrierLocations(obj,val)
            prop = 'SubcarrierLocations';
            validateattributes(val,{'numeric','cell'},{'nonempty'},...
                [class(obj) '.' prop],prop);
            if iscell(val)
                inpClass = class(val{1});
            else
                inpClass = class(val);
            end
            if isnumeric(val)
                % Vector
                % Single CSI-RS resource
                validateattributes(val,{'numeric'},...
                    {'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
                if ~any(numel(val) == obj.SCLocations_Lengths)
                    coder.internal.error('nr5g:nrCSIRS:InvalidSubcarrierLocationsLen',numel(val));
                end
                obj.(prop) = zeros(0,0,inpClass);
            else
                % Cell array
                % Multiple CSI-RS resources
                for idx = 1:numel(val)
                    validateattributes(val{idx},{'numeric'},...
                        {'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
                    if ~any(numel(val{idx}) == obj.SCLocations_Lengths)
                        coder.internal.error('nr5g:nrCSIRS:InvalidSubcarrierLocationsLen',numel(val{idx}));
                    end
                end
                obj.(prop) = {zeros(0,0,inpClass)};
            end
            obj.(prop) = val;
        end

        function obj = set.NumRB(obj,val)
            prop = 'NumRB';
            validateattributes(val,{'numeric'},...
                {'vector','integer','positive','<=',275},...
                [class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

        function obj = set.RBOffset(obj,val)
            prop = 'RBOffset';
            validateattributes(val,{'numeric'},...
                {'vector','integer','nonnegative','<=',274},...
                [class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

        function obj = set.NID(obj,val)
            prop = 'NID';
            validateattributes(val,{'numeric'},...
                {'vector','integer','nonnegative','<=',1023},...
                [class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

        % Read-only properties
        function out = get.NumCSIRSPorts(obj)
            % NumCSIRSPorts is updated based on RowNumber
            out = zeros(1,numel(obj.RowNumber));
            for rowIdx = 1:numel(obj.RowNumber)
                out(rowIdx) = obj.Ports_Options(obj.RowNumber(rowIdx));
            end
        end
    
        function out = validateConfig(obj)
            %validateConfig Validate the nrCSIRSConfig object
            %   OUT = validateConfig(OBJ) validates the inter dependent
            %   properties of specified nrCSIRSConfig configuration object
            %   and returns one structure OUT with the updated CSI-RS
            %   parameters.

            % CSIRSType
            tempCSIRSType = obj.convertToCell(obj.CSIRSType);
            numCSIRSRes = numel(tempCSIRSType);

            % CSIRSPeriod
            tempPeriod = obj.convertToCell(obj.CSIRSPeriod);
            if ~((numel(tempPeriod) == numCSIRSRes) || (isscalar(tempPeriod)))
                coder.internal.error('nr5g:nrCSIRS:InvalidNumCSIRSPeriodValues',numel(obj.CSIRSPeriod),numCSIRSRes);
            end
            if isscalar(tempPeriod)
                tempCSIRSPeriod = repmat(tempPeriod,1,numCSIRSRes);
            else
                tempCSIRSPeriod = tempPeriod;
            end

            % RowNumber
            coder.internal.errorIf((numel(obj.RowNumber) ~= numCSIRSRes),...
                'nr5g:nrCSIRS:InvalidRowNumberLen',numel(obj.RowNumber),numCSIRSRes);
            tempRowNumber = double(obj.RowNumber);

            % Density
            tempDensity = obj.convertToCell(obj.Density);
            coder.internal.errorIf((numel(tempDensity) ~= numCSIRSRes),...
                'nr5g:nrCSIRS:InvalidNumDensityValues',numel(tempDensity),numCSIRSRes);

            % SymbolLocations
            tempSymLocations = obj.convertToCell(obj.SymbolLocations);
            coder.internal.errorIf((numel(tempSymLocations) ~= numCSIRSRes),...
                'nr5g:nrCSIRS:InvalidNumSymbolLocationsValues',numel(tempSymLocations),numCSIRSRes);

            % SubcarrierLocations
            tempSCLocations = obj.convertToCell(obj.SubcarrierLocations);
            coder.internal.errorIf((numel(tempSCLocations) ~= numCSIRSRes),...
                'nr5g:nrCSIRS:InvalidNumSubcarrierLocationsValues',numel(tempSCLocations),numCSIRSRes);

            % NID
            if all(strcmpi(tempCSIRSType,'zp'))
                tempNID = double(obj.NID);
            else
                if ~(numel(obj.NID) == numCSIRSRes || (isscalar(obj.NID)))
                    coder.internal.error('nr5g:nrCSIRS:InvalidNIDLen',numel(obj.NID),numCSIRSRes);
                end
                tempNID = obj.applyScalarExpansion(obj.NID,numCSIRSRes);
            end

            % NumRB
            if ~((numel(obj.NumRB) == numCSIRSRes) ||...
                    (isscalar(obj.NumRB)))
                coder.internal.error('nr5g:nrCSIRS:InvalidNumRBLen',numel(obj.NumRB),numCSIRSRes);
            end
            tempNumRB = obj.applyScalarExpansion(obj.NumRB,numCSIRSRes);

            % RBOffset
            if ~((numel(obj.RBOffset) == numCSIRSRes) ||...
                    (isscalar(obj.RBOffset)))
                coder.internal.error('nr5g:nrCSIRS:InvalidRBOffsetLen',numel(obj.RBOffset),numCSIRSRes);
            end
            tempRBOffset = obj.applyScalarExpansion(obj.RBOffset,numCSIRSRes);

            % Validate density and subcarrier locations of all CSI-RS
            % resources
            for resIdx = 1:numCSIRSRes
                obj.validateDensity(tempRowNumber(resIdx),tempDensity{resIdx});
                obj.validateKiValues(tempRowNumber(resIdx),tempSCLocations{resIdx});
            end

            % Reassign the updated properties to the out structure
            out.CSIRSType = tempCSIRSType;
            out.CSIRSPeriod = tempCSIRSPeriod;
            out.RowNumber = tempRowNumber;
            out.Density = tempDensity;
            out.SymbolLocations = tempSymLocations;
            out.SubcarrierLocations = tempSCLocations;
            out.NID = tempNID;
            out.NumRB = tempNumRB;
            out.RBOffset = tempRBOffset;

        end

    end

    methods(Access = private)

        function out = convertToCell(~,prop)
            %convertToCell Converts the input property type to cell
            %   OUT = convertToCell(~,PROP) returns the cell output OUT by
            %   converting the input property PROP to cell.
            if iscell(prop)
                out = prop;
            else
                out = {prop};
            end
        end

        function out = applyScalarExpansion(~,prop,n)
            %applyScalarExpansion Applies scalar expansion
            %   OUT = applyScalarExpansion(~,PROP,N) returns scalar
            %   expanded output OUT by repeating the input property PROP
            %   for N times.
            if isscalar(prop)
                temp = repmat(prop,1,n);
            else
                temp = prop;
            end
            out = double(temp);
        end

        function validateDensity(~,rowIndex,density)
            %validateDensity Validate the density property
            %   validateDensity(~,ROWINDEX,DENSITY) validates the density
            %   property of a CSI-RS resource corresponding to the row
            %   index rowIndex.

            if (rowIndex == 1) && (~strcmpi(density,'three'))
                coder.internal.error('nr5g:nrCSIRS:InvalidDensityRow1',rowIndex,density,'''three''');
            elseif any(rowIndex == 4:10) && (~strcmpi(density,'one'))
                coder.internal.error('nr5g:nrCSIRS:InvalidDensityRows4to10',rowIndex,density,'''one''');
            elseif any(rowIndex == [2 3 11:18]) && strcmpi(density,'three')
                coder.internal.error('nr5g:nrCSIRS:InvalidDensityRows2_3_11to18',rowIndex,density,'{''one'',''dot5even'',''dot5odd''}');
            end

        end

        function validateKiValues(~,rowIndex,KiValues)
            %validateKiValues Validate the subcarrier locations
            %   validateKiValues(~,ROWINDEX,KIVALUES) validates the
            %   subcarrier locations of a CSI-RS resource corresponding to
            %   the row index rowIndex.

            % Consider the lengths of k_i values for all rows based on
            % TS 38.211 Table 7.4.1.5.3-1
            KiLengths = [1 1 1 1 1 4 2 2 6 3 4 4 3 3 3 4 4 4];
            % Validate the number of k_i values based on the row index
            coder.internal.errorIf(numel(KiValues) ~= KiLengths(rowIndex),...
                'nr5g:nrCSIRS:InvalidKiValuesLen',rowIndex,numel(KiValues),KiLengths(rowIndex));

            % Consider the upper bounds of k_i values for all rows based on
            % bitmaps provided in TS 38.211 Section 7.4.1.5.3
            KiUpperBounds = [3 11 10 8 10 10 10 10 10 10 10 10 10 10 10 10 10 10];
            % Validate the k_i values based on the row index
            coder.internal.errorIf(~all(KiValues(:) <= KiUpperBounds(rowIndex)),...
                'nr5g:nrCSIRS:InvalidKi',rowIndex,KiUpperBounds(rowIndex));

        end

    end

    methods(Access = protected)

        function flag = isInactiveProperty(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line
            flag = false;

            % Only required for NZP-CSI-RS
            if strcmp(prop,'NID')
                flag = ~(any(strcmpi(obj.CSIRSType,'nzp')));
            end

        end

    end
end
