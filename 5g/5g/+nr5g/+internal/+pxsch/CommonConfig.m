classdef (Abstract) CommonConfig < comm.internal.ConfigBase
    %CommonConfig Common configuration object for physical shared channel
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   CommonConfig properties (configurable):
    %
    %   NumLayers        - Number of transmission layers (1...8) (default 1)
    %   SymbolAllocation - OFDM symbols allocated for physical shared
    %                      channel within a slot (default [0 14])
    %   PRBSet           - PRBs allocated for physical shared channel
    %                      within the BWP (default 0:51)
    %   RNTI             - Radio network temporary identifier (0...65535) (default 1)
    %   EnablePTRS       - Enable or disable the PT-RS configuration (0 (default), 1)

    %   Copyright 2019-2024 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %NumLayers Number of transmission layers
        %   Specify the number of transmission layers as a scalar positive
        %   integer. The value must be in the range 1...8. The default
        %   value is 1.
        NumLayers (1,1) {mustBeNumeric, mustBeInteger, mustBeInRange(NumLayers,1,8)} = 1;

        %MappingType Mapping type of physical shared channel
        %   Specify the mapping type of physical shared channel as a
        %   character or string scalar. The value must be one of
        %   {'A', 'B'}. The default value is 'A'.
        MappingType = 'A';

        %SymbolAllocation Physical shared channel symbol allocation within a slot
        %   Specify the physical shared channel symbol allocation within a
        %   slot as a two-element vector of nonnegative integers
        %   representing the starting symbol and symbol length. The default
        %   value is [0 14].
        SymbolAllocation = [0 14];

        %PRBSet PRBs allocated for physical shared channel within the BWP
        %   Specify the PRBs (0-based) allocated for physical shared
        %   channel within the BWP as a vector of nonnegative integers in
        %   the range 0...274. The default value is 0:51.
        PRBSet = 0:51;

        %RNTI Radio network temporary identifier
        %   Specify the radio network temporary identifier as scalar
        %   nonnegative integer. The value must be in the range 0...65535.
        %   The default value is 1.
        RNTI (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(RNTI,65535)} = 1;

        %EnablePTRS Enable or disable the PT-RS configuration
        %   Specify the flag to enable or disable PT-RS as a numeric or
        %   logical scalar. The value must be binary. 0 indicates that
        %   PT-RS is disabled. 1 indicates that PT-RS is enabled. The
        %   default value is 0.
        EnablePTRS (1,1) logical = false;
    end

    % Read-only properties
    properties (SetAccess = private)
        %NumCodewords Number of codewords
        %   Number of codewords. This property is read-only and is updated
        %   based on the property DMRSPortSet of channel-specific DM-RS
        %   configuration object. If DMRSPortSet is empty, NumLayers is
        %   used to calculate the number of codewords.
        NumCodewords;
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        MappingType_Values = {'A','B'};
    end

    properties (Abstract,Constant,Hidden)
        Modulation_Values
    end

    methods

        % Constructor
        function obj = CommonConfig(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow var-size in codegen
                obj.SymbolAllocation = nr5g.internal.parseProp('SymbolAllocation',[0 14],varargin{:});
                obj.PRBSet = nr5g.internal.parseProp('PRBSet',0:51,varargin{:});
            end
        end

        function obj = set.NumLayers(obj,val)
            obj.NumLayers = val; % val is already validated
            obj = updateHiddenProps(obj);
        end

        function obj = set.MappingType(obj,val)
            prop = 'MappingType';
            val = validatestring(val,obj.MappingType_Values,[class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

        function obj = set.SymbolAllocation(obj,val)
            prop = 'SymbolAllocation';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[2 2],[1 1]);
            if ~isempty(temp)
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative','numel',2},[class(obj) '.' prop],prop);
                coder.internal.errorIf((temp(1) + temp(2)) > 14,'nr5g:nrPXSCH:InvalidSymbolAllocation',temp(1),temp(2),14);
            end
            obj.(prop) = temp;
        end

        function obj = set.PRBSet(obj,val)
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validatePRBSet(obj,temp);
            end
            obj.PRBSet = temp;
        end

		% Read-only properties
        function out = get.NumCodewords(obj)
            if isempty(obj.DMRS.DMRSPortSet)
                nlayers = double(obj.NumLayers);
            else
                nlayers = numel(obj.DMRS.DMRSPortSet);
            end
            out = ceil(nlayers/4);
        end

    end

    % Validate methods common to PDSCH and PUSCH for those properties that
    % are explicitly defined in each class
    methods(Access = protected)
		function val = validatePRBSet(obj,val)
            prop = 'PRBSet';
            validateattributes(val,{'numeric'},{'vector','integer','nonnegative','<=',274},[class(obj) '.' prop],prop);
        end
        function nid = validateNID(obj,val)
            prop = 'NID';
            % To allow codegen for varying length in a single function script
            nid = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(nid) && isempty(nid))
                validateattributes(nid,{'numeric'},{'vector','integer','nonnegative','<=',1023},[class(obj) '.' prop],prop);
            end
        end

        function modulation = validateModulation(obj,val)
            prop = 'Modulation';
            modulationValues = getModulationValues(obj);
            validateattributes(val,{'cell','char','string'},...
                {'nonempty'},[class(obj) '.' prop], prop);
            if ischar(val) || (isstring(val) && isscalar(val))
                % Character array or string scalar
                % Single modulation scheme
                modulation = validatestring(val,modulationValues,[class(obj) '.' prop],prop);
            else
                % Cell array or string array
                % One or two modulation schemes
                nValues = numel(val);
                coder.internal.errorIf(~any(nValues == [1 2]),'nr5g:nrPXSCHConfig:InvalidNumModulation',nValues);
                % Convert string array to cell array
                tempVal = convertStringsToChars(val);
                modulation = coder.nullcopy(cell(1,nValues));
                for idx = 1:numel(tempVal)
                    modulation{idx} = validatestring(tempVal{idx},modulationValues,[class(obj) '.' prop],prop);
                end
            end
        end

        function v = getModulationValues(obj)

            v = obj.Modulation_Values;

        end

    end

    methods (Access = private)
        function obj = updateHiddenProps(obj)
            %updateHiddenProps Updates the NumPorts hidden property with
            %   the value of NumLayers and then, updates the value of
            %   NLayers hidden property of DM-RS with the value of
            %   NumPorts, if required. This is used to provide appropriate
            %   values for the read-only properties, when the DM-RS port
            %   set is provided empty.
            obj.NumPorts = double(obj.NumLayers);
            obj.DMRS.NLayers = obj.NumPorts;
        end
    end

end
