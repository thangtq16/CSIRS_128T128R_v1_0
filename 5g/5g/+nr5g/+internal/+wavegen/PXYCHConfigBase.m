classdef PXYCHConfigBase < nr5g.internal.wavegen.MIMOPrecodingConfig
%ConfigBase Class offering properties common between all physical channel wavegen configuration objects
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

    % Public, tunable properties
    properties
        %Label Custom alphanumeric label
        % Specify Label as a character array or string scalar. Use this
        % property to assign a description to this object.
        Label = '';
        
        %BandwidthPartID ID of bandwidth part containing this object
        % Specify BandwidthPartID as a nonnegative integer scalar to link
        % this channel instance with one of the bandwidth parts defined in
        % the BandwidthParts property of <a href="matlab:help('nrDLCarrierConfig.BandwidthParts')"
        % >nrDLCarrierConfig</a> or <a href="matlab:help('nrULCarrierConfig.BandwidthParts')"
        % >nrULCarrierConfig</a>. The default is 1.
        BandwidthPartID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative} = 1;
        
        %SlotAllocation Allocation of slots within a period
        % Specify SlotAllocation as a scalar or row vector of nonnegative
        % integers to specify the slot position of this channel. Indexing is
        % 0-based. The values in SlotAllocation must be smaller than
        % the value of the Period property, otherwise they are ignored.
        SlotAllocation = 0:9;
        
        %Period Allocation period in slots
        % Specify the period of this channel in slots as a positive integer
        % scalar or []. An empty value indicates no repetition (single slot
        % only).
        Period = 10;
    end
    
    methods
        % Self-validate and set properties
        function obj = set.Label(obj,val)
            prop = 'Label';
            validateattributes(val, {'char', 'string'}, {'scalartext'}, ...
                [class(obj) '.' prop], prop);
            obj.(prop) = ''; % For codegen compatibility
            obj.(prop) = convertStringsToChars(val);
        end
        
        function obj = set.SlotAllocation(obj,val)
            prop = 'SlotAllocation';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
        
        function obj = set.Period(obj,val)
            prop = 'Period';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'scalar','integer','positive'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
    end
end
