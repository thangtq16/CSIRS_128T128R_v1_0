classdef nrNumerologyConfig < comm.internal.ConfigBase
    %nrCarrierConfigBase Common properties between nrCarrierConfig and nrWavegenBWPConfig
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    % Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    properties
        %SubcarrierSpacing Subcarrier spacing in kHz
        %   Specify the subcarrier spacing of the carrier in kHz. The value
        %   must be one of {15, 30, 60, 120, 240, 480, 960}. The default 
        %   value is 15.
        SubcarrierSpacing (1,1) = 15;

        %CyclicPrefix Cyclic prefix
        %   Specify the cyclic prefix as a character vector or a scalar
        %   string. The value must be 'normal' or 'extended'. Note that
        %   extended cyclic prefix only applies to 60 kHz subcarrier
        %   spacing (TS 38.211 Section 4.2). The default value is 'normal'.
        CyclicPrefix = 'normal';
    end

    properties (Constant, Hidden)
         % To allow tab completion with the values when dot indexing the
         % object, and to use in the validation of CyclicPrefix
         CyclicPrefix_Values = {'normal','extended'};
    end
    
    methods
        function obj = nrNumerologyConfig(varargin)
            %nrCarrierConfig Create nrCarrierConfig object
            %   Set the property values from any name-value pairs input to
            %   the object

            % Get the subcarrier spacing value from the name-value pairs
            scs = parseSCS(varargin{:});

            % Call the base class constructor method with the following
            % properties in addition to input name-value pairs:
            % * CyclicPrefix property to allow codegen for all possible
            %   values in a single function script
            % * SubcarrierSpacing property to allow the joint setting of
            %   CyclicPrefix and SubcarrierSpacing properties
            obj@comm.internal.ConfigBase(...
                'CyclicPrefix','normal',...
                'SubcarrierSpacing',scs,...
                varargin{:});
        end

        function obj = set.SubcarrierSpacing(obj,val)
            obj.SubcarrierSpacing = validateSubcarrierSpacing(obj,val);
            checkSCSAndCP(obj,'InvalidSCSForCP'); % Check for dependent cyclic prefix
        end

        function obj = set.CyclicPrefix(obj,val)
            prop = 'CyclicPrefix';
            val = validatestring(val,obj.CyclicPrefix_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = ''; % To allow codegen for varying cyclic prefix (CP) in a single function script
            obj.(prop) = val;
            if ~strcmp(val,'normal') % Check only when CP is extended
                checkSCSAndCP(obj,'InvalidCPForSCS'); % Check for dependent subcarrier spacing
            end
        end
    end

    methods (Access = protected)
        function val = validateSubcarrierSpacing(~,val)
            mustBeMember(val,[15 30 60 120 240 480 960]);
        end
    end

    methods (Access = private)
        function checkSCSAndCP(obj,errorID)
            % Checks for the valid combination of subcarrier spacing (SCS)
            % and cyclic prefix (CP)
            errorFlag = strcmp(obj.CyclicPrefix,'extended') && obj.SubcarrierSpacing ~= 60;
            coder.internal.errorIf(errorFlag,['nr5g:nrCarrierConfig:' errorID],obj.SubcarrierSpacing);
        end
    end
end

function scs = parseSCS(varargin)
% Provides the last subcarrier spacing value SCS from the name-value pairs
% in VARARGIN if SubcarrierSpacing is present, else defaults to 15

    scsPresence = 0;
    for i = nargin-1:-2:1
        if strcmp(varargin{i},'SubcarrierSpacing')
            scs = varargin{i+1};
            scsPresence = 1;
            break
        end
    end
    if ~scsPresence
        scs = 15;
    end
end