classdef Formats0134Common
    %Formats0134Common Common configuration object for PUCCH formats 0, 1, 3, and 4
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   Formats0134Common properties (configurable):
    %
    %   GroupHopping - Group hopping configuration
    %                  ('neither' (default), 'enable', 'disable')
    %   HoppingID    - Hopping identity (0...1023) (default [])

    %   Copyright 2020-2022 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties

        %GroupHopping Group hopping configuration
        %   Specify the group hopping configuration as one of {'neither',
        %   'enable', 'disable'} provided by higher-layer parameter
        %   pucch-GroupHopping. The value 'enable' indicates group hopping
        %   is enabled and sequence hopping is disabled. The value
        %   'disable' indicates sequence hopping is enabled and group
        %   hopping is disabled. The value 'neither' indicates both group
        %   hopping and sequence hopping is disabled. The default value is
        %   'neither'.
        GroupHopping = 'neither';

        %HoppingID Hopping identity
        %   Specify the hopping identity as a scalar nonnegative integer in
        %   range 0...1023. It is the hoppingId (0...1023), if configured,
        %   else, it is the physical layer cell identity (0...1007). Use
        %   empty ([]) to make this property equal to <a href="matlab:
        %   help('nrCarrierConfig/NCellID')">NCellID</a> of nrCarrierConfig.
        %   The default value is [].
        HoppingID = [];

    end

    properties (Constant, Hidden)
        GroupHopping_Values = {'enable', 'disable', 'neither'};
    end

    methods

        % Self-validate and set properties
        function obj = set.GroupHopping(obj,val)
            prop = 'GroupHopping';
            val = validatestring(val,obj.GroupHopping_Values,...
                [class(obj) '.' prop],prop);
            obj.(prop) = '';
            obj.(prop) = val;
        end

        function obj = set.HoppingID(obj,val)
            prop = 'HoppingID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},...
                    {'scalar','integer','nonnegative','<=',1023},...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

    end

end
