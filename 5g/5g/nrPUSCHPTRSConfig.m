classdef nrPUSCHPTRSConfig < nr5g.internal.PTRSConfigBase
    %nrPUSCHPTRSConfig NR PUSCH PT-RS configuration
    %   PTRS = nrPUSCHPTRSConfig creates a phase tracking reference signal
    %   (PT-RS) configuration object for a physical uplink shared channel
    %   (PUSCH), as described in TS 38.211 Section 6.4.1.2. This object
    %   bundles all the properties involved in PUSCH-specific PT-RS symbols
    %   and indices generation. By default, the object defines the PT-RS
    %   with frequency density set to 2 and time density set to 1.
    %
    %   PTRS = nrPUSCHPTRSConfig(Name,Value) creates a PT-RS configuration
    %   object with the specified property Name set to the specified
    %   Value. The additional Name-Value pair arguments can be specified in
    %   any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPUSCHPTRSConfig properties:
    %
    %   TimeDensity      - PT-RS time density (1 (default), 2, 4)
    %   FrequencyDensity - PT-RS frequency density (2 (default), 4). This
    %                      property is used only when transform precoding
    %                      for PUSCH is disabled
    %   NumPTRSSamples   - Number of PT-RS samples (2 (default), 4). This
    %                      property is used only when transform precoding
    %                      for PUSCH is enabled
    %   NumPTRSGroups    - Number of PT-RS groups (2 (default), 4, 8). This
    %                      property is used only when transform precoding
    %                      for PUSCH is enabled
    %   REOffset         - Subcarrier offset ('00' (default), '01', '10', '11').
    %                      This property is used only when transform
    %                      precoding for PUSCH is disabled
    %   PTRSPortSet      - PT-RS antenna port set (default []). This
    %                      property is used only when transform precoding
    %                      for PUSCH is disabled
    %   NID              - PT-RS scrambling identity (0...1007) (default []).
    %                      This property is used only when transform
    %                      precoding for PUSCH is enabled
    %
    %   Example 1:
    %   % Create a default object with all properties
    %
    %   ptrs = nrPUSCHPTRSConfig
    %
    %   Example 2:
    %   % Configure a PUSCH PT-RS object with time density set to 4,
    %   % frequency density set to 4, resource element offset set to '11'.
    %
    %   ptrs = nrPUSCHPTRSConfig('TimeDensity',4,...
    %          'FrequencyDensity',4,'REOffset','11')
    %
    %   Example 3:
    %   % Configure a PUSCH PT-RS object with number of PT-RS groups set to
    %   % 2, number of samples per PT-RS group set to 4, and scrambling
    %   % identity set to 150.
    %
    %   ptrs = nrPUSCHPTRSConfig('NumPTRSGroups',2,...
    %          'NumPTRSSamples',4,'NID',150)
    %
    %   See also nrPUSCHConfig, nrPUSCHDMRSConfig.

    % Copyright 2019-2023 The MathWorks, Inc.

    %#codegen

    properties
        %NumPTRSSamples Number of PT-RS samples (NGroupSamp)
        %   Specify the number of PT-RS samples as a scalar positive
        %   integer. The value must be one of {2, 4}. This property is used
        %   only when transform precoding for PUSCH is enabled. The default
        %   value is 2.
        NumPTRSSamples (1,1) {mustBeMember(NumPTRSSamples, [2 4])} = 2;

        %NumPTRSGroups Number of PT-RS groups (NPTRSGroup)
        %   Specify the number of PT-RS groups as a scalar positive
        %   integer. The value must be one of {2, 4, 8}. Note that when the
        %   number of PT-RS groups is set to 8, the number of PT-RS samples
        %   must be set to 4. This property is used only when transform
        %   precoding for PUSCH is enabled. The default value is 2.
        NumPTRSGroups (1,1) {mustBeMember(NumPTRSGroups, [2 4 8])} = 2;

        %PTRSPortSet PT-RS antenna port set
        %   Specify the PT-RS antenna port set as a scalar or two-element
        %   vector of nonnegative integers. Use empty ([]) to allow this
        %   property to be equal to the lowest DM-RS port number. This
        %   property is used only when transform precoding for PUSCH is
        %   disabled. The default value is [].
        PTRSPortSet = [];

        %NID PT-RS scrambling identity
        %   Specify the PT-RS scrambling identity as a nonnegative scalar
        %   integer. The value must be in range 0...1007. Use empty ([]) to
        %   allow PT-RS scrambling identity to be equal to <a href="matlab:help('nrPUSCHDMRSConfig/NRSID')">NRSID</a> of
        %   nrPUSCHDMRSConfig. This property is used only when transform
        %   precoding for PUSCH is enabled. The default value is [].
        NID = [];
    end

    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'TimeDensity','FrequencyDensity',...
            'NumPTRSSamples','NumPTRSGroups','REOffset','PTRSPortSet','NID'};

        %Mode Transmission mode indicates the control over visibility of
        %the properties based on the waveform type
        % Specify the transmission mode as one of (0, 1, 2). 0 displays the
        % properties specific to CP-OFDM. 1 displays the properties
        % specific to DFT-s-OFDM. 2 displays the list of properties which
        % is the union of both CP-OFDM and DFT-s-OFDM specific properties.
        Mode = 2;
    end

    methods
        function obj = nrPUSCHPTRSConfig(varargin)
            %nrPUSCHPTRSConfig Create a nrPUSCHPTRSConfig object
            %   Set the property values from any name-value pairs input to
            %   the object

            % Get the PTRSPortSet property value from the name-value pairs
            ptrsPorts = nr5g.internal.parseProp('PTRSPortSet',[],varargin{:});
            % Get the NID property value from name-value pairs
            nid = nr5g.internal.parseProp('NID',[],varargin{:});

            % Call the base class constructor method with all the
            % name-value pairs input
            obj@nr5g.internal.PTRSConfigBase(...
                'PTRSPortSet', ptrsPorts, ...
                'NID', nid, ...
                varargin{:});
        end

        function obj = set.PTRSPortSet(obj,val)
            prop = 'PTRSPortSet';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[2 2],[1 1]);
            if ~(isempty(temp) && isnumeric(temp))
                validateattributes(temp, {'numeric'},...
                    {'vector','integer','nonnegative'},...
                    [class(obj) '.' prop],prop);
            end
            valLen = numel(temp);
            coder.internal.errorIf(valLen>2, ...
                'nr5g:nrPUSCHPTRSConfig:InvalidNumPTRSPorts',valLen);
            obj.(prop) = temp;
        end

        function obj = set.NID(obj,val)
            prop = 'NID';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isempty(temp) && isnumeric(temp))
                validateattributes(temp,{'numeric'},...
                    {'scalar','integer','nonnegative','<=',1007},...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end

        function validateConfig(obj)
            % Check for the combination of number of PT-RS groups and
            % number of samples per PT-RS group, when transform precoding
            % is enabled
            coder.internal.errorIf((obj.Mode == 1) && (obj.NumPTRSGroups == 8) && (obj.NumPTRSSamples == 2), ...
                'nr5g:nrPUSCHPTRSConfig:InvalidNumSamplesPerPTRSGroup');
        end
    end

    methods(Access = protected)

        function flag = isInactiveProperty(obj, prop)
            % Return false if property is visible based on object
            % configuration, for the command line
            flag = false;

            %FrequencyDensity - only required when transform precoding for
            %PUSCH is disabled
            if strcmp(prop,'FrequencyDensity')
                flag = (obj.Mode == 1);
            end

            %NumPTRSSamples - only required when transform precoding for
            %PUSCH is enabled
            if strcmp(prop,'NumPTRSSamples')
                flag = (obj.Mode == 0);
            end

            %NumPTRSGroups - only required when transform precoding for
            %PUSCH is enabled
            if strcmp(prop,'NumPTRSGroups')
                flag = (obj.Mode == 0);
            end

            %REOffset - only required when transform precoding for PUSCH is
            %disabled
            if strcmp(prop,'REOffset')
                flag = (obj.Mode == 1);
            end

            %PTRSPortSet - only required when transform precoding for PUSCH
            %is disabled
            if strcmp(prop,'PTRSPortSet')
                flag = (obj.Mode == 1);
            end

            %NID - only required when transform precoding for PUSCH is
            %enabled
            if strcmp(prop,'NID')
                flag = (obj.Mode == 0);
            end
        end
    end

end
