classdef nrPDSCHPTRSConfig < nr5g.internal.PTRSConfigBase
    %nrPDSCHPTRSConfig NR PDSCH PT-RS configuration
    %   PTRS = nrPDSCHPTRSConfig creates a phase tracking reference signal
    %   (PT-RS) configuration object for a physical downlink shared
    %   channel, as described in TS 38.211 Section 7.4.1.2. This object
    %   bundles all the properties involved in PDSCH-specific PT-RS symbols
    %   and indices generation. By default, the object defines the PT-RS
    %   with frequency density set to 2 and time density set to 1.
    %
    %   PTRS = nrPDSCHPTRSConfig(Name,Value) creates a PT-RS configuration
    %   object with the specified property Name set to the specified
    %   Value. The additional Name-Value pair arguments can be specified in
    %   any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPDSCHPTRSConfig properties:
    %
    %   TimeDensity      - PT-RS time density (1 (default), 2, 4)
    %   FrequencyDensity - PT-RS frequency density (2 (default), 4)
    %   REOffset         - Resource element offset
    %                      ('00' (default), '01', '10', '11')
    %   PTRSPortSet      - PT-RS antenna port set (default [])
    %
    %   Example:
    %   % Configure a PDSCH PT-RS object with time density set to 4,
    %   % frequency density set to 4, resource element offset set to '11'.
    %
    %   ptrs = nrPDSCHPTRSConfig('TimeDensity',4,...
    %          'FrequencyDensity',4,'REOffset','11')
    %
    %   See also nrPDSCHConfig, nrPDSCHDMRSConfig.

    % Copyright 2019 The MathWorks, Inc.

    %#codegen

    properties
        %PTRSPortSet Antenna port set
        %   Specify the PT-RS antenna port set as a scalar nonnegative
        %   integer. Use empty ([]) to allow this property to be equal to
        %   the lowest DM-RS port number. The default value is [].
        PTRSPortSet = [];
    end

    properties (Hidden)
        CustomPropList = {'TimeDensity','FrequencyDensity',...
            'REOffset','PTRSPortSet'};
    end

    methods
        function obj = nrPDSCHPTRSConfig(varargin)
            %nrPDSCHPTRSConfig Create a nrPDSCHPTRSConfig object
            %   Set the property values from any name-value pairs input to
            %   the object

            % Get the value of PTRSPortSet from the name-value pairs
            ptrsPort = nr5g.internal.parseProp('PTRSPortSet',[],varargin{:});

            % Call the base class constructor method with all the
            % name-value pairs input
            obj@nr5g.internal.PTRSConfigBase(...
                'PTRSPortSet',ptrsPort,...
                varargin{:});
        end

        function obj = set.PTRSPortSet(obj,val)
            prop = 'PTRSPortSet';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~isempty(temp) || ~isnumeric(temp)
                validateattributes(temp, {'numeric'},...
                    {'scalar','integer','nonnegative'},...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
    end
end
