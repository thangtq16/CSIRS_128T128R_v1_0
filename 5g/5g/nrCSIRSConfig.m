classdef nrCSIRSConfig < nr5g.internal.nrCSIRSConfigBase
    %nrCSIRSConfig CSI-RS configuration object
    %   CSIRS = nrCSIRSConfig creates a Channel State Information Reference
    %   Signal (CSI-RS) configuration object for single or multiple CSI-RS
    %   resources. This object contains the properties related to TS 38.211
    %   Section 7.4.1.5. By default, the object defines an NZP-CSI-RS
    %   resource configured for 2 antenna ports with CDM type equal to
    %   FD-CDM2 and density equal to 1 (corresponding to the row number 3
    %   in TS 38.211 Table 7.4.1.5.3-1).
    %
    %   CSIRS = nrCSIRSConfig(Name,Value) creates a CSI-RS configuration
    %   object CSIRS with the specified property Name set to the specified
    %   Value. You can specify additional name-value arguments in any order
    %   as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrCSIRSConfig properties (configurable):
    %
    %   CSIRSType             - CSI-RS type ('nzp' (default), 'zp')
    %   CSIRSPeriod           - CSI-RS slot periodicity (Tcsi-rs) and
    %                           offset (Toffset)
    %                           ('on' (default), 'off', [Tcsi-rs Toffset])
    %   RowNumber             - Row number corresponding to a CSI-RS
    %                           resource as defined in TS 38.211
    %                           Table 7.4.1.5.3-1 (1...18) (default 3)
    %   Density               - CSI-RS resource frequency density
    %                           ('one' (default), 'three', 'dot5even', 'dot5odd')
    %   SymbolLocations       - Time-domain locations of a CSI-RS resource
    %                           (default 0)
    %   SubcarrierLocations   - Frequency-domain locations of a CSI-RS
    %                           resource (default 0)
    %   NumRB                 - Number of resource blocks (RBs) allocated
    %                           for a CSI-RS resource (1...275) (default 52)
    %   RBOffset              - Starting RB index of CSI-RS allocation
    %                           relative to carrier resource grid
    %                           (0...274) (default 0)
    %   NID                   - Scrambling identity (0...1023) (default 0)
    %
    %   nrCSIRSConfig properties (read-only):
    %
    %   NumCSIRSPorts         - Number of CSI-RS specific antenna ports
    %   CDMType               - CDM type of a CSI-RS resource
    %
    %   Note that each element of CSIRSType configures one CSI-RS resource.
    %
    %   Example 1:
    %   %  Create nrCSIRSConfig object with its default properties.
    %
    %   csirs = nrCSIRSConfig
    %
    %   Example 2:
    %   %  Create nrCSIRSConfig object with CSIRSType as 'zp'.
    %
    %   csirs = nrCSIRSConfig('CSIRSType','zp')
    %
    %   Example 3:
    %   %  Generate CSI-RS specific configuration object for 3 resources
    %   %  (ZP, NZP, ZP) with row numbers [5 3 8].
    %
    %   csirs = nrCSIRSConfig('CSIRSType',{'zp','nzp','zp'},...
    %       'CSIRSPeriod',{'on','on','off'},'RowNumber',[5 3 8],...
    %       'Density',{'one','dot5odd','one'},'SymbolLocations',{6,10,9},...
    %       'SubcarrierLocations',{0,0,[0 4]})
    %
    %   See also nrCarrierConfig, nrCSIRS, nrCSIRSIndices.

    %   Copyright 2019-2024 The MathWorks, Inc.

    %#codegen

    % Read-only properties
    properties (SetAccess = private)

        %CDMType CDM type of CSI-RS resource(s)
        %   CDM type of one or more CSI-RS resource configuration(s). This
        %   property is read-only and is updated based on the property
        %   RowNumber.
        CDMType;
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        CDMType_Options         = {'noCDM','noCDM','FD-CDM2','FD-CDM2',...
                                   'FD-CDM2','FD-CDM2','FD-CDM2','CDM4',...
                                   'FD-CDM2','CDM4','FD-CDM2','CDM4',...
                                   'FD-CDM2','CDM4','CDM8','FD-CDM2','CDM4','CDM8'};
    end

    properties (Hidden)
        CustomPropList = {'CSIRSType'; 'CSIRSPeriod'; 'RowNumber'; 'Density'; ...
            'SymbolLocations'; 'SubcarrierLocations'; 'NumRB'; 'RBOffset'; 'NID'; ...
            'NumCSIRSPorts'; 'CDMType'};
    end

    methods

        % Constructor
        function obj = nrCSIRSConfig(varargin)
            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.nrCSIRSConfigBase(varargin{:});
        end

        function out = get.CDMType(obj)
            % CDMType is updated based on RowNumber
            if isscalar(obj.RowNumber)
                if iscell(obj.CSIRSType)
                    out = {obj.CDMType_Options{obj.RowNumber(1)}};
                else
                    out = obj.CDMType_Options{obj.RowNumber};
                end
            else
                out = cell(1,numel(obj.RowNumber));
                for rowIdx = 1:numel(obj.RowNumber)
                    out{rowIdx} = obj.CDMType_Options{obj.RowNumber(rowIdx)};
                end
            end
        end
        
        function out = validateConfig(obj)
            %validateConfig Validate the nrCSIRSConfig object
            %   OUT = validateConfig(OBJ) validates the inter dependent
            %   properties of specified nrCSIRSConfig configuration object
            %   and returns one structure OUT with the updated CSI-RS
            %   parameters.

            out = validateConfig@nr5g.internal.nrCSIRSConfigBase(obj);

            out.NumCSIRSPorts = obj.NumCSIRSPorts;
            out.CDMType = obj.CDMType;
        end

    end
end
