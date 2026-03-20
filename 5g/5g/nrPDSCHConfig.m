classdef nrPDSCHConfig < nr5g.internal.BWPSizeStart & nr5g.internal.nrPDSCHConfigBase
    %nrPDSCHConfig PDSCH configuration object
    %   CFGPDSCH = nrPDSCHConfig creates a physical downlink shared channel
    %   (PDSCH) configuration object that contains the properties related
    %   to TS 38.211 Sections 7.3.1, 7.4.1.1, and 7.4.1.2. This object
    %   bundles all the properties involved in the physical downlink shared
    %   channel processing chain, such as scrambling, symbol modulation,
    %   layer mapping, VRB-to-PRB interleaving, and resource element
    %   mapping along with the reserved resource patterns. The object also
    %   contains the properties of the associated physical reference
    %   signals, such as demodulation reference signal (DM-RS) and phase
    %   tracking reference signal (PT-RS). The default nrPDSCHConfig object
    %   configures the PDSCH with mapping type A, QPSK modulation, and a
    %   resource allocation of 52 resource blocks and 14 OFDM symbols in a
    %   slot. This corresponds to full resource allocation if used in
    %   combination with a default nrCarrierConfig object. By default,
    %   nrPDSCHConfig object configures single-symbol DM-RS configuration
    %   type 1.
    %
    %   CFGPDSCH = nrPDSCHConfig(Name,Value) creates a physical downlink
    %   shared channel configuration object CFGPDSCH with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPDSCHConfig properties (configurable):
    %
    %   NSizeBWP              - Size of the bandwidth part (BWP) in terms
    %                           of number of physical resource blocks
    %                           (PRBs) (1...275) (default [])
    %   NStartBWP             - Starting PRB index of BWP relative to
    %                           common resource block 0 (CRB 0) (0...2473)
    %                           (default [])
    %   ReservedPRB           - Reserved PRBs and OFDM symbol pattern(s) as
    %                           a cell array of object(s), of class
    %                           <a href="matlab:help('nrPDSCHReservedConfig')">nrPDSCHReservedConfig</a> with the properties:
    %      <a href="matlab:help('nrPDSCHReservedConfig/PRBSet')">PRBSet</a>    - Reserved PRB indices in BWP (0-based) (default [])
    %      <a href="matlab:help('nrPDSCHReservedConfig/SymbolSet')">SymbolSet</a> - OFDM symbols associated with reserved PRBs over one or more slots (default [])
    %      <a href="matlab:help('nrPDSCHReservedConfig/Period')">Period</a>    - Total number of slots in the pattern period (default [])
    %   ReservedRE            - Reserved resource element (RE) indices
    %                           within the BWP (0-based) (default [])
    %   Modulation            - Modulation scheme(s) of codeword(s)
    %                           ('QPSK' (default), '16QAM', '64QAM', '256QAM', '1024QAM')
    %   NumLayers             - Number of transmission layers (1...8) (default 1)
    %   MappingType           - PDSCH mapping type ('A' (default), 'B')
    %   SymbolAllocation      - OFDM symbol allocation of PDSCH within a
    %                           slot (default [0 14])
    %   PRBSet                - Resource block allocation (VRB or PRB indices)
    %                           (default 0:51)
    %   PRBSetType            - Type of indices used in the PRBSet property
    %                           ('VRB' (default), 'PRB')
    %   VRBToPRBInterleaving  - Virtual resource blocks (VRB) to physical
    %                           resource blocks interleaving (0 (default), 1)
    %   VRBBundleSize         - Bundle size in terms of number of RBs (2 (default), 4)
    %   NID                   - PDSCH scrambling identity (0...1023) (default [])
    %   RNTI                  - Radio network temporary identifier
    %                           (0...65535) (default 1)
    %   DMRS                  - PDSCH-specific DM-RS configuration object,
    %                           as described in <a href="matlab:help('nrPDSCHDMRSConfig')">nrPDSCHDMRSConfig</a> with properties:
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSConfigurationType')">DMRSConfigurationType</a>   - DM-RS configuration type (1 (default), 2)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSReferencePoint')">DMRSReferencePoint</a>      - The reference point for the DM-RS
    %                                sequence to subcarrier resource mapping
    %                                ('CRB0' (default), 'PRB0')
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSTypeAPosition')">DMRSTypeAPosition</a>       - Position of first DM-RS OFDM symbol
    %                                (2 (default), 3)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSAdditionalPosition')">DMRSAdditionalPosition</a>  - Maximum number of DM-RS additional positions
    %                                (0...3) (default 0)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSLength')">DMRSLength</a>              - Number of consecutive DM-RS OFDM symbols (1 (default), 2)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/CustomSymbolSet')">CustomSymbolSet</a>         - Custom DM-RS symbol locations (0-based) (default [])
    %      <a href="matlab:help('nrPDSCHDMRSConfig/DMRSPortSet')">DMRSPortSet</a>             - DM-RS antenna port set (0...11) (default []).
    %                                The default value ([]) implies that the values
    %                                are in the range from 0 to NumLayers-1
    %      <a href="matlab:help('nrPDSCHDMRSConfig/NIDNSCID')">NIDNSCID</a>                - DM-RS scrambling identity (0...65535) (default [])
    %      <a href="matlab:help('nrPDSCHDMRSConfig/NSCID')">NSCID</a>                   - DM-RS scrambling initialization (0 (default), 1)
    %      <a href="matlab:help('nrPDSCHDMRSConfig/NumCDMGroupsWithoutData')">NumCDMGroupsWithoutData</a> - Number of CDM groups without data (1...3) (default 2)
    %   EnablePTRS            - Enable or disable the PT-RS configuration (0 (default), 1)
    %   PTRS                  - PDSCH-specific PT-RS configuration object,
    %                           as described in <a href="matlab:help('nrPDSCHPTRSConfig')">nrPDSCHPTRSConfig</a> with properties:
    %      <a href="matlab:help('nrPDSCHPTRSConfig/TimeDensity')">TimeDensity</a>      - PT-RS time density (1 (default), 2, 4)
    %      <a href="matlab:help('nrPDSCHPTRSConfig/FrequencyDensity')">FrequencyDensity</a> - PT-RS frequency density (2 (default), 4)
    %      <a href="matlab:help('nrPDSCHPTRSConfig/REOffset')">REOffset</a>         - Resource element offset ('00' (default), '01', '10', '11')
    %      <a href="matlab:help('nrPDSCHPTRSConfig/PTRSPortSet')">PTRSPortSet</a>      - PT-RS antenna port set (default [])
    %
    %   nrPDSCHConfig properties (read-only):
    %
    %   NumCodewords          - Number of codewords
    %
    %   nrPDSCHConfig methods:
    %   
    %   nrTBS - Transport block size(s) associated with transmission
    % 
    %
    %   Example 1:
    %   % Create a default PDSCH configuration object occupying 10 MHz
    %   % bandwidth at 15 kHz subcarrier spacing (52 resource blocks) and
    %   % spanning over 14 OFDM symbols in a slot.
    %
    %   pdsch = nrPDSCHConfig;
    %
    %   Example 2:
    %   % Create a PDSCH configuration object that enables the VRB-to-PRB
    %   % interleaving and PT-RS.
    %
    %   pdsch = nrPDSCHConfig('VRBToPRBInterleaving',1,'EnablePTRS',1);
    %
    %   Example 3:
    %   % Create a PDSCH configuration object that configures two reserved
    %   % PRB patterns. 
    %
    %   pdsch = nrPDSCHConfig('ReservedPRB',{nrPDSCHReservedConfig,nrPDSCHReservedConfig});
    %   pdsch.ReservedPRB{1}.PRBSet = (0:15);
    %   pdsch.ReservedPRB{1}.SymbolSet = (5:6);
    %   pdsch.ReservedPRB{1}.Period = 5;
    %   pdsch.ReservedPRB{2}.PRBSet = (0:23);
    %   pdsch.ReservedPRB{2}.SymbolSet = [2:4 7:9];
    %   pdsch.ReservedPRB{2}.Period = 3;
    %
    %   % Example 4:
    %   % Create a PDSCH configuration object that configures double-symbol
    %   % DM-RS with the number of additional positions set to 3.
    %
    %   pdsch = nrPDSCHConfig;
    %   pdsch.DMRS.DMRSLength = 2; % 1 (single-symbol DM-RS), 2 (double-symbol DM-RS)
    %   pdsch.DMRS.DMRSAdditionalPosition = 3;
    %
    %   See also nrPDSCHDMRSConfig, nrPDSCHPTRSConfig,
    %   nrPDSCHReservedConfig, nrCarrierConfig.

    %   Copyright 2019-2024 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %ReservedRE Reserved resource element (RE) indices within bandwidth part (BWP)
        %   Specify the reserved RE indices (0-based) within BWP as a
        %   vector of nonnegative integers. These reserved RE indices are
        %   unavailable for PDSCH due to the presence of other signals in a
        %   particular slot, such as channel state information reference
        %   signal (CSI-RS) and LTE cell specific reference signal. The
        %   default value is [].
        ReservedRE = [];

        %NID Physical shared channel scrambling identity
        % Specify the physical shared channel scrambling identity as a
        % scalar nonnegative integer. The value must be in the range
        % 0...1023. It is the dataScramblingIdentityPDSCH (0...1023), if
        % configured, else it is the physical layer cell identity
        % (0...1007). Use empty ([]) to make this property equal to the
        % <a href="matlab:help('nrCarrierConfig/NCellID')"
        % >NCellID</a> property of nrCarrierConfig. The default value is [].
        NID = [];
    end
    
    % Hidden properties
    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'NSizeBWP','NStartBWP','ReservedPRB','ReservedRE',...
            'Modulation','NumLayers','MappingType','SymbolAllocation','PRBSet',...
            'PRBSetType','VRBToPRBInterleaving','VRBBundleSize',...
            'NID','RNTI','DMRS','EnablePTRS','PTRS','NumCodewords'};
    end

    methods
        % Constructor
        function obj = nrPDSCHConfig(varargin)
            % Get the value of NStartBWP from the name-value pairs
            nStartBWP = nr5g.internal.parseProp('NStartBWP',[],varargin{:});
            % Get the value of NSizeBWP from the name-value pairs
            nSizeBWP = nr5g.internal.parseProp('NSizeBWP',[],varargin{:});
            % Get the value of ReservedRE from the name-value pairs
            reservedRE = nr5g.internal.parseProp('ReservedRE',[],varargin{:});
            % Get the value of NID from the name-value pairs
            nid = nr5g.internal.parseProp('NID',[],varargin{:});
            
            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.nrPDSCHConfigBase(...
                'NStartBWP', nStartBWP, ...
                'NSizeBWP', nSizeBWP, ...
                'ReservedRE',reservedRE, ...
                'NID', nid, ...
                varargin{:});
        end

        % Set properties
        function obj = set.ReservedRE(obj,val)
            prop = 'ReservedRE';
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','integer','nonnegative'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = temp;
        end
        
        function obj = set.NID(obj,val)
            prop = 'NID';
            nid = validateNID(obj,val);
            obj.(prop) = nid;
        end

        function tbs = nrTBS(obj,tcr,varargin)
            % Signatures supported are,
            % nrTBS(pdsch,tcr)
            % nrTBS(pdsch,tcr,xOh)
            % nrTBS(pdsch,tcr,xOh,tbScaling)

            narginchk(1,4);
            [tcr,xOh,tbScaling] = nr5g.internal.TBSDetermination.transportChParamsLinear(tcr, 1, varargin{:}); % Include TB scaling look-up for the PDSCH
            tbs = nr5g.internal.TBSDetermination.getTBSEntry(obj,tcr,xOh,tbScaling);
        end

    end
end
