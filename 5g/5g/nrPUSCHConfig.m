classdef nrPUSCHConfig < nr5g.internal.BWPSizeStart & nr5g.internal.pusch.ConfigBase
    %nrPUSCHConfig PUSCH configuration object
    %   CFGPUSCH = nrPUSCHConfig creates a physical uplink shared channel
    %   (PUSCH) configuration object that contains the properties related
    %   to TS 38.211 Section 6.3.1, 6.4.1.1, and 6.4.1.2. This object
    %   bundles all the properties involved in the PUSCH processing chain,
    %   such as, scrambling, symbol modulation, layer mapping, transform
    %   precoding, MIMO precoding, and resource element mapping. The object
    %   also contains the properties of the associated physical reference
    %   signals, such as, demodulation reference signal (DM-RS) and phase
    %   tracking reference signal (PT-RS). The default nrPUSCHConfig object
    %   configures the PUSCH with CP-OFDM, mapping type A, QPSK modulation,
    %   and a resource allocation of 52 resource blocks and 14 OFDM symbols
    %   in a slot. This corresponds to full resource allocation if used in
    %   combination with a default nrCarrierConfig object. By default,
    %   nrPDSCHConfig object configures single-symbol DM-RS configuration
    %   type 1.
    %
    %   CFGPUSCH = nrPUSCHConfig(Name,Value) creates a physical uplink shared
    %   channel configuration object PUSCH with the specified property Name
    %   set to the specified Value. You can specify additional name-value
    %   arguments in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPUSCHConfig properties (configurable):
    %
    %   NSizeBWP           - Size of the bandwidth part (BWP) in terms
    %                        of number of physical resource blocks
    %                        (PRBs) (1...275) (default [])
    %   NStartBWP          - Starting PRB index of BWP relative to
    %                        common resource block 0 (CRB 0) (0...2473)
    %                        (default [])
    %   Modulation         - Modulation scheme(s) of codeword(s)
    %                        ('QPSK' (default), 'pi/2-BPSK', '16QAM', '64QAM', '256QAM')
    %   NumLayers          - Number of transmission layers (1...8) (default 1)
    %   MappingType        - PUSCH mapping type ('A' (default), 'B')
    %   SymbolAllocation   - OFDM symbol allocation of PUSCH within a slot
    %                        (default [0 14])
    %   PRBSet             - PRBs allocated for PUSCH within the BWP (default 0:51)
    %   TransformPrecoding - Flag to enable transform precoding (0(default), 1).
    %                        0 indicates that transform precoding is
    %                        disabled and the waveform type is CP-OFDM. 1
    %                        indicates that transform precoding is enabled
    %                        and the waveform type is DFT-s-OFDM
    %   TransmissionScheme - PUSCH transmission scheme ('nonCodebook' (default), 'codebook')
    %   NumAntennaPorts    - Number of antenna ports (1 (default), 2, 4, 8)
    %   TPMI               - Transmitted precoding matrix indicator (0...304) (default 0)
    %   CodebookType       - Codebook type ('codebook1_ng1n4n1' (default),
    %                        'codebook1_ng1n2n2', 'codebook2', 'codebook3', 'codebook4')
    %   FrequencyHopping   - Flag to enable frequency hopping
    %                        ('neither' (default), 'intraSlot', 'interSlot')
    %   SecondHopStartPRB  - Starting PRB of second hop relative to the
    %                        BWP (0...274) (default 1)
    %   Interlacing        - Enable interlacing (default false)
    %   RBSetIndex         - Resource block set index (default 0)
    %   InterlaceIndex     - Interlace indices (0...9) (default 0)
    %   BetaOffsetACK      - Beta offset for HARQ-ACK (default 20)
    %   BetaOffsetCSI1     - Beta offset for CSI part 1 (default 6.25)
    %   BetaOffsetCSI2     - Beta offset for CSI part 2 (default 6.25)
    %   UCIScaling         - Scaling factor to limit the number of resource
    %                        elements for UCI on PUSCH (default 1)
    %   NID                - PUSCH scrambling identity (0...1023) (default [])
    %   RNTI               - Radio network temporary identifier (0...65535)
    %                        (default 1)
    %   NRAPID             - Random access preamble index to initialize the
    %                        scrambling sequence for msgA on PUSCH (0...63)
    %                        (default [])
    %   DMRS               - PUSCH-specific DM-RS configuration object, as
    %                        described in <a href="matlab:help('nrPUSCHDMRSConfig')">nrPUSCHDMRSConfig</a> with properties:
    %       <a href="matlab:help('nrPUSCHDMRSConfig/DMRSConfigurationType')">DMRSConfigurationType</a>   - DM-RS configuration type (1 (default), 2).
    %                                 For DFT-s-OFDM, the value must be 1
    %       <a href="matlab:help('nrPUSCHDMRSConfig/DMRSTypeAPosition')">DMRSTypeAPosition</a>       - Position of first DM-RS OFDM symbol
    %                                 (2 (default), 3)
    %       <a href="matlab:help('nrPUSCHDMRSConfig/DMRSAdditionalPosition')">DMRSAdditionalPosition</a>  - Maximum number of DM-RS additional positions
    %                                 (0...3) (default 0). When intra-slot
    %                                 frequency hopping is enabled, the
    %                                 value must be either 0 or 1
    %       <a href="matlab:help('nrPUSCHDMRSConfig/DMRSLength')">DMRSLength</a>              - Number of consecutive DM-RS OFDM symbols
    %                                 (1 (default), 2). When intra-slot
    %                                 frequency hopping is enabled, the
    %                                 value must be 1
    %       <a href="matlab:help('nrPUSCHDMRSConfig/CustomSymbolSet')">CustomSymbolSet</a>         - Custom DM-RS symbol locations (0-based)
    %                                 (default [])
    %       <a href="matlab:help('nrPUSCHDMRSConfig/DMRSPortSet')">DMRSPortSet</a>             - DM-RS antenna port set (0...11)
    %                                 (default []). The default value ([])
    %                                 implies that the values are in the
    %                                 range from 0 to NumLayers-1
    %       <a href="matlab:help('nrPUSCHDMRSConfig/NIDNSCID')">NIDNSCID</a>                - DM-RS scrambling identity for CP-OFDM
    %                                 (0...65535) (default [])
    %       <a href="matlab:help('nrPUSCHDMRSConfig/NSCID')">NSCID</a>                   - DM-RS scrambling initialization
    %                                 (0 (default), 1). This property is
    %                                 used only for CP-OFDM
    %       <a href="matlab:help('nrPUSCHDMRSConfig/GroupHopping')">GroupHopping</a>            - Group hopping configuration
    %                                 (0 (default), 1). This property is
    %                                 used only for DFT-s-OFDM
    %       <a href="matlab:help('nrPUSCHDMRSConfig/SequenceHopping')">SequenceHopping</a>         - Sequence hopping configuration
    %                                 (0 (default), 1). This property is
    %                                 used only for DFT-s-OFDM
    %       <a href="matlab:help('nrPUSCHDMRSConfig/NRSID')">NRSID</a>                   - DM-RS scrambling identity for
    %                                 DFT-s-OFDM (0...1007) (default [])
    %       <a href="matlab:help('nrPUSCHDMRSConfig/NumCDMGroupsWithoutData')">NumCDMGroupsWithoutData</a> - Number of CDM groups without data
    %                                 (1...3) (default 2). For DFT-s-OFDM,
    %                                 the value must be 2
    %   EnablePTRS         - Enable or disable the PT-RS configuration
    %                        (0 (default), 1)
    %   PTRS               - PUSCH-specific PT-RS configuration object, as
    %                        described in <a href="matlab:help('nrPUSCHPTRSConfig')">nrPUSCHPTRSConfig</a> with properties:
    %       <a href="matlab:help('nrPUSCHPTRSConfig/TimeDensity')">TimeDensity</a>      - PT-RS time density (1 (default), 2, 4)
    %       <a href="matlab:help('nrPUSCHPTRSConfig/FrequencyDensity')">FrequencyDensity</a> - PT-RS frequency density for CP-OFDM
    %                          (2 (default), 4)
    %       <a href="matlab:help('nrPUSCHPTRSConfig/NumPTRSSamples')">NumPTRSSamples</a>   - Number of PT-RS samples for DFT-s-OFDM
    %                          (2 (default), 4)
    %       <a href="matlab:help('nrPUSCHPTRSConfig/NumPTRSGroups')">NumPTRSGroups</a>    - Number of PT-RS groups for DFT-s-OFDM
    %                          (2 (default), 4, 8)
    %       <a href="matlab:help('nrPUSCHPTRSConfig/REOffset')">REOffset</a>         - Resource element offset for CP-OFDM
    %                          ('00' (default), '01', '10', '11')
    %       <a href="matlab:help('nrPUSCHPTRSConfig/PTRSPortSet')">PTRSPortSet</a>      - PT-RS antenna port set for CP-OFDM
    %                          (default [])
    %       <a href="matlab:help('nrPUSCHPTRSConfig/NID')">NID</a>              - PT-RS scrambling identity for DFT-s-OFDM
    %                          (0...1007) (default [])
    %
    %   nrPUSCHConfig properties (read-only):
    %
    %   NumCodewords          - Number of codewords
    % 
    %   nrPUSCHConfig methods:
    %   
    %   nrTBS - Transport block size(s) associated with transmission
    %
    %   Example 1:
    %   % Create a default PUSCH configuration object occupying 10 MHz
    %   % bandwidth at 15 kHz subcarrier spacing (52 resource blocks) and
    %   % spanning over 14 OFDM symbols in a slot.
    %
    %   pusch = nrPUSCHConfig
    %
    %   Example 2:
    %   % Create a PUSCH configuration object that enables transform
    %   % precoding and PT-RS.
    %
    %   pusch = nrPUSCHConfig('TransformPrecoding',1,'EnablePTRS',1)
    %
    %   See also nrPUSCHDMRSConfig, nrPUSCHPTRSConfig, nrCarrierConfig.

    %   Copyright 2019-2024 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %NID Physical shared channel scrambling identity
        % Specify the physical shared channel scrambling identity as a
        % scalar nonnegative integer. The value must be in the range
        % 0...1023. It is the dataScramblingIdentityPUSCH (0...1023), if
        % configured, else it is the physical layer cell identity
        % (0...1007). Use empty ([]) to make this property equal to the
        % <a href="matlab:help('nrCarrierConfig/NCellID')"
        % >NCellID</a> property of nrCarrierConfig. The default value is [].
        NID = [];
    end
    
    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'NSizeBWP','NStartBWP','Modulation','NumLayers',...
            'MappingType','SymbolAllocation','PRBSet',...
            'TransformPrecoding','TransmissionScheme','NumAntennaPorts',...
            'TPMI','CodebookType','FrequencyHopping','SecondHopStartPRB',...
            'Interlacing','RBSetIndex','InterlaceIndex',...
            'BetaOffsetACK','BetaOffsetCSI1','BetaOffsetCSI2','UCIScaling',...
            'NID','RNTI','NRAPID','DMRS','EnablePTRS','PTRS','NumCodewords'};
    end

    methods
        % Constructor
        function obj = nrPUSCHConfig(varargin)
            % Get the value of NStartBWP from the name-value pairs
            nStartBWP = nr5g.internal.parseProp('NStartBWP',[],varargin{:});
            % Get the value of NSizeBWP from the name-value pairs
            nSizeBWP = nr5g.internal.parseProp('NSizeBWP',[],varargin{:});
            % Get the value of NID from the name-value pairs
            nid = nr5g.internal.parseProp('NID',[],varargin{:});
            % Get the value of NRAPID from the name-value pairs
            nrapid = nr5g.internal.parseProp('NRAPID',[],varargin{:});
            
            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.pusch.ConfigBase(...
                'NStartBWP', nStartBWP, ...
                'NSizeBWP', nSizeBWP, ...
                'NID', nid, ...
                'NRAPID', nrapid, ...
                varargin{:});
        end
        
        % Self-validate and set properties
        function obj = set.NID(obj,val)
            prop = 'NID';
            nid = validateNID(obj,val);
            obj.(prop) = nid;
        end

        function tbs = nrTBS(obj,tcr,varargin)
            % Signatures supported are,
            % nrTBS(pusch, tcr)
            % nrTBS(pusch, tcr,xOh)
            % nrTBS(pusch, tcr,xOh, slotcarrier) - Mandatory in the interlaced PRB allocation case

            % Input argument checking and preprocessing 
            narginchk(1,4);
            if nargin == 4
                validateattributes(varargin{2},"nrCarrierConfig","scalar","nrTBS","CARRIER");
            end

            [tcr,xOh] = nr5g.internal.TBSDetermination.transportChParamsLinear(tcr, 0, varargin{:});  % Don't include TB scaling in the argument parsing  
            
            interlacing = obj.Interlacing;
            if interlacing
                % Check input signature for nrTBS(channel,tcr,xoh,carrier:nrCarrierConfig) syntax
                errFlag = interlacing && nargin ~= 4;
                coder.internal.errorIf(errFlag,'nr5g:nrTBS:InvalidSigForInterlacedPUSCH');
    
                % Dispatch to interlaced PUSCH specific function to include the PRB
                % allocation for this case
                tbs = getTBSEntryInterlaced(obj,tcr,xOh,obj,varargin{2});
            else            
                % Otherwise perform standard calculation
                tbs = nr5g.internal.TBSDetermination.getTBSEntry(obj,tcr,xOh,1);
            end
        end
       
    end

end
