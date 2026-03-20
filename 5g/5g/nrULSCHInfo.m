function info = nrULSCHInfo(varargin)
%nrULSCHInfo 5G UL-SCH and UCI coding information
%   INFO = nrULSCHInfo(...) returns the structural information, INFO,
%   related to the uplink shared channel (UL-SCH) transport coding process,
%   with additional fields covering the uplink control information (UCI)
%   encoding and multiplexing. The function provides the information of bit
%   capacities (UL-SCH and UCI on PUSCH), given the physical uplink shared
%   channel (PUSCH) resource configuration and UCI payload lengths. The
%   function calculates the bit and symbol capacities of UCI, using the
%   formulas defined in TS 38.212 Section 6.3.2.4.
%
%   INFO is a structure containing the fields:
%   CRC     - CRC polynomial selection  for the first codeword ('16' or '24A')
%   L       - Number of CRC bits (16 or 24)
%   BGN     - LDPC base graph selection (1 or 2)
%   C       - Number of code blocks
%   Lcb     - Number of parity bits per code block (0 or 24)
%   F       - Number of <NULL> filler bits per code block
%   Zc      - Lifting size selection
%   K       - Number of bits per code block after CBS
%   N       - Number of bits per code block after LDPC coding
%   GULSCH  - Number of coded and rate matched UL-SCH data bits
%   GACK    - Number of coded and rate matched HARQ-ACK bits
%   GCSI1   - Number of coded and rate matched CSI part 1 bits
%   GCSI2   - Number of coded and rate matched CSI part 2 bits
%   GACKRvd - Number of reserved bits for HARQ-ACK
%   QdACK   - Number of coded HARQ-ACK symbols per layer (Q'_ACK)
%   QdCSI1  - Number of coded CSI part 1 symbols per layer (Q'_CSI1)
%   QdCSI2  - Number of coded CSI part 2 symbols per layer (Q'_CSI2)
%
%   INFO = nrULSCHInfo(TBS,TCR) returns the structure INFO for a given
%   transport block length TBS and target code rate TCR. TBS is a scalar
%   nonnegative integer. TCR is a positive scalar with value less than
%   1. This syntax does not specify the bit capacity of the PUSCH and the
%   function only returns the coding information up to the UL-SCH rate
%   matching step. The structure field relating to the number of rate
%   matched UL-SCH bits in the PUSCH codeword is set to empty. All fields
%   relating to UCI are set to 0.
%
%   INFO = nrULSCHInfo(PUSCH,TCR,TBS,OACK,OCSI1,OCSI2) supports the
%   multiplexing of UL-SCH and UCI on the PUSCH in addition to the above
%   syntax. This syntax requires the inputs PUSCH configuration PUSCH,
%   target code rate TCR, transport block size TBS, and payload lengths of
%   each UCI data (HARQ-ACK, CSI part 1, CSI part 2). The multiplexing is
%   either UCI data only or both UL-SCH and UCI data. Any of the data
%   length parameters can be zero, if the associated data is not present.
%   The inputs OACK, OCSI1, and OCSI2 are scalar nonnegative integers
%   specifying the payload lengths of HARQ-ACK, CSI part 1, and CSI part 2,
%   respectively. When PUSCH is configured with more than 4 layers, TCR and
%   TBS can be specified as two-element vectors or scalars indicating the
%   same value for both codewords, and each field in the INFO output is a
%   1-by-2 vector. This syntax does not support PUSCH interlacing.
%
%   PUSCH is the physical uplink shared channel configuration object, as
%   described in <a href="matlab:help('nrPUSCHConfig')">nrPUSCHConfig</a> with properties:
%
%   Modulation         - Modulation scheme
%                        ('QPSK' (default), 'pi/2-BPSK', '16QAM', '64QAM', '256QAM')
%   NumLayers          - Number of transmission layers (1...8) (default 1)
%   MappingType        - Mapping type of physical uplink shared channel
%                        ('A' (default), 'B')
%   SymbolAllocation   - Symbol allocation of physical uplink shared
%                        channel (default [0 14]). This property is a
%                        two-element vector. First element represents the
%                        start of OFDM symbol in a slot. Second element
%                        represents the number of contiguous OFDM symbols
%   PRBSet             - PRBs allocated for physical uplink shared channel
%                        within a BWP (0-based) (default 0:51)
%   TransformPrecoding - Flag to enable transform precoding
%                        (0 (default), 1). 0 indicates that transform
%                        precoding is disabled and the waveform type is
%                        CP-OFDM. 1 indicates that transform precoding is
%                        enabled and waveform type is DFT-s-OFDM
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   BetaOffsetACK      - Beta offset of HARQ-ACK (default 20)
%   BetaOffsetCSI1     - Beta offset of CSI part 1 (default 6.25)
%   BetaOffsetCSI2     - Beta offset of CSI part 2 (default 6.25)
%   UCIScaling         - UCI scaling factor (default 1). The nominal value
%                        is one of (0.5, 0.65, 0.8, 1)
%   RNTI               - Radio network temporary identifier (0...65535)
%                        (default 1)
%   DMRS               - PUSCH-specific DM-RS configuration object, as
%                        described in <a href="matlab:help('nrPUSCHDMRSConfig')">nrPUSCHDMRSConfig</a> with properties:
%       DMRSConfigurationType   - DM-RS configuration type (1 (default), 2).
%                                 When transform precoding is enabled, the
%                                 value must be 1
%       DMRSTypeAPosition       - Position of first DM-RS OFDM symbol in a
%                                 slot (2 (default), 3)
%       DMRSLength              - Number of consecutive DM-RS OFDM symbols
%                                 (1 (default), 2). When intra-slot
%                                 frequency hopping is enabled, the value
%                                 must be 1. Value of 1 indicates
%                                 single-symbol DM-RS. Value of 2 indicates
%                                 double-symbol DM-RS
%       DMRSAdditionalPosition  - Maximum number of DM-RS additional
%                                 positions (0...3) (default 0). When
%                                 intra-slot frequency hopping is enabled,
%                                 the value must be either 0 or 1
%       DMRSPortSet             - DM-RS antenna port set (0...11)
%                                 (default []). The default value implies
%                                 that the values are in the range from 0
%                                 to NumLayers-1
%       CustomSymbolSet         - Custom DM-RS symbol locations (0-based)
%                                 (default []). This property is used to
%                                 override the standard defined DM-RS
%                                 symbol locations. Each entry corresponds
%                                 to a single-symbol DM-RS
%       NumCDMGroupsWithoutData - Number of CDM groups without data
%                                 (1...3) (default 2). When transform
%                                 precoding is enabled, the value must be 2
%   EnablePTRS         - Enable or disable the PT-RS configuration
%                        (0 (default), 1)
%   PTRS               - PUSCH-specific PT-RS configuration object, as
%                        described in <a href="matlab:help('nrPUSCHPTRSConfig')">nrPUSCHPTRSConfig</a> with properties:
%       TimeDensity            - PT-RS time density (1 (default), 2, 4)
%    These properties are applicable, when transform precoding is set to 0:
%       FrequencyDensity       - PT-RS frequency density (2 (default), 4)
%       REOffset               - Resource element offset
%                                ('00' (default), '01', '10', '11')
%       PTRSPortSet            - PT-RS antenna port set (default []). The
%                                default value of empty ([]) implies the
%                                value is equal to the lowest DM-RS antenna
%                                port configured
%    These properties are applicable, when transform precoding is set to 1:
%       NumPTRSSamples         - Number of PT-RS samples (2 (default), 4)
%       NumPTRSGroups          - Number of PT-RS groups (2 (default), 4, 8)
%
%   OACK is the payload length of the HARQ-ACK bits, specified as a
%   nonnegative integer. A value of 0 indicates no HARQ-ACK transmission.
%
%   OCSI1 is the payload length of the CSI part 1 bits, specified as a
%   nonnegative integer. A value of 0 indicates no CSI part 1 transmission.
%
%   OCSI2 is the payload length of the CSI part 2 bits, specified as a
%   nonnegative integer. A value of 0 indicates no CSI part 2 transmission.
%
%   Example 1:
%   % Obtain UL-SCH information before rate matching for an input transport
%   % block of length 8456 and target code rate 517/1024. The info
%   % structure fields show that there are 312 filler bits, the total size
%   % of each code block after code block segmentation is 4576 and after
%   % LDPC coding is 13728 as well as other UL-SCH related information.
%
%   nrULSCHInfo(8456,517/1024)
%
%   Example 2:
%   % Obtain information of bit capacity of UL-SCH and UCI, for a specified
%   % configuration.
%
%   % Set the beta offsets of UCI types in PUSCH configuration
%   pusch = nrPUSCHConfig;
%   pusch.BetaOffsetACK = 10;
%   pusch.BetaOffsetCSI1 = 10;
%   pusch.BetaOffsetCSI2 = 10;
%   pusch.UCIScaling = 1;
%
%   % Set the target code rate, payload lengths of transport block,
%   % HARQ-ACK, CSI part 1, and CSI part 2
%   tcr = 517/1024;
%   tbs = 8456;
%   oack = 6;
%   ocsi1 = 40;
%   ocsi2 = 10;
%
%   info = nrULSCHInfo(pusch,tcr,tbs,oack,ocsi1,ocsi2)
%
%   See also nrULSCH, nrULSCHDecoder, nrPUSCH, nrPUSCHDecode,
%   nrULSCHMultiplex, nrULSCHDemultiplex, nrPUSCHConfig.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,6);

    if nargin == 2
        % nrULSCHInfo(tbs,tcr)
        tbs = varargin{1};
        tcr = varargin{2};

        % Call the shared channel utility
        info = nr5g.internal.getSCHInfo(tbs,tcr);
        info.GULSCH  = [];
        info.GACK    = 0;
        info.GCSI1   = 0;
        info.GCSI2   = 0;
        info.GACKRvd = 0;
        info.QdACK   = 0;
        info.QdCSI1  = 0;
        info.QdCSI2  = 0;

    else
        % nrULSCHInfo(pusch,tcr,tbs,ocsi1,ocsi2,oack)
        narginchk(6,6);

        % Parse and validate inputs
        fcnName = 'nrULSCHInfo';
        pusch = varargin{1};
        tcr   = varargin{2};
        tbs   = varargin{3};
        oack  = varargin{4};
        ocsi1 = varargin{5};
        ocsi2 = varargin{6};
        coder.internal.errorIf(~(isa(pusch,'nrPUSCHConfig') && isscalar(pusch)),'nr5g:nrPUSCH:InvalidPUSCHInput');
        coder.internal.errorIf(pusch.Interlacing,'nr5g:nrULSCHMultiplex:InterlacingNotSupported');
        coder.internal.errorIf(~any(numel(tcr)==[1 2]),'nr5g:nrULSCHInfo:InvalidNumTCR');
        coder.internal.errorIf(~any(numel(tbs)==[1 2]),'nr5g:nrULSCHInfo:InvalidNumTBS');
        validateConfig(pusch);
        validateattributes(oack,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'OACK');
        validateattributes(ocsi1,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'OCSI1');
        validateattributes(ocsi2,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'OCSI2');

        % Get the shared channel information
        info = nr5g.internal.pusch.getULSCHInfo(pusch,tcr,tbs,oack,ocsi1,ocsi2);
        
    end

end
