%PXSCHResources 5G NR PXSCH, DM-RS, and PT-RS resource element indices, DM-RS and PT-RS values
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.
%
%   [IND,DMRSIND,DMRS,PTRSIND,PTRS,INFO] = PXSCHResources(CARRIER,PXSCH)
%   returns the resource element (RE) indices for a 5G NR PXSCH
%   transmission, along with the associated PXSCH DM-RS and PT-RS, for a
%   given time (symbols) and frequency (PRBs) allocation of the PXSCH, the
%   DM-RS and PT-RS configuration. The 1-based linear PXSCH indices are
%   returned in matrix, IND. They are defined relative to a
%   three-dimensional RE grid representing a 14/12-symbol slot for the full
%   carrier (in the PXSCH numerology) across the layers/DM-RS ports of the
%   PXSCH. Each column of IND represents the grid locations for a separate
%   layer/port (the third dimension of the grid). The DM-RS and PT-RS RE
%   indices have the same format and are returned in matrices DMRSIND and
%   PTRSIND respectively. The complex values of DM-RS and PT-RS sequences
%   are also returned in matrices, DMRS and PTRS respectively. Additional
%   information about the DM-RS, PT-RS, and resourcing is returned in the
%   structure INFO.
%
%   The cell-wide settings input, CARRIER, must be an nrCarrierConfig
%   object.
%
%   The PXSCH specific input, PXSCH, must be either an nrPDSCHConfig object
%   or an nrPUSCHConfig object.
% 
%   [IND,DMRSIND,DMRS,PTRSIND,PTRS,INFO] =
%   PXSCHResources(BWP,CHS,NCELLID,NSLOT,RESERVEDPRB,RESERVEDRE) returns
%   the resource element (RE) indices for a 5G NR PXSCH transmission, along
%   with the associated PXSCH DM-RS and PT-RS, for a given time (symbols)
%   and frequency (PRBs) allocation of the PXSCH, the DM-RS and PT-RS
%   configuration.
%
%   The cell-wide settings input, BWP, must be an nrWavegenBWPConfig object.
%
%   The PXSCH specific input, CHS, must be either an nrWavegenPDSCHConfig
%   object or an nrWavegenPUSCHConfig object.
%
%   Input NCELLID is the physical layer cell identity (0...1007).
%
%   Input NSLOT is the slot number.
%
%   Periodically recurring patterns of reserved PRB for downlink can be
%   defined using the RESERVEDPRB parameter, which is a cell array of
%   nrPDSCHReservedPRB configuration objects. These PRBs will be excluded
%   from the generated indices and the DL-SCH/PDSCH processing should
%   rate-match around them. It can be used to exclude SS/PBCH,
%   CORESETs/PDCCH and other resources, as defined in TS 38.214 section
%   5.1.4.
%
%   Input RESERVEDRE (0-based indices) describes resource elements that are
%   not available for PDSCH due to the presence of channel state
%   information reference signal (CSI-RS) and LTE cell specific reference
%   signal in a particular slot.
%
%   In terms of frequency domain DM-RS density, there are two different RRC
%   signaled configuration types ('dmrs-Type'). Configuration type 1
%   defines 6 subcarriers per PRB per antenna port, comprising alternate
%   subcarriers. Configuration type 2 defines 4 subcarriers per PRB per
%   antenna ports, consisting of 2 groups of 2 neighboring subcarriers.
%   Different shifts are applied to the sets of subcarriers used,
%   depending on the associated antenna port or CDM group. For type 1,
%   there are 2 possible CDM groups/shifts across up to 8 possible antenna
%   ports (p=1000...1007), and, for type 2, there are 3 possible CDM
%   groups/shifts across 12 ports (p=1000...1011). For the full
%   configuration details, see TS 38.211 section 6.4.1.1 and Section
%   7.4.1.1 for uplink and downlink, respectively.
%
%   In terms of the time-domain DM-RS symbol positions, the PXSCH mapping
%   type ('MappingType') can be either slot-wise (type A) or non
%   slot-wise (type B). When a UE is scheduled to receive PXSCH by a DCI,
%   this mapping type is signaled by the time-domain resource field in the
%   grant. The field acts as an index into an RRC configured table where
%   each row in the table specifies a combination of mapping type, slot
%   offset, K0, the symbol start and length indicator, SLIV. The mapping
%   type specifies the relative locations of the associated DM-RS. For
%   slot-wise mapping type A, the first DM-RS symbol is signaled by a field
%   in the MIB to be either 2 or 3 ('dmrs-TypeA-Position'). For the non
%   slot-wise mapping type B, the first DM-RS symbol is always the first
%   symbol of the PXSCH time allocation.
% 
%   The maximum number of DM-RS OFDM symbols used by a UE is configured by
%   RRC signaling ('dmrs-AdditionalPosition' and 'maxLength'). The DM-RS
%   can be a set of single symbols, distributed roughly uniformly across
%   the allocated PXSCH symbols, or 1 or 2 pairs of neighboring or 'double
%   symbol' DM-RS. The 'maxLength' RRC parameter (1 or 2 respectively)
%   configures whether only single symbol DM-RS or either single or double
%   symbol DM-RS are used. In the latter case, the actual selection is
%   signaled in the DCI format 1_1 message. The 'dmrs-AdditionalPosition'
%   higher-layer parameter defines the number of single or double symbol
%   DM-RS that are transmitted. The valid combinations of these two
%   parameters is given by TS 38.211 tables 6.4.1.1.3-3, 6.4.1.1.3-4 and
%   6.4.1.1.3-6 for uplink and tables 7.4.1.1.2-3 and 7.4.1.1.2-4 for
%   downlink. In this function, the value of the 'DMRSLength' input
%   parameter directly controls whether either single or double symbols are
%   used.
%
%   INFO is the output structure containing the fields:
%   G             - Bit capacity of the PXSCH. This is the length of the
%                   codeword to be output from the XL-SCH transport channel
%   Gd            - Number of resource elements per layer/port, equal to 
%                   the number of rows in the PXSCH indices
%   NREPerPRB     - Number of RE per PRB allocated to PXSCH (not accounting
%                   for any reserved resources)
%   DMRSSymbolSet - The symbol numbers in a slot containing DM-RS (0-based)
%   PTRSSymbolSet - The symbol numbers in a slot containing PT-RS (0-based)
%   CDMGroups     - CDM groups associated with the DM-RS antenna ports
%   CDMLengths    - A 2-element row vector [FD TD] specifying the length of
%                   FD-CDM and TD-CDM despreading required during channel
%                   estimation. The values depend on the frequency and
%                   time masks applied to groups of antenna ports

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen

function [pxschIndices,dmrsIndices,dmrsSymbols,ptrsIndices,ptrsSymbols,pxschInfo] = PXSCHResources(varargin)

    % Argument check 
    narginchk(2, 6);

    if nargin == 2
        carrier = varargin{1};
        pxsch = varargin{2};
        isDownlink = isa(pxsch,'nrPDSCHConfig');
    else
        bwp = varargin{1};
        wgpxsch = varargin{2};
        nCellID = varargin{3};
        nslot = varargin{4};
        reservedPRB = varargin{5};
        reservedRE = varargin{6};
        isDownlink = isa(wgpxsch,'nrWavegenPDSCHConfig');
        [carrier,pxsch] = preparePXSCHFunctionInputs(bwp, wgpxsch, nCellID, nslot, reservedPRB, reservedRE, isDownlink);
    end
    
    if isDownlink
        % Get the PDSCH resource element indices and structural information
        [pxschIndicesAll,gInfo] = nrPDSCHIndices(carrier,pxsch);
        
        % Get the PDSCH DM-RS symbols and indices
        dmrsSymbols = nrPDSCHDMRS(carrier,pxsch);
        dmrsIndices = double(nrPDSCHDMRSIndices(carrier,pxsch));
        
        % Get the PDSCH PT-RS symbols and indices
        ptrsSymbols = nrPDSCHPTRS(carrier,pxsch);
        ptrsIndices = double(nrPDSCHPTRSIndices(carrier,pxsch));
    else % Uplink
        % Get the PUSCH resource element indices, structural information,
        % and the PT-RS resource element indices
        [pxschIndicesAll,gInfo,ptrsIndicesAll] = nrPUSCHIndices(carrier,pxsch);
        
        % Get the PUSCH DM-RS symbols and indices
        dmrsSymbols = nrPUSCHDMRS(carrier,pxsch);
        dmrsIndices = double(nrPUSCHDMRSIndices(carrier,pxsch));
        
        % Get the PUSCH PT-RS symbols and indices
        ptrsSymbols = nrPUSCHPTRS(carrier,pxsch);
        ptrsIndices = double(ptrsIndicesAll);
    end
    pxschIndices = double(pxschIndicesAll);

    % Combine information into output structure
    pxschInfo.G = gInfo.G;
    pxschInfo.Gd = gInfo.Gd;
    pxschInfo.NREPerPRB = gInfo.NREPerPRB;
    pxschInfo.DMRSSymbolSet = gInfo.DMRSSymbolSet;
    pxschInfo.PTRSSymbolSet = gInfo.PTRSSymbolSet;
    pxschInfo.CDMGroups = pxsch.DMRS.CDMGroups;
    pxschInfo.CDMLengths = pxsch.DMRS.CDMLengths;
    
    end

function [carrier,pxsch] = preparePXSCHFunctionInputs(bwp, wavePXSCH, nCellID, nslot, reservedPRBIn, reservedRE, isDownlink)
%preparePXSCHFunctionInputs Creates nrCarrierConfig and nrPXSCHConfig
%objects needed by nrPXSCHIndices, nrPXSCHDMRSIndics, nrPXSCHDMRS,
%nrPXSCHPTRSIndices and nrPXSCHPTRS
%
%   [CARRIER,PXSCH] = preparePXSCHFunctionInputs(BWP,WAVEPXSCH,NCELLID,NSLOT,RESERVEDPRBIN,RESERVEDRE,ISDOWNLINK)
%   provides the carrier configuration object CARRIER and nrPXSCHConfig
%   PXSCH, given the input nrWavegenBWPConfig object BWP,
%   nrWavegenPXSCHConfig object WAVEPXSCH, cell identity NCELLID, slot
%   number NSLOT, reserved resource blocks RESERVEDPRBIN, and reserved RE
%   (for CSI-RS) RESERVEDRE. The input ISDOWNLINK defines the link
%   direction (downlink or uplink).

    % Get the carrier configuration object
    carrier = nr5g.internal.wavegen.getCarrierCfgObject(bwp, nCellID);
    carrier.NSlot = nslot;

    % Get the shared channel configuration object
    pxsch = nr5g.internal.wavegen.getPXSCHObject(wavePXSCH, carrier.SymbolsPerSlot, reservedPRBIn, reservedRE, isDownlink);
end