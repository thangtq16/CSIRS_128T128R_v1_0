function [ind,info] = nrPDSCHIndices(carrier,pdsch,varargin)
%nrPDSCHIndices Physical downlink shared channel resource element indices
%   [IND,INFO] = nrPDSCHIndices(CARRIER,PDSCH) returns the matrix IND
%   containing 1-based physical downlink shared channel resource element
%   (RE) indices within the carrier resource grid, in linear form. The
%   matrix IND is obtained from TS 38.211 Sections 7.3.1.5 and 7.3.1.6, for
%   the given carrier configuration CARRIER and downlink shared channel
%   configuration PDSCH. The number of columns in IND is equal to the
%   number of antenna ports configured. This syntax also provides the
%   structural information INFO about the bit capacity, symbol capacity,
%   the DM-RS OFDM symbol locations, and PT-RS OFDM symbol locations
%   associated with the physical downlink shared channel.
%
%   CARRIER is a carrier configuration object as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with the following properties:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15 (default), 30, 60, 120, 240, 480, 960)
%   CyclicPrefix      - Cyclic prefix ('normal' (default), 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource
%                       grid (1...275) (default 52)
%   NStartGrid        - Start of carrier resource grid relative to common
%                       resource block 0 (CRB 0) (0...2199) (default 0)
%   NSlot             - Slot number (default 0)
%
%   PDSCH is the physical downlink shared channel configuration object as
%   described in <a href="matlab:help('nrPDSCHConfig')">nrPDSCHConfig</a> with the following properties:
%
%   NSizeBWP              - Size of the bandwidth part (BWP) in terms
%                           of number of physical resource blocks (PRBs)
%                           (1...275) (default []). The default value
%                           implies the value is equal to the size of
%                           carrier resource grid
%   NStartBWP             - Starting PRB index of BWP relative to CRB 0
%                           (0...2473) (default []). The default value
%                           implies the value is equal to the start of
%                           carrier resource grid
%   ReservedPRB           - Cell array of object(s) containing the reserved
%                           physical resource blocks and OFDM symbols
%                           pattern, as described in <a href="matlab:help('nrPDSCHReservedConfig')">nrPDSCHReservedConfig</a>
%                           with properties:
%       PRBSet    - Reserved PRB indices in BWP (0-based) (default [])
%       SymbolSet - OFDM symbols associated with reserved PRBs over one or
%                   more slots (default [])
%       Period    - Total number of slots in the pattern period (default [])
%   ReservedRE            - Reserved resource element (RE) indices
%                           within BWP (0-based) (default [])
%   Modulation            - Modulation scheme(s) of codeword(s)
%                           ('QPSK' (default), '16QAM', '64QAM', '256QAM', '1024QAM')
%   NumLayers             - Number of transmission layers (1...8)
%                           (default 1)
%   MappingType           - Mapping type of physical downlink shared
%                           channel ('A' (default), 'B')
%   SymbolAllocation      - Symbol allocation of physical downlink shared
%                           channel (default [0 14]). This property is a
%                           two-element vector. First element represents
%                           the start of OFDM symbol in a slot. Second
%                           element represents the number of contiguous
%                           OFDM symbols
%   PRBSet                - Resource block allocation (VRB or PRB indices)
%                           (default 0:51)
%   PRBSetType            - Type of indices used in the PRBSet property
%                           ('VRB' (default), 'PRB')
%   VRBToPRBInterleaving  - Flag to enable VRB to PRB interleaving
%                           (0 (default),1)
%   VRBBundleSize         - Bundle size in terms of number of resource
%                           blocks (2 (default),4)
%   RNTI                  - Radio network temporary identifier (0...65535)
%                           (default 1)
%   DMRS                  - PDSCH-specific DM-RS configuration object, as
%                           described in <a href="matlab:help('nrPDSCHDMRSConfig')">nrPDSCHDMRSConfig</a> with properties:
%       DMRSConfigurationType   - DM-RS configuration type (1 (default), 2)
%       DMRSReferencePoint      - The reference point for the DM-RS
%                                 sequence to subcarrier resource mapping
%                                 ('CRB0' (default), 'PRB0')
%       DMRSTypeAPosition       - Position of first DM-RS OFDM symbol in a
%                                 slot (2 (default), 3)
%       DMRSLength              - Number of consecutive DM-RS OFDM symbols
%                                 (1 (default), 2)
%       DMRSAdditionalPosition  - Maximum number of DM-RS additional
%                                 positions (0...3) (default 0)
%       CustomSymbolSet         - Custom DM-RS symbol locations (0-based)
%                                 (default []). This property is used to
%                                 override the standard defined DM-RS
%                                 symbol locations. Each entry corresponds
%                                 to a single-symbol DM-RS
%       DMRSPortSet             - DM-RS antenna port set (0...11)
%                                 (default []). The default value implies
%                                 that the values are in the range from 0
%                                 to NumLayers-1
%       NumCDMGroupsWithoutData - Number of CDM groups without data (1...3)
%                                 (default 2)
%   EnablePTRS            - Enable or disable the PT-RS configuration
%                           (0 (default), 1). The value of 0 implies PT-RS
%                           is disabled and value of 1 implies PT-RS is
%                           enabled
%   PTRS                  - PDSCH-specific PT-RS configuration object, as
%                           described in <a href="matlab:help('nrPDSCHPTRSConfig')">nrPDSCHPTRSConfig</a> with properties:
%       TimeDensity      - PT-RS time density (1 (default), 2, 4)
%       FrequencyDensity - PT-RS frequency density (2 (default), 4)
%       REOffset         - PT-RS resource element offset
%                          ('00' (default), '01', '10', '11')
%       PTRSPortSet      - PT-RS antenna port set (default []). The default
%                          value implies the value is equal to the lowest
%                          DM-RS antenna port configured
%
%   The output structure INFO contains the following fields:
%   G             - Bit capacity of the PDSCH. This must be the
%                   length of codeword from the DL-SCH transport channel
%   Gd            - Number of resource elements per layer/port
%   DMRSSymbolSet - The OFDM symbol locations in a slot containing DM-RS
%                   (0-based)
%   NREPerPRB     - Number of RE per PRB allocated to PDSCH (not
%                   accounting for any reserved resources)
%   PTRSSymbolSet - The OFDM symbol locations in a slot containing PT-RS
%                   (0-based)
%   PRBSet        - PRBs allocated for PDSCH within the BWP
%
%   IND = nrPDSCHIndices(CARRIER,PDSCH,NAME,VALUE,...) specifies
%   additional options as NAME,VALUE pairs to allow control over the format
%   of the indices:
%
%    'IndexStyle'       - 'index' for linear indices (default)
%                         'subscript' for [subcarrier, symbol, antenna]
%                         subscript row form
%
%    'IndexBase'        - '1based' for 1-based indices (default)
%                         '0based' for 0-based indices
%
%    'IndexOrientation' - 'carrier' for carrier oriented indices (default)
%                         'bwp' for bandwidth part oriented indices
%
%   Example:
%   % Generate the data symbol indices (0-based) in linear index form of a
%   % physical downlink shared channel occupying the 10 MHz bandwidth for a
%   % 15 kHz subcarrier spacing (SCS) carrier. Configure DM-RS with length
%   % set to 1, type A position set to 2, number of additional positions
%   % set to 0, number of CDM groups without data set to 2, and
%   % configuration type set to 1.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pdsch = nrPDSCHConfig;
%   pdsch.DMRS.DMRSLength = 1;
%   pdsch.DMRS.DMRSTypeAPosition = 2;
%   pdsch.DMRS.DMRSAdditionalPosition = 0;
%   pdsch.DMRS.NumCDMGroupsWithoutData = 2;
%   pdsch.DMRS.DMRSConfigurationType = 1;
%   ind = nrPDSCHIndices(carrier,pdsch,'IndexBase','0based');
%
%   See also nrPDSCH, nrPDSCHDecode, nrPDSCHDMRSIndices,
%   nrPDSCHPTRSIndices, nrPDSCHConfig, nrPDSCHDMRSConfig,
%   nrPDSCHPTRSConfig, nrCarrierConfig.

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate inputs
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot] = nr5g.internal.pdsch.validateInputs(carrier,pdsch);

    % Get prbset, symbolset and DM-RS symbols
    [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pdsch,nSizeBWP);
    nRBSC = 12; % Number of subcarriers in a resource block

    % VRB-To-PRB interleaving
    if pdsch.VRBToPRBInterleaving && strcmpi(pdsch.PRBSetType,'PRB')
        % When the resource block allocation type is PRB, identify the data
        % associated with the VRB blocks that would map onto the specified
        % PRBs.

        % Get the RB reference point for the interleaver. If the value of
        % pdsch.DMRS.DMRSReferencePoint is PRB0, the RB reference point is
        % set to 0, assuming that the PDSCH is signaled via CORESET 0, as
        % described in TS 38.211 Section 7.3.1.6.
        rbrefpoint = nr5g.internal.pdsch.getRBReferencePoint(carrier.NStartGrid,pdsch.NStartBWP,pdsch.DMRS.DMRSReferencePoint);
        % Reference PRB order for all the resource blocks in BWP
        mapIndices = nr5g.internal.pdsch.vrbToPRBInterleaver(nSizeBWP,rbrefpoint,double(pdsch.VRBBundleSize));
        % Map the input resource blocks in the order of mapIndices
        mapMatrix = repmat(mapIndices,numel(prbset),1) == repmat(reshape(prbset,[],1),1,nSizeBWP);
        prbsetInterleave = mapIndices(any(mapMatrix,1));
    else
        prbsetInterleave = prbset;
    end

    % Remove the reserved resource blocks
    prbcell = nr5g.internal.pdsch.extractActiveResourceBlocks(pdsch.ReservedPRB,prbsetInterleave,symbolset,carrier.NSlot,carrier.SymbolsPerSlot);

    % Capture set of antenna ports required, number of ports and CDM groups
    % without data
    nports = double(pdsch.NumLayers);
    if isempty(pdsch.DMRS.DMRSPortSet)
        ports = 0:nports-1;
    else
        ports = double(pdsch.DMRS.DMRSPortSet);
    end
    cdmgroupsnodata = double(pdsch.DMRS.NumCDMGroupsWithoutData);

    % DM-RS subcarrier (SC) locations in a resource block
    if pdsch.DMRS.DMRSConfigurationType==1
        % Type 1: 6 DM-RS SC per PRB per CDM (every other SC)
        dmrssc = [0 2 4 6 8 10]';                   % RE indices in a PRB
        dshiftsnodata = 0:min(cdmgroupsnodata,2)-1; % Delta shifts for CDM groups without data
    else
        % Type 2: 4 DM-RS SC per PRB per CDM (2 groups of 2 SC)
        dmrssc = [0 1 6 7]';                            % RE indices in a PRB
        dshiftsnodata = 2*(0:min(cdmgroupsnodata,3)-1); % Delta shifts for CDM groups without data
    end
    dshifts = pdsch.DMRS.DeltaShifts;

    % Non DM-RS resource elements in a DM-RS containing symbol
    fullprb = ones(nRBSC,1);     % Binary map of all the subcarriers in an RB
    dshiftsComp = [dshifts dshiftsnodata];
    dmrsre = repmat(dmrssc,1,numel(dshiftsComp)) + repmat(dshiftsComp, numel(dmrssc),1);
    fullprb(dmrsre+1) = 0;       % Clear all RE which will carry DM-RS in at least one port
    pdschre = find(fullprb)-1;   % Find PDSCH (non DM-RS) RE in a DM-RS containing symbol

    % Assign the resource elements of a single RB used for data in each
    % OFDM symbol
    recell = coder.nullcopy(cell(1,symbperslot));
    for i = 1:numel(symbolset)
        recell{symbolset(i)+1} = (0:11)';
    end
    numDMRSSymbols = numel(dmrssymbols);
    for i = 1:numDMRSSymbols
        recell{dmrssymbols(i) + 1} = pdschre;
    end

    % Combine PRB oriented and RE per PRB oriented index arrays and expand
    % into a column of linear indices for a single antenna/layer
    slotindices = [];
    for i = 1:numel(symbolset)
        symI = symbolset(i);
        if isempty(prbcell{symI+1})
            slotindices = [slotindices; zeros(0,1)]; %#ok<AGROW>
        else
            slotindices = [slotindices; reshape(repmat(recell{symI+1},1,numel(prbcell{symI+1})) + repmat(nRBSC*(prbcell{symI+1}+nStartBWP-nStartGrid),numel(recell{symI+1}),1) + nRBSC*nSizeGrid*symI,[],1)]; %#ok<AGROW>
        end
    end

    % Get the reserved resource elements
    if ~isempty(pdsch.ReservedRE)
        % Map the reserved resource elements to the BWP and project it to
        % one layer
        grid = zeros([nSizeBWP*nRBSC symbperslot nports]);
        grid(mod(pdsch.ReservedRE(:), numel(grid))+1) = 1;
        sgrid = sum(grid,3);
        % Assign the BWP projection to carrier grid of one layer
        cgrid = zeros([nSizeGrid*nRBSC symbperslot 1]);
        bwpOffset = nStartBWP-nStartGrid;
        cgrid(bwpOffset*nRBSC+1:(bwpOffset+nSizeBWP)*nRBSC,:,1) = sgrid;
        % Reserved resource elements with carrier orientation
        reservedREOneLayer = find(cgrid ~= 0);
    else
        reservedREOneLayer = zeros(0,1);
    end

    % PT-RS resource elements
    if pdsch.EnablePTRS
        % PT-RS OFDM symbol locations
        ptrssymbols = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbols,double(pdsch.PTRS.TimeDensity));

        % PT-RS subcarrier locations
        if isempty(pdsch.PTRS.PTRSPortSet)
            ptrsPorts = min(ports(:));
        else
            ptrsPorts = double(pdsch.PTRS.PTRSPortSet);
        end
        ptrsREOffset = pdsch.PTRS.REOffset;
        kPTRS = double(pdsch.PTRS.FrequencyDensity);
        ind = nr5g.internal.pxsch.ptrsSubcarrierIndicesCPOFDM(prbset,pdsch.DMRS.DMRSConfigurationType,kPTRS,ptrsREOffset,ptrsPorts,pdsch.RNTI);

        % Map the PT-RS indices to the carrier grid of one layer/port and
        % extract linear indices with respect to carrier
        grid = zeros([nSizeGrid*nRBSC symbperslot 1]);
        grid(ind{1}+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(:)+1) = 1;
        ptrsIndMapOneLayer = find(grid == 1);
    else
        ptrssymbols = zeros(1,0);
        ptrsIndMapOneLayer = zeros(0,1);
    end

    % Remove the reserved resource elements and PT-RS resource elements
    allReservedREOneLayer = [ptrsIndMapOneLayer; reservedREOneLayer];
    if ~isempty(allReservedREOneLayer)
        grid = zeros([nSizeGrid*nRBSC symbperslot 1]);
        grid(slotindices+1) = 2;
        grid(allReservedREOneLayer) = grid(allReservedREOneLayer) + 1;
        if pdsch.VRBToPRBInterleaving
            slotIndices = slotindices(ismember(slotindices(:)+1,find(grid == 2)));
        else
            slotIndices = find(grid == 2)-1;
        end
    else
        slotIndices = slotindices;
    end

    % Expand slotIndices over all ports (0-based)
    Gd = numel(slotIndices);
    ind = repmat(slotIndices(:),1,nports) + repmat((nRBSC*symbperslot*nSizeGrid)*(0:nports-1),Gd,1);

    % Number of layers and modulation order for each codeword
    ncw = 1 + (nports > 4);                  % Number of codewords, deduced from total layers
    nlayers = fix((nports + (0:ncw-1))/ncw); % Number of layers per codeword
    qm = ones(1,ncw);
    if iscell(pdsch.Modulation)
        qm(1) = nr5g.internal.getQm(pdsch.Modulation{1});
        if ncw == 2 && numel(pdsch.Modulation) == 2
            qm(end) = nr5g.internal.getQm(pdsch.Modulation{2});
        else
            qm(end) = qm(1);
        end
    else
        qm(1) =  nr5g.internal.getQm(pdsch.Modulation);
        qm(end) = qm(1);
    end

    % Combine information into a structure
    info.G = Gd*qm.*nlayers;
    info.Gd = Gd;
    info.NREPerPRB = nRBSC*(length(symbolset)-numDMRSSymbols) + length(pdschre)*numDMRSSymbols;
    info.DMRSSymbolSet = dmrssymbols;
    info.PTRSSymbolSet = ptrssymbols;
    info.PRBSet = sort(prbset(:)).';

    % Apply options
    if nargin > 2
        fcnName = 'nrPDSCHIndices';
        opts = nr5g.internal.parseOptions(fcnName,{'IndexStyle','IndexBase','IndexOrientation','MultiColumnIndex'},varargin{:},'MultiColumnIndex',true);
        
        % Apply PV pairs for non-empty output only
        ind = nr5g.internal.applyIndicesOptions([nSizeGrid*nRBSC symbperslot nports],opts,ind(:));
        if pdsch.VRBToPRBInterleaving && strcmp(opts.IndexOrientation,'bwp')...
                && strcmp(opts.IndexStyle,'index') && ~isempty(ind)
            base = double(strcmp(opts.IndexBase,'0based'));
            [k,l,p] = ind2sub([nSizeGrid*nRBSC symbperslot nports],ind+base);
            k = k - nRBSC*(nStartBWP-nStartGrid);
            bwpGridSize = [nSizeBWP*nRBSC symbperslot nports];
            ind = uint32(bwpGridSize(1)*bwpGridSize(2)*(p-1) + bwpGridSize(1)*(l-1)+k-base);
        else
            ind = nr5g.internal.applyIndexOrientation([nStartGrid nSizeGrid], [nStartBWP nSizeBWP],symbperslot,opts,ind);
        end
    else
        % 1-based
        ind = uint32(ind + 1);
    end
end