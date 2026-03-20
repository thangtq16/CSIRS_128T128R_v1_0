function ind = nrPDSCHPTRSIndices(carrier,pdsch,varargin)
%nrPDSCHPTRSIndices Physical downlink shared channel PT-RS resource element indices
%   IND = nrPDSCHPTRSIndices(CARRIER,PDSCH) returns the column vector IND
%   containing 1-based phase tracking reference signal (PT-RS) resource
%   element (RE) indices of physical downlink shared channel within the
%   carrier resource grid, in linear form. The column vector IND is
%   obtained from TS 38.211 Section 7.4.1.2.2, for the given carrier
%   configuration CARRIER and downlink shared channel configuration PDSCH.
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
%   RNTI                  - Radio network temporary identifier (0...65535)
%                           (default 1)
%   DMRS                  - PDSCH-specific DM-RS configuration object, as
%                           described in <a href="matlab:help('nrPDSCHDMRSConfig')">nrPDSCHDMRSConfig</a> with properties:
%       DMRSConfigurationType  - DM-RS configuration type (1 (default), 2)
%       DMRSTypeAPosition      - Position of first DM-RS OFDM symbol in a
%                                slot (2 (default), 3)
%       DMRSLength             - Number of consecutive DM-RS OFDM symbols
%                                (1 (default), 2)
%       DMRSAdditionalPosition - Maximum number of DM-RS additional
%                                positions (0...3) (default 0)
%       CustomSymbolSet        - Custom DM-RS symbol locations (0-based)
%                                (default []). This property is used to
%                                override the standard defined DM-RS symbol
%                                locations. Each entry corresponds to a
%                                single-symbol DM-RS
%       DMRSPortSet            - DM-RS antenna port set (0...11)
%                                (default []). The default value implies
%                                that the values are in the range from 0 to
%                                NumLayers-1
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
%   IND = nrPDSCHPTRSIndices(CARRIER,PDSCH,NAME,VALUE,...) specifies
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
%   % Generate PT-RS indices of a physical downlink shared channel
%   % occupying the 10 MHz bandwidth for a 15 kHz subcarrier spacing (SCS)
%   % carrier. Configure DM-RS with length set to 1, number of additional
%   % positions set to 0, type A position set to 2, and configuration type
%   % set to 1. Enable PT-RS with time density set to 1, frequency density
%   % set to 2, and resource element offset set to '01'.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pdsch = nrPDSCHConfig;
%   pdsch.DMRS.DMRSLength = 1;
%   pdsch.DMRS.DMRSTypeAPosition = 2;
%   pdsch.DMRS.DMRSAdditionalPosition = 0;
%   pdsch.DMRS.DMRSConfigurationType = 1;
%   pdsch.EnablePTRS = 1;
%   pdsch.PTRS.TimeDensity = 1;
%   pdsch.PTRS.FrequencyDensity = 2;
%   pdsch.PTRS.REOffset = '01';
%   ind = nrPDSCHPTRSIndices(carrier,pdsch);
%
%   See also nrPDSCHPTRS, nrPDSCHDMRSIndices, nrPDSCHIndices,
%   nrPDSCHConfig, nrPDSCHDMRSConfig, nrPDSCHPTRSConfig, nrCarrierConfig.

%   Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate inputs
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot] = nr5g.internal.pdsch.validateInputs(carrier,pdsch);

    % Capture set of antenna ports required
    nports = double(pdsch.NumLayers);
    if isempty(pdsch.DMRS.DMRSPortSet)
        ports = 0:nports-1;
    else
        ports = double(pdsch.DMRS.DMRSPortSet);
    end
    nRBSC = 12; % Number of subcarriers in a resource block

    % Check the presence of PT-RS
    if ~pdsch.EnablePTRS
        % When EnablePTRS is set to 0, return empty output
        ind = zeros(0,1);
    else
        % Get prbset, symbolset and DM-RS symbols
        [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pdsch,nSizeBWP);

        % PT-RS OFDM symbol set
        ptrssymbols = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbols,double(pdsch.PTRS.TimeDensity));

        % Remove the reserved PRB
        [prbActive,reservedRBflag] = nr5g.internal.pdsch.extractActiveResourceBlocks(pdsch.ReservedPRB,prbset,symbolset,carrier.NSlot,carrier.SymbolsPerSlot);

        % PT-RS port set
        if isempty(pdsch.PTRS.PTRSPortSet)
            ptrsPort = min(ports(:));
        else
            ptrsPort = double(pdsch.PTRS.PTRSPortSet(1));
        end

        % Subcarrier locations of PT-RS for each OFDM symbol
        kptrs = double(pdsch.PTRS.FrequencyDensity);
        reOffset = pdsch.PTRS.REOffset;
        subInd = nr5g.internal.pxsch.ptrsSubcarrierIndicesCPOFDM(prbset,pdsch.DMRS.DMRSConfigurationType,kptrs,reOffset,ptrsPort,pdsch.RNTI);

        % Map the PT-RS indices to the carrier grid of one layer/port and
        % extract linear indices with respect to carrier
        grid = zeros([nSizeGrid*nRBSC symbperslot 1]);
        grid(subInd{1}+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(:)+1) = 1;
        ptrsIndTemp = find(grid == 1)-1; % 0-based

        % Find the PT-RS port offset through the combination of DM-RS
        % ports/layers configured
        lm = (ports == repmat(ptrsPort,size(ports)));
        index = find(lm(:));

        % Apply port offset
        portOffset = nRBSC*nSizeGrid*symbperslot*(index-1);
        ind = ptrsIndTemp(:)+portOffset(1); % 0-based

        % Get the reserved resource elements
        if ~isempty(pdsch.ReservedRE)
            % Map the reserved resource elements to the BWP and project it
            % to one layer
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

        % Get the active resource element indices if there are reserved
        % resource blocks
        if reservedRBflag
            % Active subcarrier indices for the allocated RB
            slotindicesActive = [];
            for i = 1:numel(ptrssymbols)
                symI = ptrssymbols(i);
                if isempty(prbActive{symI+1})
                    slotindicesActive = [slotindicesActive; zeros(0,1)]; %#ok<AGROW>
                else
                    slotindicesActive = [slotindicesActive; reshape(repmat((0:nRBSC-1)',1,numel(prbActive{symI+1}))...
                        + repmat(nRBSC*(prbActive{symI+1}+nStartBWP-nStartGrid),nRBSC,1) + nRBSC*nSizeGrid*symI,[],1)]; %#ok<AGROW>
                end
            end
        else
            slotindicesActive = zeros(0,1);
        end

        % Map the active slot indices, PT-RS indices and reserved resource
        % elements to the carrier grid of one layer/port
        cgrid = zeros([nSizeGrid*nRBSC symbperslot 1]);
        cgrid(slotindicesActive+1) = 2;
        cgrid(ptrsIndTemp+1) = cgrid(ptrsIndTemp+1) + 1;
        cgrid(reservedREOneLayer) = cgrid(reservedREOneLayer) + 4;

        % Get the inactive PT-RS resource element indices
        if reservedRBflag
            % Indices which are part of scheduled resource blocks but not
            % active resource blocks, and the overlapped indices with
            % reserved resource elements
            % * Value of 1 indicates PT-RS is part of scheduled resource
            %   blocks
            % * Value of 5 indicates PT-RS is overlapped with reserved RE
            %   in scheduled resource blocks
            % * Value of 7 indicates PT-RS is overlapped with reserved RE
            %   in active resource blocks
            inactivePTRSInd = [find(cgrid == 1); find(cgrid == 5); find(cgrid == 7)];
        else
            % No reserved resource elements at RB level, implies the
            % slotindicesActive is empty
            % Get the PT-RS indices which are overlapped with reserved RE
            inactivePTRSInd = find(cgrid == 5);
        end

        % Get the active PT-RS resource elements
        if ~isempty(inactivePTRSInd)
            logicalMatrix = (repmat(ind(:,1)-portOffset(1),1,numel(inactivePTRSInd)) == repmat(reshape(inactivePTRSInd(:)-1,1,[]),numel(ind(:,1)),1));
            ind = ind( ~sum(logicalMatrix,2) ,:);
        end
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPDSCHPTRSIndices';
        opts = nr5g.internal.parseOptions(fcnName,{'IndexStyle','IndexBase','IndexOrientation'},varargin{:});
        ind = nr5g.internal.applyIndicesOptions([nSizeGrid*nRBSC symbperslot nports],opts,ind(:));
        ind = nr5g.internal.applyIndexOrientation([nStartGrid nSizeGrid], [nStartBWP nSizeBWP],symbperslot,opts,ind);
    else
        % 1 based
        ind = uint32(ind(:) + 1);
    end

end
