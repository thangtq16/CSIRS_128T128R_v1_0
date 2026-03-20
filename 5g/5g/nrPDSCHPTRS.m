function sym = nrPDSCHPTRS(carrier, pdsch, varargin)
%nrPDSCHPTRS Physical downlink shared channel phase tracking reference signal
%   SYM = nrPDSCHPTRS(CARRIER,PDSCH) returns the phase tracking reference
%   signal (PT-RS) symbols, SYM, of physical downlink shared channel for
%   the given carrier configuration object CARRIER and channel transmission
%   configuration object PDSCH according to TS 38.211 Section 7.4.1.2.1.
%
%   CARRIER is a carrier configuration object as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with the following properties:
%
%   NCellID           - Physical layer cell identity (0...1007) (default 1)
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
%       DMRSReferencePoint     - The reference point for the DM-RS
%                                sequence to subcarrier resource mapping
%                                ('CRB0' (default), 'PRB0'). Use 'CRB0', if
%                                the subcarrier reference point for DM-RS
%                                sequence mapping is subcarrier 0 of common
%                                resource block 0 (CRB 0). Use 'PRB0', if
%                                the reference point is subcarrier 0 of the
%                                first PRB of the BWP (PRB 0). The latter
%                                should be used when the PDSCH is signaled
%                                via CORESET 0. In this case the BWP
%                                parameters should also be aligned with
%                                this CORESET
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
%       NIDNSCID               - DM-RS scrambling identity (0...65535)
%                                (default []). Use empty ([]) to set the
%                                value to NCellID
%       NSCID                  - DM-RS scrambling initialization
%                                (0 (default), 1)
%       DMRSDownlinkR16        - Release 16 low PAPR DM-RS 
%                                (0 (default), 1)
%   EnablePTRS            - Enable or disable the PT-RS configuration
%                           (0 (default), 1). The value of 0 implies PT-RS
%                           is disabled and value of 1 implies PT-RS is
%                           enabled
%   PTRS                  - PDSCH-specific PT-RS configuration object, as
%                           described in <a href="matlab:help('nrPDSCHPTRSConfig')">nrPDSCHPTRSConfig</a> with properties:
%       TimeDensity      - PT-RS time density (1 (default), 2, 4)
%       FrequencyDensity - PT-RS frequency density (2 (default), 4)
%       REOffset         - PT-RS resource element offset
%                         ('00' (default), '01', '10', '11')
%       PTRSPortSet      - PT-RS antenna port set (default []). The default
%                          value implies the value is equal to the lowest
%                          DM-RS antenna port configured
%
%   SYM = nrPDSCHPTRS(CARRIER,PDSCH,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the data type of the
%   output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example:
%   % Generate PT-RS symbols of a physical downlink shared channel
%   % occupying the 10 MHz bandwidth for a 15 kHz subcarrier spacing (SCS)
%   % carrier. Configure DM-RS with length set to 1, additional position
%   % set to 0, type A position set to 2, and configuration type set to 1.
%   % Enable PT-RS with time density set to 1, frequency density set to 2,
%   % and resource element offset set to '01'.
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
%   sym = nrPDSCHPTRS(carrier,pdsch);
%
%   See also nrPDSCHPTRSIndices, nrPDSCHDMRS, nrPDSCHConfig,
%   nrPDSCHDMRSConfig, nrPDSCHPTRSConfig, nrCarrierConfig.

%   Copyright 2019-2025 The MathWorks, Inc.

%#codegen

    narginchk(2,4);

    % Validate inputs
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot] = nr5g.internal.pdsch.validateInputs(carrier,pdsch);

    % Get prbset, symbolset and DM-RS symbols
    [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pdsch,nSizeBWP);

    % Cache the DM-RS port set
    nports = double(pdsch.NumLayers);
    if isempty(pdsch.DMRS.DMRSPortSet)
        ports = 0:nports-1;
    else
        ports = double(pdsch.DMRS.DMRSPortSet);
    end

    % PT-RS OFDM symbol set
    ptrssymbols = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbols,double(pdsch.PTRS.TimeDensity));

    % Generate PT-RS
    ptrs = complex(zeros(0,1));
    if ~isempty(ptrssymbols) && pdsch.EnablePTRS && ~isempty(prbset)

        % Remove any reserved PRB
        [prbActive,reservedRBFlag] = nr5g.internal.pdsch.extractActiveResourceBlocks(pdsch.ReservedPRB,prbset,symbolset,carrier.NSlot,carrier.SymbolsPerSlot);

        % Cache the PT-RS port set
        if isempty(pdsch.PTRS.PTRSPortSet)
            ptrsPort = min(ports(:));
        else
            ptrsPort = double(pdsch.PTRS.PTRSPortSet);
        end

        % Subcarrier locations of PT-RS for each OFDM symbol
        [kRefTable,dmrsSCPattern,nDMRSSC] = nr5g.internal.pxsch.ptrsSubcarrierInfo(pdsch.DMRS.DMRSConfigurationType);
        colIndex = strcmpi(pdsch.PTRS.REOffset,{'00','01','10','11'});
        kRERef = kRefTable(ptrsPort(1)+1,colIndex);

        % Get the PRB reference point associated with PRB set values for
        % the DM-RS sequence indexing
        prbrefpoint = nr5g.internal.pdsch.getRBReferencePoint(carrier.NStartGrid,pdsch.NStartBWP,pdsch.DMRS.DMRSReferencePoint);

        % Get the DM-RS base sequence for the PT-RS port
        dmsym = dmrssymbols(1);

        % Codegen compatible cell-array-of-empty-vectors initialisation
        initEmpty = zeros(1,0);
        coder.varsize('initEmpty',[1,Inf],[0,1]);
        prbcell = repmat({initEmpty},1,symbperslot);
        prbcell{dmsym(1)+1} = prbset;
        [symcell,~,port2baseseq] = nr5g.internal.prbsDMRSSequenceSets(carrier,...  % NCellID and NSlot
                                          pdsch.DMRS,...                 % DM-RS config (scrambling ID and CDM groups)
                                          pdsch.DMRS.DMRSDownlinkR16,... % R16 control
                                          prbcell,prbrefpoint,...        % Frequency part (PRB)
                                          dmsym,...                      % Time part (OFDM symbol required)
                                          ptrsPort);                     % Antenna port
        dmrsSym = reshape(symcell{port2baseseq(1)},nDMRSSC,[]);

        % PT-RS subcarrier index position relative to the number of DM-RS
        % symbols in a resource block
        [~,scIndex] = find(repmat(kRERef(1),size(dmrsSCPattern)) == dmrsSCPattern);

        % Loop over number of PT-RS OFDM symbol locations
        kptrs = double(pdsch.PTRS.FrequencyDensity);
        ind = zeros(0,1);
        for i = 1:numel(ptrssymbols)
            % Get PRB locations which contain PT-RS
            ptrsSym = ptrssymbols(i);
            nPXSCHRB = numel(prbset);
            if mod(nPXSCHRB,kptrs) == 0
                kRBRef = mod(double(pdsch.RNTI),kptrs);
            else
                kRBRef = mod(double(pdsch.RNTI),mod(nPXSCHRB,kptrs));
            end
            prbsetPTRS = prbset(kRBRef+1:kptrs(1):end);

            % Get the active set of PRB locations
            if reservedRBFlag
                prbActiveSet = prbActive{ptrsSym+1};
                c = zeros(1,275);
                c(prbActiveSet+1) = 1;
                c(prbsetPTRS+1) = c(prbsetPTRS+1) + 1;
                prbsetActive = find(c==2)-1;
            else
                prbsetActive = prbsetPTRS;
            end
            lm = (repmat(prbset,numel(prbsetActive),1) == repmat(prbsetActive(:),1,numel(prbset)));

            % Get the PT-RS symbols from the active set and record the
            % linear subcarrier indices of PT-RS presence to remove the
            % reserved resource elements
            if ~isempty(prbsetActive)
                ptrs = [ptrs; reshape(dmrsSym(scIndex,sum(lm,1)~=0),[],1)]; %#ok<AGROW>
                ind = [ind; reshape(prbsetActive*12+kRERef(1)+12*(nStartBWP-nStartGrid)+12*nSizeGrid*ptrsSym,[],1)]; %#ok<AGROW>
            end
        end

        % Remove the reserved resource elements
        if ~isempty(pdsch.ReservedRE)
            % Map the reserved resource elements to the BWP and project it
            % to one layer
            nRBSC = 12; % Number of subcarriers in a resource block
            grid = zeros([nSizeBWP*nRBSC symbperslot nports]);
            grid(mod(pdsch.ReservedRE(:), numel(grid))+1) = 1;
            sgridRef = sum(grid,3);
            sgridRef(sgridRef ~=0) = 1;
            % Assign the BWP projection to carrier grid of one layer
            cgrid = zeros([nSizeGrid*nRBSC symbperslot 1]);
            bwpOffset = nStartBWP-nStartGrid;
            cgrid(bwpOffset*nRBSC+1:(bwpOffset+nSizeBWP)*nRBSC,:,1) = sgridRef;
            cgrid(ind+1) = cgrid(ind+1) + 1;
            ptrsInd = find(cgrid == 2); % Indices which occupy the reserved resource elements
            if ~isempty(ptrsInd)
                logicalMatrix = repmat(ind(:,1), 1, numel(ptrsInd)) == repmat(reshape(ptrsInd-1,1,[]),numel(ind(:,1)),1);
                ptrs = ptrs( ~sum(logicalMatrix,2) ,:);
            end
        end
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPDSCHPTRS';
        opts = nr5g.internal.parseOptions(fcnName,{'OutputDataType'},varargin{:});
        sym = cast(ptrs(:),opts.OutputDataType);
    else
        sym = ptrs(:);
    end

end