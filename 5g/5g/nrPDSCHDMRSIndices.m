function ind = nrPDSCHDMRSIndices(carrier,pdsch,varargin)
%nrPDSCHDMRSIndices Physical downlink shared channel DM-RS resource element indices
%   IND = nrPDSCHDMRSIndices(CARRIER,PDSCH) returns the matrix IND
%   containing 1-based demodulation reference signal (DM-RS) resource
%   element (RE) indices of physical downlink shared channel within the
%   carrier resource grid, in linear form. The matrix IND is obtained from
%   TS 38.211 Section 7.4.1.1.2, for the given carrier configuration
%   CARRIER and downlink shared channel configuration PDSCH. The number of
%   columns in IND is equal to the number of antenna ports configured.
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
%
%   IND = nrPDSCHDMRSIndices(CARRIER,PDSCH,NAME,VALUE,...) specifies
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
%   % Generate the DM-RS indices (0-based) in linear index form of a
%   % physical downlink shared channel occupying the 10 MHz bandwidth for a
%   % 15 kHz subcarrier spacing (SCS) carrier. Configure DM-RS with type A
%   % position set to 2, configuration type set to 1, number of additional
%   % positions set to 0, and length set to 1.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pdsch = nrPDSCHConfig;
%   pdsch.DMRS.DMRSLength = 1;
%   pdsch.DMRS.DMRSTypeAPosition = 2;
%   pdsch.DMRS.DMRSAdditionalPosition = 0;
%   pdsch.DMRS.DMRSConfigurationType = 1;
%   ind = nrPDSCHDMRSIndices(carrier,pdsch,'IndexBase','0based');
%
%   See also nrPDSCHDMRS, nrTimingEstimate, nrChannelEstimate,
%   nrPDSCHPTRSIndices, nrPDSCHIndices, nrPDSCHConfig, nrPDSCHDMRSConfig,
%   nrCarrierConfig.

%   Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate inputs
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot] = nr5g.internal.pdsch.validateInputs(carrier,pdsch);

    % Get prbset, symbolset and DM-RS symbols
    [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pdsch,nSizeBWP);
    nRBSC = 12; % Number of subcarriers in a resource block

    % Capture the number of antenna ports
    nports = double(pdsch.NumLayers);

    % Get the DM-RS symbol indices
    ind = zeros(0,nports);
    if ~isempty(dmrssymbols)
        % Remove the reserved resource blocks
        prbcell = nr5g.internal.pdsch.extractActiveResourceBlocks(pdsch.ReservedPRB,prbset,symbolset,carrier.NSlot,carrier.SymbolsPerSlot);

        % DM-RS subcarrier locations in a resource block for each port
        dmrsSubcarrierLocations = pdsch.DMRS.DMRSSubcarrierLocations(:,1:nports);

        % Number of DM-RS resource elements in a resource block
        ndmrssc = size(dmrsSubcarrierLocations,1);

        % Expand the subcarrier locations across all the active resource
        % blocks in a slot for all the ports
        nTotalRE = nRBSC*symbperslot*nSizeGrid; % Number of resource elements in a carrier grid of one layer/port
        for i = 1:numel(dmrssymbols)
            dmSymI = dmrssymbols(i);
            prbActive = reshape(repmat(nRBSC*(prbcell{dmSymI+1}+nStartBWP-nStartGrid),ndmrssc,1),[],1);
            ind = [ind; repmat(dmrsSubcarrierLocations,numel(prbcell{dmSymI+1}),1)...
                + repmat(prbActive,1,nports) + nRBSC*nSizeGrid*dmSymI + repmat(nTotalRE*(0:nports-1),numel(prbActive),1)]; %#ok<AGROW>
        end
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPDSCHDMRSIndices';
        opts = nr5g.internal.parseOptions(fcnName,{'IndexStyle','IndexBase','IndexOrientation','MultiColumnIndex'},varargin{:},'MultiColumnIndex',true);

        % Apply PV pairs
        ind = nr5g.internal.applyIndicesOptions([nSizeGrid*nRBSC symbperslot nports],opts,ind(:));
        ind = nr5g.internal.applyIndexOrientation([nStartGrid nSizeGrid], [nStartBWP nSizeBWP],symbperslot,opts,ind);
    else
        % 1 based, linear indexing
        ind = uint32(ind + 1);
    end
end