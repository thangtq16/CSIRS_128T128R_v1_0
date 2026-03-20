function [ind,info,ptrsInd] = nrPUSCHIndices(carrier,pusch,varargin)
%nrPUSCHIndices Physical uplink shared channel resource element indices
%   [IND,INFO,PTRSIND] = nrPUSCHIndices(CARRIER,PUSCH) returns the matrix
%   IND containing 1-based physical uplink shared channel resource element
%   (RE) indices within the carrier resource grid, in linear form. The
%   matrix IND is obtained from TS 38.211 Sections 6.3.1.6 and 6.3.1.7, for
%   the given carrier configuration CARRIER, and uplink shared channel
%   configuration PUSCH. The number of columns in IND is equal to the
%   number of antenna ports configured. In case of transform precoding
%   enabled, the output IND contains the combined locations of data and
%   PT-RS. This function signature also provides the structural information
%   INFO and the phase tracking reference signal (PT-RS) RE indices,
%   PTRSIND. INFO contains the bit capacity, symbol capacity, demodulation
%   reference signal (DM-RS) OFDM symbol locations, and PT-RS OFDM symbol
%   locations associated with the physical uplink shared channel. PTRSIND
%   is a matrix of PT-RS REs within the carrier resource grid. In case of
%   transform precoding enabled, the output PTRSIND represent the
%   projections of PT-RS locations prior to transform precoding on to the
%   carrier resource grid.
%
%   CARRIER is a carrier configuration object, as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with properties:
%
%   SubcarrierSpacing   - Subcarrier spacing in kHz
%                         (15 (default), 30, 60, 120, 240, 480, 960)
%   CyclicPrefix        - Cyclic prefix ('normal' (default), 'extended')
%   NSizeGrid           - Number of resource blocks in carrier resource
%                         grid (1...275) (default 52)
%   NStartGrid          - Start of carrier resource grid relative to common
%                         resource block 0 (CRB 0) (0...2199) (default 0)
%   NSlot               - Slot number (default 0)
%   IntraCellGuardBands - Intracell guard bands (default [])
%
%   PUSCH is the physical uplink shared channel configuration object, as
%   described in <a href="matlab:help('nrPUSCHConfig')">nrPUSCHConfig</a> with properties:
%
%   NSizeBWP           - Size of the bandwidth part (BWP) in terms
%                        of number of physical resource blocks (PRBs)
%                        (1...275) (default []). The default value implies
%                        the value is equal to the size of carrier resource
%                        grid
%   NStartBWP          - Starting PRB index of BWP relative to CRB 0
%                        (0...2473) (default []). The default value implies
%                        the value is equal to the start of carrier
%                        resource grid
%   Modulation         - Modulation scheme(s) of codeword(s)
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
%   TransmissionScheme - Transmission scheme of physical uplink shared
%                        channel ('nonCodebook' (default), 'codebook')
%   NumAntennaPorts    - Number of antenna ports (1 (default), 2, 4)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - PRB start for second hop relative to the BWP
%                        (0-based) (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
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
%   The output structure INFO contains the following fields:
%   G             - Bit capacity of the PUSCH. This must be the
%                   length of codeword from the UL-SCH transport channel
%   Gd            - Number of resource elements per layer/port
%   DMRSSymbolSet - The OFDM symbol locations in a slot containing DM-RS
%                   (0-based)
%   NREPerPRB     - Number of RE per PRB allocated to PUSCH (not
%                   accounting for any reserved resources)
%   PTRSSymbolSet - The OFDM symbol locations in a slot containing PT-RS
%                   (0-based)
%   PRBSet        - PRBs allocated for PUSCH within the BWP
%
%   [IND,INFO,PTRSIND] = nrPUSCHIndices(CARRIER,PUSCH,NAME,VALUE,...)
%   specifies additional options as NAME,VALUE pairs to allow control over
%   the format of the output indices IND and PTRSIND:
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
%   For operation with shared spectrum channel access for FR1, set
%   Interlacing = true and specify the allocated frequency resources using
%   the RBSetIndex and InterlaceIndex properties of the PUSCH
%   configuration. The PRBSet, FrequencyHopping, and SecondHopStartPRB
%   properties are ignored.
%
%   Example 1:
%   % Generate the data symbol indices (0-based) in linear index form of a
%   % physical uplink shared channel occupying the 10 MHz bandwidth for a
%   % 15 kHz subcarrier spacing (SCS) carrier. Configure DM-RS with length
%   % set to 1, type A position set to 2, number of additional positions
%   % set to 0, number of CDM groups without data set to 2, and
%   % configuration type set to 1.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pusch = nrPUSCHConfig;
%   pusch.DMRS.DMRSLength = 1;
%   pusch.DMRS.DMRSTypeAPosition = 2;
%   pusch.DMRS.DMRSAdditionalPosition = 0;
%   pusch.DMRS.NumCDMGroupsWithoutData = 2;
%   pusch.DMRS.DMRSConfigurationType = 1;
%   ind = nrPUSCHIndices(carrier,pusch,'IndexBase','0based');
%
%   Example 2:
%   % Generate the UL-SCH modulated symbol indices of a physical uplink
%   % shared channel with transform precoding enabled, transmission
%   % scheme set to codebook, and number of antenna ports set to 4.
%
%   carrier = nrCarrierConfig;
%   pusch = nrPUSCHConfig('TransformPrecoding',1);
%   pusch.TransmissionScheme = 'codebook';
%   pusch.NumAntennaPorts = 4;
%   ind = nrPUSCHIndices(carrier,pusch);
%
%   Example 3:
%   % Generate and plot the data symbol indices of a physical uplink shared
%   % channel occupying first 5 resource blocks of a 30 kHz SCS carrier
%   % having 5 MHz transmission bandwidth. Enable intra-slot frequency
%   % hopping with starting resource block of second hop at 4. Enable PT-RS
%   % with time density set to 2, and frequency density set to 4.
%
%   % Configure carrier with 30 kHz subcarrier spacing and 5 MHz bandwidth
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 30;
%   carrier.NSizeGrid = 11; % 5 MHz bandwidth
%
%   % Configure PUSCH
%   pusch = nrPUSCHConfig;
%   pusch.PRBSet = 0:4;
%   pusch.FrequencyHopping = 'intraSlot';
%   pusch.SecondHopStartPRB = 4;
%   pusch.EnablePTRS = 1;
%   pusch.PTRS.TimeDensity = 2;
%   pusch.PTRS.FrequencyDensity = 4;
%
%   % Get the resource element indices
%   ind = nrPUSCHIndices(carrier,pusch);
%
%   % Plot the indices on the grid
%   grid = complex(zeros([carrier.NSizeGrid*12 carrier.SymbolsPerSlot pusch.NumLayers]));
%   grid(ind) = 1;
%   imagesc(abs(grid(:,:,1)));
%   axis xy;
%   xlabel('OFDM symbols');
%   ylabel('Subcarriers');
%   title('PUSCH resource elements in the carrier resource grid');
%
%   See also nrPUSCH, nrPUSCHDecode, nrPUSCHDMRSIndices,
%   nrPUSCHPTRSIndices, nrPUSCHConfig, nrPUSCHDMRSConfig,
%   nrPUSCHPTRSConfig, nrCarrierConfig, nrIntraCellGuardBandsConfig.

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate inputs
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot,freqHopping] = nr5g.internal.pusch.validateInputs(carrier,pusch);

    % Assign the structure ftable to pass into the initializeResources
    % internal function
    ftable.ChannelName = 'PUSCH';
    ftable.MappingTypeB = strcmpi(pusch.MappingType,'B');
    ftable.DMRSSymbolSet = @nr5g.internal.pusch.lookupPUSCHDMRSSymbols;
    ftable.IntraSlotFreqHoppingFlag = strcmpi(freqHopping,'intraSlot');

    % Relative slot number
    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    % Get prbset, symbolset and dmrssymbols
    [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pusch,nSizeBWP,ftable);
    nsymbols = numel(symbolset);
    nRBSC = 12;

    % Assign the PRB set for each OFDM symbol in a cell array based on the
    % frequency hopping configuration
    prbcell = cell(1,symbperslot);
    for i = 1:symbperslot
        prbcell{i} = zeros(1,0);
    end
    if nsymbols
        startSym = min(symbolset);
    else
        startSym = 0;
    end
    prbsetHop = nr5g.internal.prbSetTwoHops(...
        prbset,freqHopping,pusch.SecondHopStartPRB,nslot);
    if strcmpi(freqHopping,'intraSlot')
        fhoplen = fix(nsymbols/2);
        for i = 1:fhoplen
            prbcell{i+startSym} = prbsetHop(1,:);
        end
        for i = fhoplen+1:nsymbols
            prbcell{i+startSym} = prbsetHop(2,:);
        end
    else % 'interSlot' or 'neither'
        for i = 1:nsymbols
            prbcell{i+startSym} = prbsetHop(1,:);
        end
    end

    % Capture set of antenna ports required, number of ports and CDM groups
    % without data
    if isempty(pusch.DMRS.DMRSPortSet)
        layers = 0:double(pusch.NumLayers)-1;
    else
        layers = double(pusch.DMRS.DMRSPortSet);
    end
    nlayers = numel(layers);
    if strcmpi(pusch.TransmissionScheme,'nonCodebook')
        nports = nlayers;
    else
        nports = double(pusch.NumAntennaPorts);
    end
    cdmgroupsnodata = double(pusch.DMRS.NumCDMGroupsWithoutData);

    % DM-RS subcarrier (SC) locations in a resource block
    if pusch.DMRS.DMRSConfigurationType==1
        % Type 1: 6 DM-RS SC per PRB per CDM (every other SC)
        dmrssc = [0 2 4 6 8 10]';                   % RE indices in a PRB
        dshiftsnodata = 0:min(cdmgroupsnodata,2)-1; % Delta shifts for CDM groups without data
    else
        % Type 2: 4 DM-RS SC per PRB per CDM (2 groups of 2 SC)
        dmrssc = [0 1 6 7]';                            % RE indices in a PRB
        dshiftsnodata = 2*(0:min(cdmgroupsnodata,3)-1); % Delta shifts for CDM groups without data
    end
    dshifts = pusch.DMRS.DeltaShifts;

    % Non DM-RS resource elements in a DM-RS containing symbol
    fullprb = ones(nRBSC,1);     % Binary map of all the subcarriers in an RB
    dshiftsComp = [dshifts dshiftsnodata];
    dmrsre = repmat(dmrssc,1,numel(dshiftsComp)) + repmat(dshiftsComp, numel(dmrssc),1);
    fullprb(dmrsre+1) = 0;       % Clear all RE which will carry DM-RS in at least one port
    puschre = find(fullprb)-1;   % Find PUSCH (non DM-RS) RE in a DM-RS containing symbol

    % Assign the resource elements of a single RB used for data in each
    % OFDM symbol
    recell = coder.nullcopy(cell(1,symbperslot));
    for i = 1:numel(symbolset)
        recell{symbolset(i)+1} = (0:11)';
    end
    for i = 1:numel(dmrssymbols)
        recell{dmrssymbols(i) + 1} = puschre;
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

    % PT-RS resource elements
    slotIndices = slotindices;
    nPTRS = 0;
    if pusch.EnablePTRS && ~isempty(dmrssymbols) && ~isempty(prbset)
        % PT-RS OFDM symbol locations
        ptrssymbols = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbols,double(pusch.PTRS.TimeDensity));

        if ~pusch.TransformPrecoding
            % PT-RS subcarrier locations
            if isempty(pusch.PTRS.PTRSPortSet)
                ptrsPorts = min(layers(:));
            else
                ptrsPorts = double(pusch.PTRS.PTRSPortSet);
            end
            ptrsREOffset = pusch.PTRS.REOffset;
            kPTRS = double(pusch.PTRS.FrequencyDensity);
            fhop = ~strcmpi(freqHopping,'neither');
            ind = nr5g.internal.pxsch.ptrsSubcarrierIndicesCPOFDM(prbset,pusch.DMRS.DMRSConfigurationType,kPTRS,ptrsREOffset,ptrsPorts,pusch.RNTI,fhop,double(pusch.SecondHopStartPRB));

            % Find the linear indices of data symbols for one layer/port,
            % by marking the locations of PT-RS and data symbols in the
            % resource grid
            grid = zeros([nSizeGrid*nRBSC symbperslot 1]);
            grid(slotindices+1) = 2;
            for p = 1:size(ind,2)
                if ~fhop
                    % FrequencyHopping set to 'neither'
                    grid(ind{p}(:,1)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(:)+1) = 1;
                elseif strcmpi(freqHopping,'interSlot')
                    % FrequencyHopping set to 'interSlot'
                    if mod(nslot,2) == 1
                        colIndex = 2;
                    else
                        colIndex = 1;
                    end
                    grid(ind{p}(:,colIndex)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(:)+1) = 1;
                else
                    % FrequencyHopping set to 'intraSlot'
                    dmrsIndex = dmrssymbols >= (floor(pusch.SymbolAllocation(end)/2)+pusch.SymbolAllocation(1));
                    index = ptrssymbols >= (floor(pusch.SymbolAllocation(end)/2)+pusch.SymbolAllocation(1));
                    if ~isempty(dmrssymbols(~dmrsIndex))
                        grid(ind{p}(:,1)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(~index)+1) = 1;
                    end
                    if ~isempty(dmrssymbols(dmrsIndex))
                        grid(ind{p}(:,2)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(index)+1) = 1;
                    end
                end
            end
            slotIndices = find(grid == 2)-1;
        else
            % DFT-s-OFDM
            % Reduce only the symbol capacity by PT-RS and use all the
            % indices for data
            symInd = nr5g.internal.pusch.ptrsSymIndicesDFTsOFDM(symbolset,dmrssymbols,ptrssymbols);
            scInd = nr5g.internal.pusch.ptrsSCIndicesDFTsOFDM(double(pusch.PTRS.NumPTRSSamples),double(pusch.PTRS.NumPTRSGroups),numel(prbset)*12);
            nPTRS = numel(symInd)*numel(scInd);
        end
    else
        ptrssymbols = zeros(1,0);
    end

    % Expand slotIndices over all ports (0-based)
    ind = repmat(slotIndices(:),1,nports) + repmat((nRBSC*symbperslot*nSizeGrid)*(0:nports-1),numel(slotIndices),1);

    % Number of layers and modulation order for each codeword
    ncw = 1 + (nlayers > 4);                      % Number of codewords, deduced from total layers
    layersPerCW = fix((nlayers + (0:ncw-1))/ncw); % Number of layers per codeword
    qm = ones(1,ncw);
    if iscell(pusch.Modulation)
        qm(1) = nr5g.internal.getQm(pusch.Modulation{1});
        if ncw == 2 && numel(pusch.Modulation) == 2
            qm(end) = nr5g.internal.getQm(pusch.Modulation{2});
        else
            qm(end) = qm(1);
        end
    else
        qm(1) =  nr5g.internal.getQm(pusch.Modulation);
        qm(end) = qm(1);
    end

    % Combine information into a structure
    GdPUSCH = numel(slotIndices)-nPTRS;
    info.G = GdPUSCH*qm.*layersPerCW;
    info.Gd = GdPUSCH;
    info.NREPerPRB = nRBSC*(length(symbolset)-length(dmrssymbols)) + length(puschre)*length(dmrssymbols);
    info.DMRSSymbolSet = dmrssymbols;
    info.PTRSSymbolSet = ptrssymbols;
    info.PRBSet = prbset(:).';

    % Generate PT-RS indices (1-based)
    if nargout == 3
        temp = nrPUSCHPTRSIndices(carrier,pusch);
        if pusch.TransformPrecoding
            ptrsInd = uint32(ind(temp(:,1),:) + 1);
        else
            ptrsInd = temp;
        end
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPUSCHIndices';
        opts = nr5g.internal.parseOptions(fcnName,...
            {'IndexStyle','IndexBase','IndexOrientation','MultiColumnIndex'},varargin{:},'MultiColumnIndex',true);

        % Apply PV pairs to PUSCH indices output
        ind = nr5g.internal.applyIndicesOptions(...
            [nSizeGrid*nRBSC symbperslot nports],opts,ind(:));
        ind = nr5g.internal.applyIndexOrientation(...
            [nStartGrid nSizeGrid],[nStartBWP nSizeBWP],symbperslot,opts,ind);

        % Apply PV pairs to PT-RS indices output
        if nargout == 3
            % Set MultiColumnIndex to false, since PT-RS is not present in all ports
            opts = nr5g.internal.parseOptions(fcnName,...
                {'IndexStyle','IndexBase','IndexOrientation','MultiColumnIndex'},varargin{:},'MultiColumnIndex',false); 
            ptrsIndSize = size(ptrsInd);
            ptrsInd = nr5g.internal.applyIndicesOptions(...
                [nSizeGrid*nRBSC symbperslot nports],opts,ptrsInd(:)-1); % Convert to 0-based
            ptrsInd = nr5g.internal.applyIndexOrientation(...
                [nStartGrid nSizeGrid],[nStartBWP nSizeBWP],symbperslot,opts,ptrsInd);
            if strcmpi(opts.IndexStyle,'index')
                % Reshape the output for linear indexing
                if isempty(ptrsInd)
                    % This statement and code is needed for code generation
                    % as reshape cannot handle resizing empty inputs 
                    ptrsInd = repmat(uint32(0),ptrsIndSize);
                else
                    ptrsInd = reshape(ptrsInd,ptrsIndSize);
                end
            end
        end
    else
        % 1-based
        ind = uint32(ind + 1);
    end
end