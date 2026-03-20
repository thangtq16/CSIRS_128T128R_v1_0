function [pdcchIndices,pdcchDMRS,pdcchDMRSIndices] = nrPDCCHResources( ...
        carrier,pdcch,varargin)
%nrPDCCHResources Physical downlink control channel (PDCCH) resources
%   INDICES = nrPDCCHResources(CARRIER,PDCCH) returns the PDCCH resource
%   element indices, INDICES, as per TS 38.211 Section 7.3.2, for the
%   specified carrier-specific configuration object, CARRIER, and
%   PDCCH-specific configuration object, PDCCH. By default, INDICES is a
%   column vector using 1-based, linear indexing, with carrier grid as the
%   reference. Alternative indexing formats can also be used.
%
%   CARRIER is a carrier-specific configuration object, as described in
%   <a href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a> with these applicable properties:
%
%   <a href="matlab:help('nrCarrierConfig/SubcarrierSpacing')"
%   >SubcarrierSpacing</a>     - Subcarrier spacing in kHz
%   <a href="matlab:help('nrCarrierConfig/CyclicPrefix')"
%   >CyclicPrefix</a>          - Cyclic prefix type
%   <a href="matlab:help('nrCarrierConfig/NSlot')"
%   >NSlot</a>                 - Slot number
%   <a href="matlab:help('nrCarrierConfig/NFrame')"
%   >NFrame</a>                - System frame number
%   <a href="matlab:help('nrCarrierConfig/NSizeGrid')"
%   >NSizeGrid</a>             - Size of the carrier resource grid in terms of
%                           number of resource blocks (RBs)
%   <a href="matlab:help('nrCarrierConfig/NStartGrid')"
%   >NStartGrid</a>            - Starting RB index of the carrier resource grid
%                           relative to common resource block 0 (CRB 0)
%
%   PDCCH is a PDCCH-specific configuration object, as described in
%   <a href="matlab:help('nrPDCCHConfig')"
%   >nrPDCCHConfig</a> with these applicable properties:
%
%   <a href="matlab:help('nrPDCCHConfig/NStartBWP')"
%   >NStartBWP</a>             - Starting RB index of the bandwidth part (BWP) 
%                           resource grid relative to CRB 0
%   <a href="matlab:help('nrPDCCHConfig/NSizeBWP')"
%   >NSizeBWP</a>              - Number of resource blocks in BWP
%   <a href="matlab:help('nrPDCCHConfig/CORESET')"
%   >CORESET</a>               - <a href="matlab:help('nrCORESETConfig')"
%   >Control resource set</a> configuration object
%   <a href="matlab:help('nrPDCCHConfig/SearchSpace')"
%   >SearchSpace</a>           - <a href="matlab:help('nrSearchSpaceConfig')"
%   >Search space set</a> configuration object
%   <a href="matlab:help('nrPDCCHConfig/RNTI')"
%   >RNTI</a>                  - Radio network temporary identifier
%   <a href="matlab:help('nrPDCCHConfig/DMRSScramblingID')"
%   >DMRSScramblingID</a>      - PDCCH DM-RS scrambling identity
%   <a href="matlab:help('nrPDCCHConfig/AggregationLevel')"
%   >AggregationLevel</a>      - PDCCH aggregation level {1,2,4,8,16}
%   <a href="matlab:help('nrPDCCHConfig/AllocatedCandidate')"
%   >AllocatedCandidate</a>    - Candidate used for the PDCCH instance
%
%   [INDICES,DMRS,DMRSINDICES] = nrPDCCHResources(CARRIER,PDCCH) also
%   returns PDCCH demodulation reference signal (DM-RS) symbols, DMRS, and
%   PDCCH DM-RS resource element indices, DMRSINDICES, as per TS 38.211
%   Section 7.4.1.3, for the specified configuration objects.
%
%   [___] = nrPDCCHResources(CARRIER,PDCCH,NAME,VALUE,...) specifies
%   additional options as NAME,VALUE pairs to allow control over the format
%   of the indices and DM-RS symbol output datatype:
%
%   'IndexStyle'       - 'index' for linear indices (default)
%                        'subscript' for [subcarrier, symbol, antenna]
%                        subscript row form
%
%   'IndexBase'        - '1based' for 1-based indices (default)
%                        '0based' for 0-based indices
%
%   'IndexOrientation' - 'carrier' for indices referenced to carrier 
%                        grid (default)
%                        'bwp' for indices referenced to BWP grid
%
%   'OutputDataType'   - 'double' for double precision (default)
%                        'single' for single precision
%
%   The outputs INDICES and DMRSINDICES have the same indexing format as
%   specified by the index-formatting Name,Value pairs.
%   OutputDataType only applies to the DM-RS symbol output.
%
%   % Example 1:
%   % Generate PDCCH resource element indices for the default 
%   % configurations of nrCarrierConfig and nrPDCCHConfig and place PDCCH
%   % symbols in the carrier grid.
%
%   carrier = nrCarrierConfig;
%   pdcch = nrPDCCHConfig;
%   dciCW = randi([0 1],864,1);                             % DCI codeword
%   sym = nrPDCCH(dciCW,pdcch.DMRSScramblingID,pdcch.RNTI); % PDCCH symbols
%   ind = nrPDCCHResources(carrier,pdcch);                  % indices
%   cgrid = nrResourceGrid(carrier);
%   cgrid(ind) = sym;
%
%   % Example 2:
%   % Generate PDCCH resource element indices, DM-RS symbols and indices
%   % for an interleaved CORESET with a duration of 3 symbols, for an
%   % aggregation level of 16.
%
%   carrier = nrCarrierConfig;
%   pdcch = nrPDCCHConfig;
%   pdcch.NStartBWP = 6;
%   pdcch.NSizeBWP = 36;
%   pdcch.CORESET.FrequencyResources = ones(1,6);
%   pdcch.CORESET.Duration = 3;
%   pdcch.CORESET.REGBundleSize = 3;
%   pdcch.AggregationLevel = 16;
%
%   [ind,dmrs,dmrsInd] = nrPDCCHResources(carrier,pdcch);
%
%   % Example 3:
%   % Generate PDCCH resource element indices and DM-RS symbol indices
%   % using 1-based, subscript indexing form referenced to the BWP grid.
%
%   carrier = nrCarrierConfig;
%   carrier.NStartGrid = 3;
%   carrier.NSizeGrid = 60;
%   pdcch = nrPDCCHConfig;
%   pdcch.NStartBWP = 5;
%   pdcch.NSizeBWP = 48;
%   pdcch.CORESET.FrequencyResources = ones(1,6);
%   pdcch.CORESET.Duration = 3;
%   pdcch.CORESET.CCEREGMapping = 'noninterleaved';
%   pdcch.AggregationLevel = 16;
%
%   [ind,~,dmrsInd] = nrPDCCHResources(carrier,pdcch, ...
%       'IndexOrientation','bwp','IndexStyle','subscript');
%
%   See also nrPDCCHConfig, nrSearchSpaceConfig, nrCORESETConfig, nrPDCCH,
%   nrCarrierConfig.

%   Copyright 2019-2022 The MathWorks, Inc.

%   References:
%   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical channels and
%   modulation. Sections 7.3.2, 7.4.1.3.
%   [2] 3GPP TS 38.213, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical layer
%   procedures for control. Sections 10, 13.
%   [3] 3GPP TS 38.331, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Radio Resource
%   Control (RRC) protocol specification. Section 6.3.2,
%   SearchSpace IE.

%#codegen

    % Validate and parse inputs
    fcnName = 'nrPDCCHResources';
    narginchk(2,10);
    opts = nr5g.internal.parseOptions(fcnName, ...
        {'IndexBase','IndexStyle','OutputDataType', ...
         'IndexOrientation'},varargin{:});
    dType = opts.OutputDataType;
    
    nr5g.internal.pdcch.validateInputs(carrier,pdcch,fcnName);
    
    % Check for occasion and get CCE indices for PDCCH instance
    [isOccasion,cceIndices] = getInstance(carrier,pdcch);

    if isOccasion
        % Extract CORESET parameter object
        crst = pdcch.CORESET;

        % Get REG bundles and interleaving pattern for entire CORESET
        % The CORESET REG bundles are returned as columns of 2,3,6 REG/RB 1-based indices
        % Note this operation will also account for any BWP/carrier index orientation defined in the opts
        [regBundles,f,coresetRBIdx] = nr5g.internal.pdcch.getREGBundles(carrier,pdcch,opts);

        % Turn the candidate CCE indices into associated RB indices, accounting for any interleaving
        rbCCEIdx = zeros(6,pdcch.AggregationLevel,'uint32');
        if strcmpi(crst.CCEREGMapping,'interleaved')
            regBInt = regBundles(:,f+1);          % Interleave them
            cce = reshape(regBInt,6,[]);          % Form CCEs (column groups of 6 REG/RB)
            rbCCEIdx(:,:) = cce(:,cceIndices+1);  % Select CCEs of the PDCCH instance
        else
            % nonInterleaved: regB == CCE
            cce = reshape(regBundles,6,[]);       % Form CCEs (column groups of 6 REG/RB)
            rbCCEIdx(:,:) = cce(:,cceIndices+1);  % Select CCEs of the PDCCH instance
        end

        % Repool the RBs, per symbol in slot, for frequency first mapping
        % Each column out contains the PDCCH PRB indices for the associated ODFM symbol (index value are still relative to overall resource grid)
        % Indices are 1-based RB at this point, wrt carrier grid, and sorted in (freq) ascending, RE mapping order
        pdcchRBIdx = sort(reshape(rbCCEIdx,crst.Duration,[])'); %#ok

        % Establish the indices of the RB that will contain associated PDCCH DM-RS
        if strcmpi(pdcch.CORESET.PrecoderGranularity,'allContiguousRBs')
            dmrsRBIdx = uint32(coresetRBIdx);
        else
            dmrsRBIdx = pdcchRBIdx;
        end

        % Specify the resource element (RE) level indices from PRB indices
        symIdxPerRB = uint32([0 2:4 6:8 10:11]') + 1; % Excluding DM-RS, 1-based column (note the +1 part)
        dmrsIdxPerRB = uint32([1   5   9]') + 1;      % DM-RS, 1-based column (note the +1 part)
        
        % Expand each RB index with its RE offset indices (1-based), using implicit expansion addition (frequency-first offset within an RB)
        tmpPDCCHIndices     = reshape(12*uint32(reshape(pdcchRBIdx,1,[])-1)+symIdxPerRB,[],1);
        tmpPDCCHDMRSIndices = reshape(12*uint32(reshape(dmrsRBIdx,1,[])-1)+dmrsIdxPerRB,[],1);

        % Compute DM-RS symbols
        pdcchDMRS = nr5g.internal.pdcch.dmrsSymbols(carrier,pdcch,dmrsRBIdx,opts);

        % Both PDCCH and DM-RS indices are currently in 'index', 1-based format (already accounting for carrier vs BWP orientation)
        % so adjust if required to be otherwise 
        if strcmpi(opts.IndexStyle,'subscript') || ...
                strcmpi(opts.IndexBase,'0based')
            % Handle other indexing options: 0-based, subscript
            %   via applyIndicesOptions, which takes 0-based inputs
            if strcmpi(opts.IndexStyle,'subscript')
                if strcmpi(opts.IndexOrientation,'carrier')
                    % single antenna only
                    reGridSize = [12*uint32(carrier.NSizeGrid) carrier.SymbolsPerSlot];
                else
                    reGridSize = [12*uint32(pdcch.NSizeBWP) carrier.SymbolsPerSlot];
                end

                pdcchIndices = zeros(size(tmpPDCCHIndices,1),3,'uint32');
                pdcchDMRSIndices = zeros(size(tmpPDCCHDMRSIndices,1),3,'uint32');
                pdcchIndices(:,:) = nr5g.internal.applyIndicesOptions( ...
                    reGridSize,opts,tmpPDCCHIndices-1);
                pdcchDMRSIndices(:,:) = nr5g.internal.applyIndicesOptions( ...
                    reGridSize,opts,tmpPDCCHDMRSIndices-1);
            else % 0-based
                pdcchIndices = tmpPDCCHIndices-1;
                pdcchDMRSIndices = tmpPDCCHDMRSIndices-1;
            end
        else % index, 1-based
            pdcchIndices = tmpPDCCHIndices;
            pdcchDMRSIndices = tmpPDCCHDMRSIndices;
        end

    else % No occasion

        % Return empties
        pdcchDMRS = zeros(0,1,dType);
        if strcmpi(opts.IndexStyle,'subscript')
            pdcchIndices = zeros(0,3,'uint32');
            pdcchDMRSIndices = zeros(0,3,'uint32');
        else
            pdcchIndices = zeros(0,1,'uint32');
            pdcchDMRSIndices = zeros(0,1,'uint32');
        end
    end

end

function [isOccasion,instance] = getInstance(carrier,pdcch)
% Section 10, [2].
%
% For input carrier, pdcch configs, returns the candidate instance (one set
% of CCE indexes per slot, does not cover multiple symbolStarts)
%   0-based frame and slot numbers, for the current NSlot only
%   For monitoring, start from offset for duration in slots
%   Assumes nCI = 0 (carrier indicator field)

    [isOccasion,slotNum] = nr5g.internal.pdcch.isOccasion(carrier,pdcch);

    if isOccasion
        % Get allocated candidate CCE indices only       
        if isprop(pdcch,'CCEOffset') && ~isempty(pdcch.CCEOffset)
            % Explicit CCE specification
            instance = pdcch.CCEOffset(1)+uint32(0:pdcch.AggregationLevel-1)'; 
        else
            % CCE specification in terms of PDCCH candidate number
            instance = getInstanceCCEIndexes(pdcch.SearchSpace,pdcch.CORESET, ...
                double(pdcch.RNTI),slotNum,pdcch.AggregationLevel, ...
                pdcch.AllocatedCandidate);       
        end
    else
        % not monitored, bail out
        instance = zeros(0,1,'uint32');
    end
end

function instance = getInstanceCCEIndexes(ssCfg,crstCfg,rnti,slotNum,L,allocCand)
% For the allocated candidate, return the CCE indexes (L-sized).
% Slot based, not per monitored occasion in a slot (assume only one per SS)

    nCI = 0;   % Assumes nCI = 0 (carrier indicator field)
    numCCEs = double(crstCfg.Duration)*sum(crstCfg.FrequencyResources);

    Yp = nr5g.internal.pdcch.getYp(ssCfg.SearchSpaceType,ssCfg.CORESETID, ...
            rnti,slotNum);

    aggLvls = [1 2 4 8 16];
    MsAL = 1;
    MsAL(:) = ssCfg.NumCandidates(L==aggLvls);
    ms = allocCand-1;

    % CCE indices for the instance
    aL = uint32(L);      % Make the calculated indices of uint32 type
    cceoffset = aL*( mod(Yp + floor(double(ms*numCCEs)/double(aL*MsAL)) + nCI,floor(numCCEs/double(aL))) );
    instance = cceoffset+(0:aL-1)';

end
