function ind = nrPRSIndices(carrier,prs,varargin)
%nrPRSIndices PRS resource element indices
%   IND = nrPRSIndices(CARRIER,PRS) returns the positioning reference
%   signal (PRS) resource element indices, IND, as defined in TS 38.211
%   Section 7.4.1.7.3, given the carrier specific configuration object
%   CARRIER and positioning reference signal configuration object PRS. The
%   function also handles the conditions related to the mapping of PRS
%   resources to slots, as defined in TS 38.211 Section 7.4.1.7.4.
%
%   CARRIER is a carrier specific configuration object, as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>. Only these object properties are relevant for this
%   function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15 (default), 30, 60, 120, 240, 480, 960)
%   CyclicPrefix      - Cyclic prefix ('normal' (default), 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource
%                       grid (1...275) (default 52)
%   NStartGrid        - Start of carrier resource grid relative to common
%                       resource block 0 (CRB 0) (0...2199) (default 0)
%   NSlot             - Absolute slot number (default 0)
%   NFrame            - Absolute system frame number (default 0)
%
%   PRS is a positioning reference signal configuration object to specify
%   one or more PRS resources in a PRS resource set, as described in
%   <a href="matlab:help('nrPRSConfig')">nrPRSConfig</a>. Only these object properties are relevant for this
%   function:
%
%   PRSResourceSetPeriod  - PRS resource set slot periodicity
%                           (TPRSPeriod) and slot offset (TPRSOffset)
%                           ('on' (default), 'off', [TPRSPeriod TPRSOffset])
%   PRSResourceOffset     - Slot offset of each PRS resource relative
%                           to PRS resource set slot offset (TPRSOffset)
%                           (0...511) (default 0)
%   PRSResourceRepetition - PRS resource repetition factor
%                           (1 (default), 2, 4, 6, 8, 16, 32)
%   PRSResourceTimeGap    - Slot offset between two consecutive repeated
%                           instances of a PRS resource (1 (default), 2, 4, 8, 16, 32)
%   MutingPattern1        - Muting bit pattern option-1 (default [])
%   MutingBitRepetition   - Number of consecutive instances of a PRS
%                           resource set corresponding to a single element
%                           of MutingPattern1 binary vector (1 (default), 2, 4, 8)
%   MutingPattern2        - Muting bit pattern option-2 (default [])
%   NumPRSSymbols         - Number of OFDM symbols allocated for each
%                           PRS resource (0...12) (default 12)
%   SymbolStart           - Starting OFDM symbol of each PRS resource
%                           in a slot (0...13) (default 0)
%   NumRB                 - Number of physical resource blocks (PRBs)
%                           allocated for all PRS resources (0...275)
%                           (default 52)
%   RBOffset              - Starting PRB index of all PRS resources
%                           relative to the carrier resource grid
%                           (0...274) (default 0)
%   CombSize              - Comb size of all PRS resources
%                           (2 (default), 4, 6, 12)
%   REOffset              - Starting resource element (RE) offset in
%                           the first OFDM symbol of each PRS resource
%                           (0...CombSize-1) (default 0)
%
%   Note that the following five properties can be specified as scalars or
%   vectors, which are unique to each PRS resource in a PRS resource set:
%   1. PRSResourceOffset
%   2. NumPRSSymbols
%   3. SymbolStart
%   4. REOffset
%   5. NPRSID
%   The number of configured PRS resources is considered as the maximum of
%   lengths of above five mentioned properties. For the above five
%   properties, when the value is specified as a vector, the length must be
%   equal to the number of configured PRS resources. When the property is
%   specified as a scalar, the same value is used for all the PRS resources
%   in a PRS resource set.
%
%   IND = nrPRSIndices(...,NAME,VALUE,...) specifies additional options as
%   NAME,VALUE pairs to allow control over the format of the indices:
%
%   'IndexStyle'           - 'index' for linear indices (default)
%                            'subscript' for [subcarrier, symbol, antenna]
%                            subscript row form
%
%   'IndexBase'            - '1based' for 1-based indices (default)
%                            '0based' for 0-based indices
%
%   'OutputResourceFormat' - 'concatenated' for output of all PRS
%                            resources concatenated into a single column (default)
%                            'cell' for cell array output with each cell
%                            corresponding to an individual PRS resource
%
%   % Example 1:
%   % Generate PRS indices for the default configurations generated by
%   % nrCarrierConfig and nrPRSConfig.
%
%   carrier = nrCarrierConfig;
%   prs = nrPRSConfig;
%   ind = nrPRSIndices(carrier,prs);
%
%   % Example 2:
%   % Generate 0-based PRS indices in subscript style for the default
%   % nrCarrierConfig and nrPRSConfig objects.
%
%   carrier = nrCarrierConfig;
%   prs = nrPRSConfig;
%   ind = nrPRSIndices(carrier,prs,'IndexStyle','subscript','IndexBase','0based');
%
%   % Example 3:
%   % Generate RE indices and symbols for two PRS resources in a resource
%   % set. Map the symbols to the carrier resource grid spanning 20 slots.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%
%   % Set PRS parameters
%   prs = nrPRSConfig;
%
%   % Set the properties related to PRS slot configuration
%   prs.PRSResourceSetPeriod = [8 0]; % Resource set periodicity of 8 slots
%                                     % and resource set slot offset of 0 slots
%   prs.PRSResourceOffset = [0 4];    % Configure two PRS resources with
%                                     % slot offsets 0 and 4 relative to
%                                     % resource set offset
%   prs.PRSResourceRepetition = 2;    % Repeat each PRS resource twice
%   prs.PRSResourceTimeGap = 1;       % Configure two PRS resource
%                                     % repetition indices with no gap
%
%   % Set the properties related to PRS muting configuration
%   prs.MutingPattern1 = [1 1];  % Transmit all PRS resource set instances
%   prs.MutingBitRepetition = 1; % One instance of a PRS resource set
%                                % corresponding to a single element of
%                                % MutingPattern1 binary vector
%   prs.MutingPattern2 = [1 0];  % Mute second repetition index of all the
%                                % PRS resources within an active instance
%                                % of a PRS resource set
%
%   % Set the properties related to PRS time-domain allocation
%   prs.NumPRSSymbols = [6 12];
%   prs.SymbolStart = [6 0];
%
%   % Set the properties related to PRS frequency-domain allocation
%   prs.NumRB = 40;
%   prs.RBOffset = 4;
%   prs.CombSize = 4;
%   prs.REOffset = [1 3];
%   % Set PRS sequence identity
%   prs.NPRSID = 5;
%
%   % Get the number of OFDM symbols per slot
%   numSymPerSlot = carrier.SymbolsPerSlot;
%   % Set the number of slots to 20
%   numSlots = 20;
%   grid = complex(zeros(carrier.NSizeGrid*12,carrier.SymbolsPerSlot*numSlots));
%   for slotIdx = 0:numSlots-1
%       carrier.NSlot = slotIdx;
%       indCell = nrPRSIndices(carrier,prs,'OutputResourceFormat','cell');
%       symCell = nrPRS(carrier,prs,'OutputResourceFormat','cell');
%       slotGrid = nrResourceGrid(carrier);
%       slotGrid(indCell{1}) = 70*symCell{1};  % Resource element mapping of PRS resource 1
%                                              % with some scaling for plotting purpose
%       slotGrid(indCell{2}) = 250*symCell{2}; % Resource element mapping of PRS resource 2
%                                              % with some scaling for plotting purpose
%       grid(:,(1:numSymPerSlot) + numSymPerSlot*slotIdx) = slotGrid;
%   end
%   figure()
%   image(abs(grid));
%   axis xy;
%   L = line(ones(2),ones(2),'LineWidth',8);              % Generate lines
%   set(L,{'color'},{[0.18 0.51 0.98];[0.96 0.95 0.11]}); % Set the colors
%   legend('PRS resource 1','PRS resource 2');            % Create legend
%   title('PRS resource elements');
%   xlabel('OFDM symbols');
%   ylabel('Subcarriers');
%
%   See also nrCarrierConfig, nrPRSConfig, nrPRS.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate the inputs and get the slot schedule of all the PRS
    % resources in a resource set
    [prs,PRSPresence,numRes] = nr5g.internal.validateAndSchedulePRS(carrier,prs);

    % Parse options
    opts = nr5g.internal.parseOptions('nrPRSIndices',...
        {'IndexBase','IndexStyle','OutputResourceFormat'},varargin{:});

    % Get the number of resource elements allocated for all the PRS
    % resources within a PRB
    KPRSComb    = prs.CombSize;
    nRBSC       = 12;
    numREsPerRB = nRBSC/KPRSComb;
    mIndPerRB   = (0:numREsPerRB-1)';

    % Get the equivalent CRB numbers (0-based) of the PRBs across which all
    % the PRS resources span
    nStartGrid = double(carrier.NStartGrid);
    nCRB = prs.RBOffset + nStartGrid + (0:prs.NumRB-1);
    m = reshape(mIndPerRB+nCRB*numREsPerRB,[],1);

    % Get the dimensions of carrier slot resource grid
    gridSize = [double(carrier.NSizeGrid)*nRBSC carrier.SymbolsPerSlot 1];

    % Generate PRS indices
    prsInd = coder.nullcopy(cell(1,numRes));
    numPRSIndPerRes = zeros(1,numRes);
    for resIdx = 1:numRes
        % Get the number of OFDM symbols allocated for a PRS resource
        LPRS = prs.NumPRSSymbols(resIdx);

        % Get the relative frequency offset values for all the OFDM symbols
        % allocated for PRS
        kPrime = prs.FrequencyOffsetValues([2 4 6 12] == KPRSComb,(1:LPRS)+1);
        if ~PRSPresence(resIdx) || (LPRS == 0) || isempty(nCRB)
            kSubs = zeros(0,1);
            lSubs = zeros(0,1);
        else
            % Subcarrier indices (relative to Point A)
            kSubs = reshape(m*KPRSComb+mod(prs.REOffset(resIdx)+...
                kPrime,KPRSComb),[],1);
            % Get the subcarrier indices which are relative to the start of
            % carrier
            kSubs = kSubs - (nStartGrid*nRBSC);

            % OFDM symbol indices
            l = prs.SymbolStart(resIdx) + (0:LPRS-1);
            lSubs = reshape(repmat(l,numel(m),1),[],1);

            % Store the number of REs of a PRS resource
            numPRSIndPerRes(resIdx) = numel(kSubs);
        end
        % Apply indices options
        prsInd{resIdx} = nr5g.internal.applyIndicesOptions(gridSize,...
            opts,kSubs,lSubs);
    end

    % Apply OutputResourceFormat option on the generated PRS indices
    if strcmpi(opts.OutputResourceFormat,'cell')
        ind = prsInd;
    else
        ncols = size(prsInd{1},2);
        prsIndexingPerRes = cumsum([0 numPRSIndPerRes]) + 1;
        ind = zeros(prsIndexingPerRes(end)-1,ncols,'uint32');
        for resIdx = 1:numRes
            ind(prsIndexingPerRes(resIdx):prsIndexingPerRes(resIdx+1)-1,:) = prsInd{resIdx};
        end
    end
end
