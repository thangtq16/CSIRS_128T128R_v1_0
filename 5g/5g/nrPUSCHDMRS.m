function sym = nrPUSCHDMRS(carrier,pusch,varargin)
%nrPUSCHDMRS Physical uplink shared channel demodulation reference signal
%   SYM = nrPUSCHDMRS(CARRIER,PUSCH) returns the demodulation reference
%   signal (DM-RS) symbols, SYM, of physical uplink shared channel for the
%   given carrier configuration CARRIER, and uplink shared channel
%   configuration PUSCH according to TS 38.211 Section 6.4.1.1.1. The
%   output SYM is a matrix with number of columns equal to the number of
%   antenna ports configured.
%
%   CARRIER is a carrier configuration object, as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with properties:
%
%   NCellID             - Physical layer cell identity (0...1007) (default 1)
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
%   TPMI               - Transmitted precoding matrix indicator (0...304)
%                        (default 0)
%   CodebookType       - Codebook type ('codebook1_ng1n4n1' (default),
%                        'codebook1_ng1n2n2', 'codebook2', 'codebook3', 'codebook4')
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - PRB start for second hop relative to the BWP
%                        (0-based) (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   DMRS               - PUSCH-specific DM-RS configuration object, as
%                        described in <a href="matlab:help('nrPUSCHDMRSConfig')">nrPUSCHDMRSConfig</a> with properties:
%       DMRSConfigurationType  - DM-RS configuration type (1 (default), 2).
%                                When transform precoding is enabled, the
%                                value must be 1
%       DMRSTypeAPosition      - Position of first DM-RS OFDM symbol in a
%                                slot (2 (default), 3)
%       DMRSLength             - Number of consecutive DM-RS OFDM symbols
%                                (1 (default), 2). When intra-slot
%                                frequency hopping is enabled, the value
%                                must be 1. Value of 1 indicates
%                                single-symbol DM-RS. Value of 2 indicates
%                                double-symbol DM-RS
%       DMRSAdditionalPosition - Maximum number of DM-RS additional
%                                positions (0...3) (default 0). When
%                                intra-slot frequency hopping is enabled,
%                                the value must be either 0 or 1
%       DMRSPortSet            - DM-RS antenna port set (0...11)
%                                (default []). The default value implies
%                                that the values are in the range from 0 to
%                                NumLayers-1
%       CustomSymbolSet        - Custom DM-RS symbol locations (0-based)
%                                (default []). This property is used to
%                                override the standard defined DM-RS symbol
%                                locations. Each entry corresponds to a
%                                single-symbol DM-RS
%    These properties are applicable, when transform precoding is set to 0:
%       NIDNSCID               - DM-RS scrambling identities (0...65535)
%                                (default []). Use empty ([]) to set the
%                                value to NCellID
%       NSCID                  - DM-RS scrambling initialization
%                                (0 (default), 1)
%       DMRSUplinkR16          - Enable R16 low PAPR DM-RS sequence for 
%                                CP-OFDM (0 (default), 1)
%       DMRSEnhancedR18        - Enable R18 enhanced DM-RS multiplexing
%                                (0 (default), 1)
%    These properties are applicable, when transform precoding is set to 1:
%       GroupHopping           - Group hopping configuration
%                                (0 (default), 1). 0 indicates that group
%                                hopping is disabled. 1 indicates that
%                                group hopping is enabled
%       SequenceHopping        - Sequence hopping configuration
%                                (0 (default), 1). 0 indicates that
%                                sequence hopping is disabled. 1 indicates
%                                that sequence hopping is enabled. Note
%                                that both group hopping and sequence
%                                hopping must not be enabled
%                                simultaneously
%       NRSID                  - DM-RS scrambling identity (0...1007)
%                                (default []). Use empty ([]) to set the
%                                value to NCellID
%       DMRSUplinkTransformPrecodingR16 - Enable R16 low PAPR DM-RS sequence 
%                                for DFT-s-OFDM (0 (default), 1) when 
%                                pi/2-BPSK modulation is used for PUSCH
%
%   SYM = nrPUSCHDMRS(CARRIER,PUSCH,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the data type of the
%   output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   For operation with shared spectrum channel access for FR1, set
%   Interlacing = true and specify the allocated frequency resources using
%   the RBSetIndex and InterlaceIndex properties of the PUSCH
%   configuration. The PRBSet, FrequencyHopping, and SecondHopStartPRB
%   properties are ignored.
%
%   Example 1:
%   % Generate DM-RS symbols of a physical uplink shared channel
%   % occupying the 10 MHz bandwidth for a 15 kHz subcarrier spacing (SCS)
%   % carrier. Configure DM-RS with type A position set to 2, number of
%   % additional positions set to 0, length set to 1, and configuration
%   % type set to 1.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pusch = nrPUSCHConfig;
%   pusch.DMRS.DMRSTypeAPosition = 2;
%   pusch.DMRS.DMRSAdditionalPosition = 0;
%   pusch.DMRS.DMRSLength = 1;
%   pusch.DMRS.DMRSConfigurationType = 1;
%   sym = nrPUSCHDMRS(carrier,pusch);
%
%   Example 2:
%   % Generate DM-RS symbols of a physical uplink shared channel with
%   % transform precoding enabled, transmission scheme set to codebook,
%   % number of antenna ports set to 4 and TPMI set to 2.
%
%   carrier = nrCarrierConfig;
%   pusch = nrPUSCHConfig('TransformPrecoding',1);
%   pusch.TransmissionScheme = 'codebook';
%   pusch.NumAntennaPorts = 4;
%   pusch.TPMI = 2;
%   sym = nrPUSCHDMRS(carrier,pusch);
%
%   See also nrPUSCHDMRSIndices, nrTimingEstimate, nrChannelEstimate,
%   nrPUSCHPTRS, nrPUSCHConfig, nrPUSCHDMRSConfig, nrCarrierConfig,
%   nrIntraCellGuardBandsConfig.

%   Copyright 2019-2025 The MathWorks, Inc.

%#codegen

    narginchk(2,4);

    % Validate inputs
    [~,~,nSizeBWP,nStartBWP,symbperslot,freqHopping] = nr5g.internal.pusch.validateInputs(carrier,pusch);

    % Assign the structure ftable to pass into the initializeResources
    % internal function
    ftable.ChannelName = 'PUSCH';
    ftable.MappingTypeB = strcmpi(pusch.MappingType,'B');
    ftable.DMRSSymbolSet = @nr5g.internal.pusch.lookupPUSCHDMRSSymbols;
    ftable.IntraSlotFreqHoppingFlag = strcmpi(freqHopping,'intraSlot');

    % Relative slot number
    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    % Get prbset, symbolset and dmrssymbols
    [prbset,symbolset,dmrssymbols,ldash] = nr5g.internal.pxsch.initializeResources(carrier,pusch,nSizeBWP,ftable);
    nsymbols = numel(symbolset);

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

    % Capture the number of transmission layers required
    nLayers = double(pusch.NumLayers);

    % Number of DM-RS subcarrier locations in a single resource block
    % 6 DM-RS QPSK symbols (type 1) or 4 DM-RS QPSK symbols (type 2) per RB
    ndmrsre = 4 + (pusch.DMRS.DMRSConfigurationType==1)*2;

    % Get the DM-RS symbols across all ports
    dmrsSymPort = complex(zeros(0,nLayers));
    numDMRSSym = numel(dmrssymbols);
    if numDMRSSym
        % Cache the scrambling identities, accounting for 
        % the transform/non-transform precoding parameter differences
        if pusch.TransformPrecoding
            % Parameter processing for the SC-FDMA/transform precoding case

            % Use low PAPR type 2 sequence if dmrs-UplinkTransformPrecoding-r16 set and pi/2-BPSK in use, otherwise type 1 sequence
            if iscell(pusch.Modulation)
                modstring = pusch.Modulation{1};
            else
                modstring = pusch.Modulation;
            end
            lowpaprtype = 1+(pusch.DMRS.DMRSUplinkTransformPrecodingR16 && strcmpi(modstring,'pi/2-BPSK'));

            % Prepare the NRSID parameter
            % If empty and R16 & pi/2-BPSK then use selected nidnscid later (this is case for low PAPR type 2) 
            if isempty(pusch.DMRS.NRSID) && lowpaprtype==1   % Allow empty to be passed on in the case of type 2
                nrsid = carrier.NCellID;
            else
                nrsid = pusch.DMRS.NRSID;
            end
        else
            % OFDM/non-transform precoding case
            nrsid = pusch.DMRS.NRSID;       % Cached, but is not in play for OFDM (i.e. not transform precoding)
            lowpaprtype = 0;                % Type 1/type 2 low PAPR sequences are not used for OFDM
        end

        % Construct a cell array where each row is a given 'base' DM-RS PRBS sequences
        % for an antenna port, across the OFDM symbols containing DM-RS. 
        % For R16 low PAPR DM-RS, the 'base' sequences have a dependency on the CDM group
        if ~pusch.TransformPrecoding
            [symcell,cnLen,port2baseseq] = nr5g.internal.prbsDMRSSequenceSets(carrier,...       % NSlot part    
                                                pusch.DMRS,...
                                                pusch.DMRS.DMRSUplinkR16,...
                                                prbcell,nStartBWP,...              % PRB part
                                                dmrssymbols);                      % Specific symbol numbers & DM-RS ports required
        else
            % If using transform precoding, or OFDM and not using dmrs-Uplink-r16 then there is a single DM-RS sequence
            % set for all DM-RS ports (cinit value has no dependency on the CDM group)
            nscid = pusch.DMRS.NSCID;        % Get the single NSCID value
            if isempty(pusch.DMRS.NIDNSCID)  % For SC-FDMA, this parameter is relevant for low PAPR type 2 sequence initialization
                nidnscid = carrier.NCellID;
            else
                nidnscid = pusch.DMRS.NIDNSCID;
            end
            port2baseseq = ones(1,nLayers);  % Mapping between ports required and, in this case, a single base DM-RS sequence
            symcell = coder.nullcopy(cell(1,numDMRSSym));
            cnLen = zeros(1,numDMRSSym);
            % Loop over the base sequences
            for i=1:numDMRSSym
                % Included the empty check to avoid run-time error in codegen
                % for empty PRB set with reshape function
                if ~isempty(prbcell{dmrssymbols(i)+1})
                    % Get the type 1/type 2 low PAPR sequence
                    symcell{i} = reshape(nr5g.internal.pusch.lowPAPRSequence(...
                                                struct('Type', lowpaprtype,...
                                                'NIDNSCID',nidnscid,'NSCID',nscid(1),...           % Type 2 parameters 
                                                'NRSID',nrsid,...                                  % Type 1 and both type 1 & type 2 shared hopping ID 
                                                'GroupHopping',pusch.DMRS.GroupHopping,'SequenceHopping',pusch.DMRS.SequenceHopping),...
                                                ndmrsre,prbcell{dmrssymbols(i)+1},nslot,dmrssymbols(i),ldash(i),symbperslot),...
                                              [],1);
                else
                    symcell{i} = zeros(0,1);
                end
                cnLen(i) = length(symcell{i});
            end
        end

        % Accumulated lengths of the DM-RS when concatenated across the OFDM symbols
        % This is used to identify ranges for the DM-RS values for a given symbol
        % within the concatenated array output
        cndmrs = [0 cumsum(cnLen)];
        % Preallocate array for the returned DM-RS symbols
        dmrsSymPort = complex(zeros(cndmrs(end),nLayers));

        % Loop over the ports/layers and apply TD/TD OCC masks
        fmaskAllPorts = pusch.DMRS.FrequencyWeights;
        tmaskAllPorts = pusch.DMRS.TimeWeights;

        nwf = size(fmaskAllPorts,1);
        nfmaskrep = lcm(ndmrsre,nwf)/nwf;

        % Loop over the antenna ports and apply associated FD/TD OCC masks
        for pidx = 1:nLayers

            % Get FD OCC mask to be applied to every DM-RS carrying symbol of port
            fmask = fmaskAllPorts(:,pidx);
            % Replicate the basic FD OCC mask, across the minimum number of PRB for a complete cover             
            fmaskprb = reshape(repmat(fmask,nfmaskrep,1),ndmrsre,[]);   

            % ldash (l') contains the TD weight selection to use per symbol
            tmask = tmaskAllPorts(ldash+1,pidx);

            % Apply combined time and frequency mask values and concatenate
            % across all the OFDM symbols
            % For current port, step through each DM-RS symbols values, apply FD/TD OCC weights for that symbol, and assign result into a subset of the output matrix
            for i = 1:size(symcell,2)
               % Select the FD masks for each PRB in allocation and reshape into a single column
               rbidxs = prbcell{dmrssymbols(i)+1}+nStartBWP;
               fmaskallprb = reshape(fmaskprb(:,mod(rbidxs,size(fmaskprb,2))+1),[],1);  
               % Apply time and frequency weights to DM-RS symbols
               dmrsSymPort(cndmrs(i)+1:cndmrs(i+1),pidx) = symcell{port2baseseq(pidx),i}.*fmaskallprb*tmask(i);
            end
        end

    end

    % Provide the DM-RS symbols based on the number of antenna ports when
    % transmission scheme is set to codebook
    if strcmpi(pusch.TransmissionScheme,'codebook') && ~isempty(dmrsSymPort)
        % Use the CDM group number to label the DM-RS symbols and group
        % them into different sets of rows. Within a PRB, we know that the
        % lower CDM groups have lower delta shifts
        cdm = zeros(3,1);
        cdm(pusch.DMRS.CDMGroups+1) = 1;
        cdmGroups = find(cdm)-1;
        ngroups = numel(cdmGroups);
        logicalMatrix = repmat(pusch.DMRS.CDMGroups,ngroups,1) == repmat(cdmGroups,1,nLayers);
        [cdmgroupsidx, ~] = find(logicalMatrix);
        cdmgroupsidx = reshape(cdmgroupsidx,1,[]);
        nDMRSPerLayer = numel(prbset)*ndmrsre*numDMRSSym;

        % DM-RS symbols
        pdmrsSymbols = complex(zeros([ngroups*nDMRSPerLayer nLayers]));
        indices = repmat((cdmgroupsidx-1),nDMRSPerLayer,1) + reshape(1:ngroups:ngroups*nDMRSPerLayer*nLayers,[],nLayers);  
        pdmrsSymbols(indices) = dmrsSymPort;

        % Get the precoding matrix from the codebook
        W = nrPUSCHCodebook(nLayers,pusch.NumAntennaPorts,pusch.TPMI,pusch.TransformPrecoding,pusch.CodebookType);

        % Apply codebook matrix to symbols
        dmrs = pdmrsSymbols * W;
    else
        dmrs = dmrsSymPort;
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPUSCHDMRS';
        opts = nr5g.internal.parseOptions(fcnName,{'OutputDataType'},varargin{:});
        sym = cast(dmrs,opts.OutputDataType);
    else
        % Cast to double to have same behavior for empty output in codegen
        % path and simulation path
        sym = double(dmrs);
    end

end