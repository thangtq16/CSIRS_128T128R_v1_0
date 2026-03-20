function sym = nrPUSCHPTRS(carrier,pusch,varargin)
%nrPUSCHPTRS Physical uplink shared channel phase tracking reference signal
%   SYM = nrPUSCHPTRS(CARRIER,PUSCH) returns the phase tracking reference
%   signal (PT-RS) symbols, SYM, of physical uplink shared channel, for the
%   given carrier configuration CARRIER, and uplink shared channel
%   configuration PUSCH according to TS 38.211 Section 6.4.1.2.1. The
%   output SYM is a matrix with number of columns depending on the
%   transmission scheme and transform precoding. The number of columns in
%   SYM equals to:
%   - number of PT-RS antenna ports configured, when transform precoding is
%   disabled and transmission scheme is set to non-codebook
%   - number of antenna ports configured, when transform precoding is
%   disabled and transmission scheme is set to codebook
%   - number of transmission layers, when transform precoding is enabled
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
%   RNTI               - Radio network temporary identifier (0...65535)
%                        (default 1)
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
%       NIDNSCID               - DM-RS scrambling identity (0...65535)
%                                (default []). Use empty ([]) to set the
%                                value to NCellID
%       NSCID                  - DM-RS scrambling initialization
%                                (0 (default), 1)
%       DMRSUplinkR16          - Enable R16 low PAPR DM-RS sequence for 
%                                CP-OFDM (0 (default), 1)
%    These properties are applicable, when transform precoding is set to 1:
%       NRSID                  - DM-RS scrambling identity (0...1007)
%                                (default []). Use empty ([]) to
%                                set the value to NCellID
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
%       NID                    - PT-RS scrambling identity (0...1007)
%                                (default []). Use empty ([]) to set the
%                                value to DM-RS scrambling identity NRSID
%
%   SYM = nrPUSCHPTRS(CARRIER,PUSCH,NAME,VALUE) specifies additional
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
%   % Generate PT-RS symbols for a PUSCH transmission occupying a 10 MHz
%   % bandwidth for a 15 kHz subcarrier spacing (SCS) carrier, with 
%   % transform precoding set to 0. Configure DM-RS with number of
%   % additional positions set to 0, length set to 1, type A position set
%   % to 2, and configuration type set to 1. Enable PT-RS with time density
%   % set to 1, and frequency density set to 2.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pusch = nrPUSCHConfig('TransformPrecoding',0);
%   pusch.DMRS.DMRSLength = 1;
%   pusch.DMRS.DMRSTypeAPosition = 2;
%   pusch.DMRS.DMRSAdditionalPosition = 0;
%   pusch.DMRS.DMRSConfigurationType = 1;
%   pusch.EnablePTRS = 1;
%   pusch.PTRS.TimeDensity = 1;
%   pusch.PTRS.FrequencyDensity = 2;
%   sym = nrPUSCHPTRS(carrier,pusch);
%
%   Example 2:
%   % Generate PT-RS symbols for a PUSCH transmission occupying a 10 MHz
%   % bandwidth for a 15 kHz SCS carrier, with transform precoding enabled.
%   % Configure DM-RS with length set to 1, type A position set to 2, 
%   % number of additional positions set to 0, and configuration type 
%   % set to 1. Enable PT-RS with number of PT-RS samples set to 2, 
%   % and number of PT-RS groups set to 4.
%
%   carrier = nrCarrierConfig;
%   pusch = nrPUSCHConfig('TransformPrecoding',1);
%   pusch.DMRS.DMRSLength = 1;
%   pusch.DMRS.DMRSTypeAPosition = 2;
%   pusch.DMRS.DMRSAdditionalPosition = 0;
%   pusch.DMRS.DMRSConfigurationType = 1;
%   pusch.EnablePTRS = 1;
%   pusch.PTRS.NumPTRSSamples = 2;
%   pusch.PTRS.NumPTRSGroups = 4;
%   sym = nrPUSCHPTRS(carrier,pusch);
%
%   Example 3:
%   % Generate the PT-RS symbols for a PUSCH transmission with transform
%   % precoding enabled, transmission scheme set to codebook, number of
%   % antenna ports set to 4, and TPMI set to 2. Enable PT-RS with number
%   % of PT-RS samples set to 4, and number of PT-RS groups set to 8.
%
%   carrier = nrCarrierConfig;
%   pusch = nrPUSCHConfig('TransformPrecoding',1);
%   pusch.TransmissionScheme = 'codebook';
%   pusch.NumAntennaPorts = 4;
%   pusch.TPMI = 2;
%   pusch.EnablePTRS = 1;
%   pusch.PTRS.NumPTRSSamples = 4;
%   pusch.PTRS.NumPTRSGroups = 8;
%   sym = nrPUSCHPTRS(carrier,pusch);
%
%   See also nrPUSCHPTRSIndices, nrPUSCHDMRS, nrPUSCHConfig,
%   nrPUSCHDMRSConfig, nrPUSCHPTRSConfig, nrCarrierConfig,
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
    [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pusch,nSizeBWP,ftable);

    % Capture set of transmission layers required
    nLayers = double(pusch.NumLayers);
    if isempty(pusch.DMRS.DMRSPortSet)
        dmrsports = 0:nLayers-1;
    else
        dmrsports = double(pusch.DMRS.DMRSPortSet);
    end

    % Get the number of PT-RS ports
    if ~pusch.TransformPrecoding
        % PT-RS port set
        if isempty(pusch.PTRS.PTRSPortSet)
            ptrsPorts = min(dmrsports(:));
        else
            ptrsPorts = double(unique(pusch.PTRS.PTRSPortSet(:)));
        end
    else
        ptrsPorts = dmrsports;
    end
    nPTRSPorts = numel(ptrsPorts);

    % Get the number of antenna ports
    codebookTxFlag = strcmpi(pusch.TransmissionScheme,'codebook');
    if codebookTxFlag
        nports = double(pusch.NumAntennaPorts);
    else
        nports = nLayers;
    end

    % Cache the number of columns in the output
    ncols = nPTRSPorts;
    if ~pusch.TransformPrecoding && codebookTxFlag
        ncols = nports;
    end

    if pusch.EnablePTRS && ~isempty(dmrssymbols) && ~isempty(prbset)
        % PRB matrix where each row corresponding to the PRB set of each hop
        prbMat = [prbset;prbset-min(prbset)+double(pusch.SecondHopStartPRB)];

        % PT-RS OFDM symbol set
        ptrssymbols = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbols,double(pusch.PTRS.TimeDensity));

        if ~pusch.TransformPrecoding

            %
            % OFDM WAVEFORM CASE
            %

            % Assign the frequency mode to different values based on the
            % frequency hopping configuration
            switch lower(freqHopping)
                case 'intraslot'
                    freqMode = 2;
                case 'interslot'
                    freqMode = mod(nslot,2);
                otherwise
                    freqMode = 0;
            end

            % Subcarrier locations of PT-RS for each OFDM symbol
            [kRefTable,dmrsSCPattern,nDMRSSC] = nr5g.internal.pxsch.ptrsSubcarrierInfo(pusch.DMRS.DMRSConfigurationType);
            colIndex = strcmpi(pusch.PTRS.REOffset,{'00','01','10','11'});
            kRERef = kRefTable(ptrsPorts+1,colIndex);

            % DM-RS OFDM symbol locations within the second hop (if any)
            dmrsIndex = dmrssymbols >= (floor(pusch.SymbolAllocation(end)/2)+pusch.SymbolAllocation(1));
            dmrsHop2 = dmrssymbols(dmrsIndex);  % DM-RS symbol indices in second hop
            dmrsHop2Flag = ~isempty(dmrsHop2);

            % First generate DM-RS symbols for each hop

            % In the case of interslot frequency hopping, the second PRB set
            % needs selected for old numbered slots
            prbOneHop = prbMat(1 + (freqMode == 1),:);
    
            % Codegen compatible cell-array-of-empty-vectors initialisation
            initEmpty = zeros(1,0);
            coder.varsize('initEmpty',[1,Inf],[0,1]);
            prbcell = repmat({initEmpty},1,symbperslot);

            % Specify the symbol and PRB indices for the DM-RS symbols which will 
            % initialise the PT-RS 
            twosymbols = logical(freqMode) && dmrsHop2Flag;
            dmsym = zeros(1,1+double(twosymbols));    
            dmsym(1) = dmrssymbols(1); 
            prbcell{dmsym(1)+1} = prbOneHop;  
            if twosymbols
                dmsym(2) = dmrsHop2(1);
                prbcell{dmsym(2)+1} = prbMat(2,:);
            end
            % Get the DM-RS PRBS-based base sequences for the symbols and ports of interest  
            [symcell,~,port2baseseq] = nr5g.internal.prbsDMRSSequenceSets(carrier,...  % NSlot part
                                              pusch.DMRS,...                 % DM-RS config (CDM groups)
                                              pusch.DMRS.DMRSUplinkR16,...   % R16 control
                                              prbcell,nStartBWP,...          % PRB part
                                              dmsym,ptrsPorts);              % Symbol numbers & CDM groups part
            
            % Resource block offset of PT-RS
            nPUSCHRB = numel(prbset);
            kptrs = double(pusch.PTRS.FrequencyDensity);
            if mod(nPUSCHRB,kptrs) == 0
                kRBRef = mod(double(pusch.RNTI),kptrs);
            else
                kRBRef = mod(double(pusch.RNTI),mod(nPUSCHRB,kptrs));
            end

            ptrsTemp = coder.nullcopy(cell(1,numel(ptrsPorts)));
            dmrsSym = complex(zeros(nDMRSSC,numel(prbset),1+double((freqMode~=0)&&dmrsHop2Flag)));
            for p = 1:numel(ptrsPorts)
                % PT-RS symbols for each hop
                [~,scIndex] = find(repmat(kRERef(p),size(dmrsSCPattern)) == dmrsSCPattern);
                % Map the base sequence cell array into a normal numerical array
                for i = 1:numel(dmsym)
                    dmrsSym(:,:,i) = reshape(symcell{port2baseseq(p),i},nDMRSSC,[]);  % Each column per plane represents DM-RS for each PRB for DM-RS symbol 'i' 
                end
                ptrsHop = permute(dmrsSym(scIndex,kRBRef(1)+1:kptrs(1):end,:),[2 3 1]); % Each column corresponds to a hop

                % PT-RS symbol indices in second hop
                SecondHopInd = ptrssymbols >= (floor(pusch.SymbolAllocation(end)/2)+pusch.SymbolAllocation(1));

                % PT-RS symbols and indices for each port
                if freqMode == 2
                    % Intra-slot frequency hopping enabled
                    if ~isempty(dmrssymbols(~dmrsIndex))
                        % First hop contents
                        ptrsTemp{1,p} = repmat(ptrsHop(:,1),nnz(~SecondHopInd),1);
                    end
                    if dmrsHop2Flag
                        % Second hop contents
                        % Provide PT-RS symbols and indices for second hop, when
                        % at-least one DM-RS symbol index is present in second hop
                        ptrsTemp{1,p} = [ptrsTemp{1,p}; repmat(ptrsHop(:,2),nnz(SecondHopInd),1)];
                    end
                else
                    % Intra-slot frequency hopping disabled
                    ptrsTemp{1,p} = repmat(ptrsHop(:,1),numel(ptrssymbols),1);
                end
            end

            % NonOrthogonal port multiplexing
            symT = complex(zeros(0,1));
            for i = 1:nPTRSPorts
                if ~isempty(ptrsTemp{1,i})
                    symT = [symT;reshape(ptrsTemp{1,i},[],1)]; %#ok<AGROW>
                end
            end
            symNonCodebook = reshape(symT,[],nPTRSPorts);

            % Provide the PT-RS symbols based on the number of antenna
            % ports, when transmission scheme is set to codebook
            if codebookTxFlag
                % Get the normed PT-RS symbols and indices to perform
                % multiplication with precoding matrix
                unqRERef = unique(kRERef(:));
                factor = numel(unqRERef);
                pptrsSymbols = complex(zeros(size(symNonCodebook,1)*factor,nLayers));
                portMatrix = repmat(reshape(kRERef,1,[]),factor,1) == repmat(unqRERef,1,numel(kRERef));
                for i = 1:nPTRSPorts
                    jPort = find(portMatrix(:,i));
                    [~,offset] = find(ptrsPorts(i) == dmrsports);
                    pptrsSymbols(jPort(1):factor:end, offset(1)) = symNonCodebook(:,i);
                end
                % Apply codebook matrix for PT-RS symbols
                ptrs = pptrsSymbols * nrPUSCHCodebook(nLayers,pusch.NumAntennaPorts,pusch.TPMI,pusch.TransformPrecoding,pusch.CodebookType);
            else
                ptrs = symNonCodebook;
            end
        else

            %
            % TRANSFORM PRECODED/SC-FDMA WAVEFORM CASE
            %

            % Transform precoding (DFT-s-OFDM)
            if isempty(pusch.PTRS.NID)
                if isempty(pusch.DMRS.NRSID)
                    ptrsNID = double(carrier.NCellID);
                else
                    ptrsNID = double(pusch.DMRS.NRSID);
                end
            else
                ptrsNID = double(pusch.PTRS.NID);
            end

            % Get nGroupSamp and nPTRSGroup
            nGroupSamp = double(pusch.PTRS.NumPTRSSamples); % Number of samples in a PT-RS group
            nPTRSGroup = double(pusch.PTRS.NumPTRSGroups);  % Number of PT-RS groups

            % Get orthogonal sequence
            w = nr5g.internal.pusch.ptrsOrthogonalSeqDFTsOFDM(pusch.RNTI,nGroupSamp);

            % Get the subcarrier locations of PT-RS
            nPUSCHSC = numel(prbset)*12;
            mTemp = nr5g.internal.pusch.ptrsSCIndicesDFTsOFDM(nGroupSamp,nPTRSGroup,nPUSCHSC);

            % Get the PRBS sequence
            mDash = nGroupSamp*nPTRSGroup;
            prbs = zeros(mDash,1,'logical');
            if ~isempty(ptrssymbols)
                cinit = mod(2^17*((symbperslot*nslot+ptrssymbols(1)+1)*(2*ptrsNID+1))+2*ptrsNID,2^31);
                prbs = nrPRBS(cinit,mDash);
            end

            % Get the PT-RS symbols multiplied with betaPrime and the PT-RS indices
            modBPSK = 1/sqrt(2)*(1-2*prbs(:));
            symTemp = repmat(w,nPTRSGroup,1).*exp(1j*pi*(mod(mTemp,2)/2)).*...
                complex(modBPSK,modBPSK);
            % Replicate base symTemp vector for each PT-RS symbol (creating a single column), then this complete column block across each layer 
            ptrs = repmat(repmat(symTemp,numel(ptrssymbols),1),1,nLayers); 
        end
    else
        ptrs = complex(zeros(0,ncols));
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPUSCHPTRS';
        opts = nr5g.internal.parseOptions(fcnName,{'OutputDataType'},varargin{:});
        sym = cast(ptrs,opts.OutputDataType);
    else
        % Cast to double to have same behavior for empty output in codegen
        % path and simulation path
        sym = double(ptrs);
    end

end