function info = OFDMInfo(carrier,prach,opts)
%OFDMInfo PRACH OFDM modulation related information
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2020-2025 The MathWorks, Inc.

%#codegen
    
    % Sampling rate corresponding to 1/T_s where T_s is defined in TS
    % 38.211 Section 4.1. This will be adopted as the nominal sampling rate
    % for calculations here
    sr_nominal = 30.72e6;
    
    % Nominal IDFT size (i.e. in terms of the sampling rate above). This
    % corresponds to parts of the expressions for N_u in TS 38.211 Tables
    % 6.3.3.1-1 and 6.3.3.1-2:
    % For formats 0, 1 and 2: 24576
    % For format 3: 6144
    % For short formats: 2048 * 2^-mu
    n_idft_nominal = 2048 * 15 / prach.SubcarrierSpacing;
    
    % Get PRACH slot grid size and record the number of subcarriers sizK
    % and the number of OFDM symbols sizL
    carrierSCS = double(carrier.SubcarrierSpacing);
    carrierNRB = double(carrier.NSizeGrid);
    siz = nr5g.internal.prach.gridSize(carrierNRB,carrierSCS,prach,1);
    sizK = siz(1);
    sizL = siz(2);
    
    % Calculate OFDM information for TS 38.211 Section 5.3.1 "OFDM
    % basedband signal generation for all channels except PRACH", for the
    % numerology described in TS 38.211 Section 5.3.2
    LRA = prach.LRA;
    if (LRA==839)
        % For long sequences (LRA=839) numerology mu=0 is assumed
        deltaf_RA = 15;
    else % LRA==139,571,1151
        % For short sequences (LRA=139,571,1151) the numerology given by
        % the PRACH subcarrier spacing is used
        deltaf_RA = prach.SubcarrierSpacing;
    end
    muinfo = nr5g.internal.OFDMInfo(sizK/12,deltaf_RA,false,struct());
    
    % Calculate OFDM symbol start samples n_mu_start_l. These are the
    % sample indices corresponding to t_mu_start_l described in TS 38.211
    % Section 5.3.2, calculated at the nominal sampling rate. N_mu_u and
    % N_mu_CP are also defined in Section 5.3.2. n_mu_tot_l is the number
    % of samples to the end of OFDM symbol 'l' and is used subsequently for
    % calculating guard lengths
    N_mu_u = muinfo.Nfft * sr_nominal / muinfo.SampleRate;
    N_mu_CP = muinfo.CyclicPrefixLengths * sr_nominal / muinfo.SampleRate;
    nSlot = mod(prach.NPRACHSlot,muinfo.SlotsPerSubframe);
    start_instant = sum(N_mu_CP(1:nSlot*muinfo.SymbolsPerSlot) + N_mu_u); % Start of the PRACH slot within the carrier subframe
    N_mu_CP = N_mu_CP(nSlot*muinfo.SymbolsPerSlot + (1:muinfo.SymbolsPerSlot));
    n_mu_tot_l = cumsum(N_mu_CP + N_mu_u);
    n_mu_start_l = [0 n_mu_tot_l(1:end-1)];
    
    % Get starting OFDM symbol l_0 of the first PRACH time occasion,
    % defined in TS 38.211 Section 5.3.2
    configTableRow = nr5g.internal.prach.getVariablesFromConfigTable(prach.FrequencyRange,prach.DuplexMode,prach.ConfigurationIndex);
    l_0 = configTableRow.StartingSymbol;
    
    % For long sequences (LRA=839) and non-zero l_0, the starting position
    % is implemented as an initial guard length N_offset
    straddleSF = false;
    prachFormat = prach.Format;
    if (LRA==839 && l_0 > 0)
        N_offset = n_mu_tot_l(l_0);
        l_0 = 0;
        % For preamble formats 0 and 3, record that an active PRACH
        % preamble straddles an extra subframe due to the non-zero l_0
        if (any(strcmpi(prachFormat,{'0','3'})))
            straddleSF = true;
        end
    else
        N_offset = 0;
    end
    
    % Get the total number of PRACH time occasions n_RA_t, defined in TS
    % 38.211 Section 5.3.2
    n_RA_t = prach.NumTimeOccasions;
    
    % Get the duration N_RA_dur in OFDM symbols of one PRACH time occasion,
    % defined in TS 38.211 Section 5.3.2
    N_RA_dur = prach.PRACHDuration;
    
    % In the case of format C0, each preamble has one active sequence
    % period (see Table 6.3.3.1-2) but including the guard and the cyclic
    % prefix, the preamble spans two OFDM symbols. For this reason, the
    % slot grid related to format C0 has 7 OFDM symbols, rather than 14,
    % and each value related to it that is derived directly from TS 38.211
    % is halved
    if (strcmpi(prachFormat,'C0'))
        l_0 = l_0 / 2;
    end
    
    % Get the vector of OFDM symbol indices l_starts (0-based) which
    % correspond to the starting symbol of each PRACH time occasion in the
    % PRACH slot. TS 38.211 Section 5.3.2 only considers a single 'l' for
    % the start of a given PRACH time occasion, this vector contains all
    % values of 'l'
    l_starts = l_0 + ((0:(n_RA_t-1)) * N_RA_dur(1));
    
    % If the starting OFDM symbol l_0 of the first PRACH time occasion is
    % non-zero, add OFDM symbol zero to the vector of starting symbol
    % indices so that total duration of OFDM symbols 0...(l_0-1) is
    % correctly accounted for in subsequent calculations
    if (l_0 > 0)
        l_starts = [0 l_starts];
    end
    
    % Get the cyclic prefix length N_RA_CP and useful OFDM symbol period
    % N_u from TS 38.211 Table 6.3.3.1-1 or 6.3.3.1-2
    if (LRA==839)
        formatTable1 = nr5g.internal.prach.getTable6331x(1);
        formatTableRow = formatTable1(strcmpi(formatTable1.Format,prachFormat),end-2:end-1);
    else % LRA==139,571,1151
        formatTable2 = nr5g.internal.prach.getTable6331x(2);
        formatTableRow = formatTable2(strcmpi(formatTable2.Format,prachFormat),end-2:end-1);
    end
    N_RA_CP = formatTableRow.N_CP;
    N_u = formatTableRow.N_u;
    
    % For short sequences (LRA=139,571,1151), adjust cyclic prefix lengths
    % and useful lengths to account for numerology (2^-mu term in TS 38.211
    % Table 6.3.3.1-2)
    if (LRA~=839)
        N_RA_CP = N_RA_CP * n_idft_nominal / 2048;
        N_u = N_u * n_idft_nominal / 2048;
    end
    
    % Calculate cyclic prefix lengths N_RA_CP_l as described in TS 38.211
    % Section 5.3.2, at nominal sampling rate. Note that the vector
    % N_RA_CP_l only has non-zero values in the positions where 'l'
    % corresponds to the first OFDM symbol of a PRACH time occasion.
    coder.varsize("N_RA_CP_l",[1 14],[0 1]);
    N_RA_CP_l = zeros([1 sizL]);
    N_RA_CP_l(l_starts + 1) = repmat(N_RA_CP,1,numel(l_starts));
    if (l_0 > 0)
        N_RA_CP_l(1) = 0;
    end
    
    % In the case of format C0, each preamble has one active sequence
    % period (see Table 6.3.3.1-2) but including the guard and the cyclic
    % prefix, the preamble spans two OFDM symbols. For this reason, the
    % slot grid related to format C0 has 7 OFDM symbols, rather than 14.
    % Therefore, the symbol indices 'l_starts' and 'l_ends' that address
    % the PRACH slot grid must be adjusted to correctly address the OFDM 
    % information for numerology mu (for other formats, no adjustment is 
    % required). 'l_starts' is adjusted here, 'l_ends' is adjusted later
    % once it has been calculated
    if (strcmpi(prachFormat,'C0'))
        l_mu_starts = l_starts*2;
    else
        l_mu_starts = l_starts;
    end

    % Calculate n_RA_start, the sample indices at nominal sampling rate
    % that correspond to the starting times of the PRACH preambles in a
    % subframe, t_RA_start, defined in TS 38.211 Section 5.3.2
    n_RA_start = n_mu_start_l(l_mu_starts + 1).';
    
    % For short sequences (LRA=139,571,1151), the cyclic prefix for each
    % PRACH occasion will be extended by n=16 samples either zero, one, or
    % two times depending on how many times the PRACH occasion crosses time
    % instants 0 and 0.5 ms
    if (LRA~=839)
        n0dot5ms = sr_nominal * 0.5 * 1e-3;
        interval_end = n_RA_start + ...
            repmat(N_u + N_RA_CP,numel(l_mu_starts),1);
        if (l_0 > 0)
            % The interval corresponding to non-zero l_0 spans OFDM symbols
            % 0...(l_0-1), so the interval ends at the start of the second 
            % interval (OFDM symbol l_0), given by n_RA_start(2)
            interval_end(1) = n_RA_start(2);
        end
        n = 16;
        for instant = mod([0 n0dot5ms] - start_instant,n0dot5ms*2)
            i = find((n_RA_start <= instant) & ...
                (interval_end > instant),1,'first');
            if (~isempty(i))
                l = l_starts(i) + 1;
                N_RA_CP_l(l) = N_RA_CP_l(l) + n;
            end
        end
    end
    % Make sure that all CP lengths are within the FFT size, to counteract
    % the extra OFDM symbol added in nrPRACH and nrPRACHIndices. For
    % example, format C2 has a CP length equal to the nominal IDFT size.
    % Thus, for format C2, the CP lengths are zero but extended to 'n'
    % samples.
    idx = N_RA_CP_l >= n_idft_nominal;
    N_RA_CP_l_phasecomp = N_RA_CP_l;
    if any(idx) && (nr5g.internal.prach.getNumOFDMSymbols(prach)~=prach.PRACHDuration)
        N_RA_CP_l(idx) = N_RA_CP_l(idx) - n_idft_nominal;
    end
    
    % Calculate guard lengths N_GP_l at nominal sampling rate. The guard
    % lengths are not explicitly mentioned in TS 38.211, but they span the
    % interval between (n_RA_start + N_u + N_RA_CP_l) for one PRACH time
    % occasion and n_RA_start for the next PRACH time occasion (or the end
    % of the PRACH slot). Note that the useful OFDM symbol period N_u for a
    % PRACH time occasion is split between multiple OFDM symbols, each
    % having useful period n_idft_nominal, with only the first OFDM symbol
    % having a cyclic prefix and only the last having a guard. Therefore
    % N_GP_l only has non-zero values in the positions where 'l'
    % corresponds to the last OFDM symbol of a PRACH time occasion (or in
    % the last OFDM symbol of the PRACH slot). The final guard length of
    % the PRACH slot pads the PRACH slot length up to an integer multiple
    % of the slot length of the reference numerology mu
    N_GP_l = zeros([1 sizL]);
    l_ends_temp = l_starts + repmat(N_RA_dur - 1,1,numel(l_starts));
    if (l_0 > 0)
        l_ends_temp = [(l_0 - 1) l_ends_temp(2:end)];
    end
    if (l_ends_temp(end) == (sizL-1))
        l_ends = l_ends_temp(1:end-1);
    else
        l_ends = l_ends_temp;
    end
    n_RA_tot_l = cumsum(n_idft_nominal + N_RA_CP_l);
    if (strcmpi(prachFormat,'C0'))
        l_mu_ends = (l_ends*2 + 1);
    else
        l_mu_ends = l_ends;
    end
    N_GP_l(l_ends + 1) = diff([0 n_mu_tot_l(l_mu_ends+1) - n_RA_tot_l(l_ends + 1)]);
    negIdx = find(N_GP_l(l_ends + 1) < 0);
    N_GP_l(l_ends(negIdx-1) + 1) = N_GP_l(l_ends(negIdx-1) + 1) + N_GP_l(l_ends(negIdx) + 1);
    N_GP_l(l_ends(negIdx) + 1) = 0;
    n_RA_end = N_offset + n_RA_tot_l(end) + sum(N_GP_l);
    n_mu_end = n_mu_tot_l(end);
    N_GP_l(end) = (n_mu_end * ceil(n_RA_end / n_mu_end)) - n_RA_end;        
    
    % Calculate the number of samples per subframe
    samplesPerSubframe = (sr_nominal * 1e-3);

    % For long sequences (LRA=839), establish the adjustment to the number
    % of subframes 'aSF' required to account for cases where the starting
    % subframe of the PRACH preamble occurs partway through the nominal
    % PRACH slot period
    if (LRA==839)
        subframesPerSlot = prach.SubframesPerPRACHSlot(1);
        slotsPerPeriod = prach.PRACHSlotsPerPeriod(1);
        aSF = getSubframeAdjustment(prach,configTableRow,subframesPerSlot,slotsPerPeriod,straddleSF);
        if (aSF >= 0)
            % Add the appropriate number of extra samples to 'N_offset'
            N_offset = N_offset + (aSF * samplesPerSubframe);
        else
            % If 'aSF' is negative, this signals that a waveform shorter
            % than the nominal PRACH slot period should be produced for the
            % current (inactive) PRACH slot, to balance PRACH slots with
            % extra subframes against the nominal PRACH slot period. The
            % OFDM information here results in an empty waveform of
            % duration 'N_offset' being produced by nrPRACHOFDMModulate
            N_offset = (subframesPerSlot + straddleSF + aSF) * samplesPerSubframe;
            N_RA_CP_l = zeros(1,0);
            N_GP_l = zeros(1,0);
        end
    end
    
    % Calculate carrier OFDM information
    carrierinfo = nr5g.internal.OFDMInfo(carrier,struct());
    
    % Determine ratio 'R' between carrier sampling rate and nominal
    % sampling rate
    R = carrierinfo.SampleRate / sr_nominal;
    
    % Create OFDM information output structure w.r.t. carrier sampling rate
    info.NSubcarriers = sizK;
    nfft = n_idft_nominal * R;
    info.Nfft = nfft;
    info.SubcarrierSpacing = prach.SubcarrierSpacing * 1e3;
    info.SampleRate = carrierinfo.SampleRate;
    cpLengths = N_RA_CP_l * R;
    cpLengthsPhaseComp = N_RA_CP_l_phasecomp * R;
    info.CyclicPrefixLengths = cpLengths;
    guardLengths = N_GP_l * R;
    info.GuardLengths = guardLengths;
    info.OffsetLength = N_offset * R;
    if (isempty(opts.Windowing))
        info.Windowing = defaultWindowing(prach.FrequencyRange,LRA,prachFormat,deltaf_RA,nfft);
    else
        coder.internal.errorIf(opts.Windowing>nfft, ...
            'nr5g:nrOFDM:WindowingTooLarge',opts.Windowing,nfft);
        info.Windowing = opts.Windowing;
    end
    info.Resampling = carrierinfo.Resampling;

    % Calculate time-domain frequency shift needed in case of an odd number
    % of subcarriers in the PRACH grid
    freqShift = 0;
    if (mod(sizK,2))
        % TS 38.211 Section 5.3.2
        K = carrierSCS/prach.SubcarrierSpacing;
        % The number of subcarriers is increased by one, as the OFDM
        % modulaton requires it to be even
        sizKplus1 = sizK + 1;
        freqShift = prach.SubcarrierSpacing*1e3*(sizKplus1-K*carrierNRB*12)/2; % Hz
    end
    info.FrequencyShift = freqShift;
    
    % Calculate symbol phase compensation corresponding to the frequency
    % shift above, ensuring that the phase compensation is held constant
    % across each whole preamble (detected by inspecting CP lengths). Note
    % that phase compensation for preamble P must consider CP lengths for
    % preambles 1:P and guard lengths for preambles 1:P-1.
    hasCP = find(cpLengthsPhaseComp).'; % OFDM symbol(s) that contain CP
    p = [hasCP(:,1) [hasCP(2:end,1)-1; numel(cpLengthsPhaseComp)]]; % First and last symbol of each preamble: each row corresponds to one preamble
    cpPlusGuardLengths = cpLengthsPhaseComp + [0 guardLengths(1:end-1)];
    symbolPhases = nr5g.internal.OFDMPhaseCompensation(nfft,cpPlusGuardLengths,prach.SubcarrierSpacing,freqShift);
    for r = 1:size(p,1)
        symbolPhases(p(r,1):p(r,2)) = symbolPhases(p(r,1));
    end
    info.SymbolPhases = symbolPhases;
    
end

% For long sequences (LRA=839), establish the adjustment to the number of
% subframes 'aSF' required to account for cases where the starting subframe
% of the PRACH preamble occurs partway through the nominal PRACH slot
% period. The nominal periods are defined here as intervals of
% 'subframesPerSlot' duration, starting at subframe 0 of frame n_SFN mod x
% = 0 where 'x' is given by the PRACH configuration table. Note that 'aSF'
% can be negative, indicating by how many subframes an inactive PRACH slot
% should be shortened. Example 2 for nrPRACHOFDMInfo above shows the OFDM
% information for NPRACHSlot = 0, 1 and 2. The corresponding values of
% 'aSF' produced by getSubframeAdjustment are 1, -3 and 2.
function aSF = getSubframeAdjustment(prach,configTableRow,subframesPerSlot,slotsPerPeriod,straddleSF)

    % Determine the set of subframe numbers where PRACH preambles start
    % within the period of 'x' frames
    y = configTableRow.y;
    x = configTableRow.x;
    startSFs = configTableRow.sfn;
    startSFs = startSFs + (y * 10);
    startSFs = startSFs(:) + (x * 10 * (0:(subframesPerSlot-1)));
    startSFs = startSFs(:).';
    
    % Determine the starting subframes of the nominal PRACH slots    
    nominalSFs = [0 cumsum(repmat(subframesPerSlot,1,slotsPerPeriod-1))];
    
    % Establish which nominal PRACH slots are active, and their 
    % corresponding PRACH slot indices
    activeSlotFn = @(n)any(startSFs>=n & startSFs<=(n+subframesPerSlot-1));
    activePRACHSlot = arrayfun(activeSlotFn,nominalSFs);
    activeIndex = find(activePRACHSlot);
    
    % Adjust the number of subframes in the PRACH slots:
    % (a) where the starting subframe of the PRACH preamble occurs 
    %     partway through the nominal PRACH slot period, and therefore the 
    %     PRACH slot needs lengthened to fully span the preamble
    % (b) where the lengthening of PRACH slots according to (a) above
    %     results in the PRACH slot timeline running behind the nominal
    %     timeline, such that a PRACH preamble cannot start "on time". In 
    %     this case, an earlier inactive PRACH slot needs to be shortened
    slotSFs = nominalSFs;
    aSF = zeros(size(slotSFs));
    % For each active PRACH slot index
    for i = 1:numel(activeIndex)
        % (a) Calculate extra slots required
        idxa = activeIndex(i);
        delta = startSFs(i) - slotSFs(idxa);
        if (delta < 0)
            % (b) If the number of extra slots is negative, adjust the
            % slot after the previous active PRACH slot to be empty
            idxb = activeIndex(i-1) + 1;
            [slotSFs,aSF] = adjust(slotSFs,aSF,idxb,-subframesPerSlot);
            % (a) Recalculate extra slots after step (b) above
            delta = startSFs(i) - slotSFs(idxa);
        end 
        % (a) Adjust the current active PRACH slot by adding extra slots
        [slotSFs,aSF] = adjust(slotSFs,aSF,idxa,delta);
    end
    
    % Adjust additional PRACH slots to ensure that the total number of 
    % subframes matches the nominal period
    if (straddleSF)
        % If the PRACH preamble straddles an extra subframe due to a
        % non-zero starting symbol, remove the extra subframe from inactive
        % PRACH slots and shorten the slot following active PRACH slots by
        % one subframe
        aSF(~activePRACHSlot) = -1;
        aSF(activeIndex + 1) = aSF(activeIndex + 1) - 1;
    else
        % Establish the number of extra subframes 'eSF' by which the 
        % combined duration of all PRACH slots exceeds the nominal period
        last = slotSFs(end) + aSF(end) + subframesPerSlot - 1;
        nominal_last = slotsPerPeriod*subframesPerSlot - 1;
        eSF = last - nominal_last;
        % If extra subframes are present
        if (eSF > 0)
            % If the last active PRACH slot is not the last slot of the
            % nominal period
            if(activeIndex(end) < numel(aSF))
                % Shorten the PRACH slot following the last active PRACH
                % slot by 'eSF'
                idx = activeIndex(end) + 1;
                aSF(idx) =  aSF(idx) - eSF;
            else % last active PRACH slot is the last slot
                % If the current PRACH slot is in the second or subsequent
                % nominal period
                if (prach.NPRACHSlot >= slotsPerPeriod)
                    % shorten the first PRACH slot by 'eSF'
                    aSF(1) =  aSF(1) - eSF;
                else % current PRACH slot is in the first nominal period
                    % The final active PRACH slot of the first nominal
                    % period will have 'eSF' extra subframes beyond the end
                    % of the nominal period. This is unavoidable, because
                    % they contain part of an active PRACH preamble. The
                    % duration of the first period is therefore the nominal
                    % period plus 'eSF'. For the second period and
                    % subsequent periods, the first slot is shortened by
                    % 'eSF' above to make space for these extra subframes,
                    % so the second period (and subsequent periods) will be
                    % of nominal length
                end
            end
        end
    end

    % Select the element of the 'aSF' vector that corresponds to the 
    % current PRACH slot
    aSF = aSF(mod(prach.NPRACHSlot,slotsPerPeriod) + 1);

end

% Adjust the PRACH slot timeline 'slotSFs' by adding 'extra' slots in
% positions idx+1 and beyond, and record the number of extra subframes in
% 'eSF'
function [slotSFs,eSF] = adjust(slotSFs,eSF,idx,extra)
    eSF(idx) = extra;
    slotSFs(idx+1:end) = slotSFs(idx+1:end) + eSF(idx);
end

% Default value for OFDM windowing. E = N_CP - W, scaled according to ratio
% between the input FFT size 'nfft' and the nominal FFT size, where N_CP, W
% and the nominal FFT size are given in TS 38.101-1 Annex F.5.5 and TS
% 38.101-2 Annex F.5.5. The value of E is the maximum amount of windowing
% and overlapping between adjacent OFDM symbols that can be applied while
% still maintaining the EVM measurement window W
function E = defaultWindowing(frequencyRange,LRA,prachFormat,deltaf_RA,nfft)

    coder.varsize('prachFormats',[1 9],[0 1]);
    coder.varsize('prachFormats{:}',[1 2],[0 1]);
    if (LRA==839)
        % TS 38.101-1 Table F.5.5-1
        prachFormats = {'0' '1' '2' '3'};
        N_CPs = [3168 21024 4688 3168];
        nfft_nominals = [24576 24576 24576 6144];
        Ws = [2307 20163 3827 2952];
        nfft_nominal = nfft_nominals(strcmpi(prachFormats,prachFormat));
    else % LRA==139,571,1151
        prachFormats = {'A1' 'A2' 'A3' 'B1' 'B2' 'B3' 'B4' 'C0' 'C2'};
        pow2mu = deltaf_RA / 15;
        if (strcmpi(frequencyRange,'FR1'))
            % TS 38.101-1 Table F.5.5-2
            N_CPs = [288 576 864 216 360 504 936 1240 2048] / pow2mu;
            nfft_nominal = 2048 / pow2mu;
            Ws = [144 432 720 72 216 360 792 1096 1904] / pow2mu;
        else % 'FR2'
            % TS 38.101-2 Table F.5.5-1
            N_CPs = [1152 2304 3456 864 1440 2016 3744 4960 8192] / pow2mu;
            nfft_nominal = 8192 / pow2mu;
            Ws = [576 1728 2880 288 864 1440 3168 4384 7616] / pow2mu;
        end
    end
    N_CP = N_CPs(strcmpi(prachFormats,prachFormat));
    W = Ws(strcmpi(prachFormats,prachFormat));
    
    E = floor((N_CP - W) * nfft / nfft_nominal);
    
end
