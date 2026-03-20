function info = OFDMInfo(varargin)
    %OFDMInfo OFDM modulation related information
    %
    %   Note: This is an internal undocumented function and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   INFO = OFDMInfo(NRB,SCS,ECP,OPTS) provides dimensional information
    %   related to OFDM modulation for number of resource blocks NRB (1...275),
    %   subcarrier spacing SCS (15,30,60,120,240,480,960), flag ECP indicating
    %   extended (true) or normal (false) cyclic prefix length and
    %   options structure OPTS which may contain the following optional fields:
    %   Nfft                 - Desired number of IFFT points to use in the OFDM
    %                          modulator
    %   SampleRate           - Desired sample rate of the OFDM modulated
    %                          waveform
    %   Windowing            - Number of time-domain samples over which
    %                          windowing and overlapping of OFDM symbols is
    %                          applied
    %   CarrierFrequency     - Carrier frequency in Hz, denoted f_0 in
    %                          TS 38.211 Section 5.4
    %   CyclicPrefixFraction - FFT window position within the cyclic prefix
    %
    %   INFO = OFDMInfo(CARRIER,OPTS) provides the same information given
    %   carrier configuration object CARRIER.

    %   Copyright 2019-2024 The MathWorks, Inc.

    %#codegen
    persistent cache

    isMATLAB = isempty(coder.target);

    if isMATLAB
        % Gather values from GPU in order to generate hash key
        [varargin{:}] = gather(varargin{:});
        
        hk = keyHash(varargin);
        if isempty(cache)
            cache = configureDictionary("uint64","struct");
        elseif isKey(cache,hk)
            info = cache(hk);
            return;
        end
    end

    if (nargin==2)
        carrier = varargin{1};
        NRB = carrier.NSizeGrid;
        SCS = carrier.SubcarrierSpacing;
        ECP = strcmpi(carrier.CyclicPrefix,'extended');
        opts = varargin{2};
    else
        NRB = varargin{1};
        SCS = varargin{2};
        ECP = varargin{3};
        opts = varargin{4};
    end

    % Subcarrier spacing configuration
    % TS 38.211 Section 4.3.2
    SCS = double(SCS);
    mu = log2(SCS / 15);

    % Total number of subcarriers in the resource grid
    % TS 38.211 Section 4.4.4.1
    NRB = double(NRB);
    K = NRB * 12;

    % Transmission bandwidth
    tx_bw = K * SCS * 1e3;

    userSampleRate = isfield(opts,'SampleRate') && ~isempty(opts.SampleRate);
    userNfft = isfield(opts,'Nfft') && ~isempty(opts.Nfft);
    if (userSampleRate)

        % OFDM sample rate is given by input option
        SR = opts.SampleRate;

        if (userNfft)

            % IDFT size is given by input option
            nfft = opts.Nfft(1);

        else

            % Choose IDFT size based on these rules:
            %   * must result in integer cyclic prefix lengths
            %     -> must be an integer multiple of 128 for normal cyclic
            %        prefix length (cyclic prefix lengths for nfft=128
            %        are {9,10}, {9,11}, {9,13}, {9,17} and {9,25} for
            %        mu = 0...4 respectively, and the lengths in each of
            %        those sets have no common factors). nfft=128 is also
            %        a reasonable minimum unit for extended CP, although
            %        any multiple of 4 would create integer cyclic prefix
            %        lengths for any mu
            %     -> nfft = 128 * Y
            %   * lower bound on Y is given by maximum occupancy
            %     (K / nfft) of 85%
            %   * upper bound on Y is ceil((2 * SR / (SCS * 1e3)) / 128)
            %   * select Y to maximize gcd(SCS * 1e3 * nfft,SR)
            min_nfft = ceil((K / 0.85) / 128) * 128;
            max_nfft = max(ceil((2 * SR / (SCS * 1e3)) / 128) * 128,min_nfft);
            nffts = min_nfft:128:max_nfft;
            g = gcd(SCS * 1e3 * nffts,SR);
            idx = find(g==max(g));
            idx = idx(1);
            nfft = nffts(idx);

        end

    else

        if (userNfft)

            % IDFT size is given by input option
            nfft = opts.Nfft(1);

        else

            % Choose IDFT size based on three rules:
            %   * power of 2 size
            %   * maximum occupancy (K / nfft) of 85%
            %   * minimum of 128 (so that cyclic prefix lengths are
            %     always integer)
            nfft = max(power(2,ceil(log2(K / 0.85))),128);

        end

        % OFDM sample rate given by subcarrier spacing 'SCS' and IDFT
        % size 'Nfft'
        SR = SCS * 1e3 * nfft;

    end

    % Verify that the user-specified IDFT size is sufficient for the
    % number of subcarriers K
    if (userNfft)
        coder.internal.errorIf(nfft<K, ...
            'nr5g:nrOFDM:NfftTooSmall',nfft,K);
    end

    % OFDM symbols per slot
    % TS 38.211 Section 4.3.2
    if ECP
        L = 12;
    else
        L = 14;
    end

    % Slots per subframe
    % TS 38.211 Table 4.3.2-1
    slotsPerSubframe = 2^mu;

    % Nominal cyclic prefix lengths in numerology mu=0 and in units of T_s
    % TS 38.211 Section 5.3.1
    if ECP
        N_CP = 512;
    else
        N_CP = 144;
    end

    % Adjust for IDFT size in use
    N_CP = N_CP / 2048 * nfft;

    % Create vector of cyclic prefix lengths across a subframe and adjust
    % cyclic prefix lengths at start of each half subframe
    N = L * slotsPerSubframe;
    cpLength = N_CP*ones(1,N/2-1);
    cpLengthStart = (SCS * nfft - (N*nfft + (N-2)*N_CP)) / 2;
    cpLengths = repmat([cpLengthStart, cpLength],1,2);

    % For user-specified sample rate or IDFT size, verify that cyclic
    % prefix lengths are integer
    if (userSampleRate || userNfft)
        cpLengthMod1 = mod(cpLengths,1);
        if (any(cpLengthMod1))
            i = find(cpLengthMod1,1,'first');
            i = i(1);
            coder.internal.error( ...
                'nr5g:nrOFDM:NonIntegerCPLengths', ...
                nfft,i,sprintf('%g',cpLengths(i)));
        end
    end

    info = coder.internal.constantPreservingStruct('NSubcarriers',K,'Nfft',nfft, ...
        'SubcarrierSpacing',SCS*1e3,'SampleRate',SR,'CyclicPrefixLengths',cpLengths, ...
        'SymbolsPerSlot',L,'SlotsPerSubframe',slotsPerSubframe);

    % The 'Windowing', 'SymbolPhases', and 'CyclicPrefixFraction' fields
    % need to exist in 'info' output for code generation even if they are
    % not fields in input 'opts'. Otherwise, setting the stickyStruct using
    % local function setStruct does not work. Default is [].
    if (isfield(opts,'Windowing'))
        if (isempty(opts.Windowing))
            windowing = defaultWindowing(SCS,NRB,ECP,nfft);
        else
            windowing = opts.Windowing(1);
            coder.internal.errorIf(windowing>nfft, ...
                'nr5g:nrOFDM:WindowingTooLarge',windowing,nfft);
        end
        info = setStruct(info,'Windowing',windowing);
    else
        info = setStruct(info,'Windowing',[]);
    end

    if (isfield(opts,'CarrierFrequency'))
        symbolPhases = nr5g.internal.OFDMPhaseCompensation(nfft,cpLengths,SCS,opts.CarrierFrequency);
        info = setStruct(info,'SymbolPhases', ...
            symbolPhases);
    else
        info = setStruct(info,'SymbolPhases',[]);
    end

    if (isfield(opts,'CyclicPrefixFraction'))
        info = setStruct(info,'CyclicPrefixFraction',opts.CyclicPrefixFraction);
    else
        info = setStruct(info,'CyclicPrefixFraction',[]);
    end

    % Calculate filter design parameters for resampling between FFT sample
    % rate and desired sample rate, if required
    aStop = 70;
    fftSR = SCS * 1e3 * nfft;
    if (~isequal(fftSR,SR))

        coder.internal.errorIf(SR <= tx_bw, ...
            'nr5g:nrOFDM:SampleRateTooSmall',sprintf('%d',int32(SR)), ...
            NRB,SCS,sprintf('%d',int32(tx_bw)));

        coder.internal.errorIf(userNfft && (fftSR == tx_bw), ...
            'nr5g:nrOFDM:CriticallySampled',NRB,K,nfft);

        g = gcd(fftSR,SR);
        L = SR / g;
        M = fftSR / g;
        maxLM = max([L M]);
        if (L > M)
            R = (fftSR - tx_bw) / fftSR;
        else
            R = (SR - tx_bw) / SR;
        end
        TW = 2 * R / maxLM;
    else
        L = 1;
        M = 1;
        TW = 1;
    end
    info = setStruct(info,'Resampling', ...
        coder.internal.constantPreservingStruct('L',L,'M',M, ...
        'TW',TW,'AStop',aStop));

    if isMATLAB
        cache(hk) = info;
    end
end

% Default value for OFDM windowing. E = N_CP - W, scaled according to ratio
% between the input FFT size 'nfft' and the nominal FFT size, where N_CP, W
% and the nominal FFT size are given in TS 38.104 Annexes B.5.2 and C.5.2,
% TS 38.101-1 Annexes F.5.3 and F.5.4, and TS 38.101-2 Annexes F.5.3 and
% F.5.4. The value of E is the maximum amount of windowing and overlapping
% between adjacent OFDM symbols that can be applied while still maintaining
% the EVM measurement window W. Where different documents result in
% different values for a given configuration, the lower value of E is used
function E = defaultWindowing(SCS,NRB,ECP,nfft)

    switch (SCS)

        case 15
            % CBW =         [5   10   15   20   25   30   35   40   45   50] MHz
            NRBs =          [25  52   79   106  133  160  188  216  242  270];
            nfft_nominals = [512 1024 1536 2048 2048 3072 3072 4096 4096 4096];
            N_CPs =         [36  72   108  144  144  216  216  288  288  288];
            % TS 38.101-1 Table F.5.3-1
            Ws101 =         [18  36   54   72   72   108  108  144  144  144];
            % TS 38.104 Table B.5.2-1
            Ws104 =         [14  28   44   58   72   108  108  144  144  144];
            Ws = [Ws101; Ws104];
        case 30
            % CBW =         [5   10  15  20   25   30   35   40   45   50   60   70   80   90   100] MHz
            NRBs =          [11  24  38  51   65   78   92   106  119  133  162  189  217  245  273];
            nfft_nominals = [256 512 768 1024 1024 1536 1536 2048 2048 2048 3072 3072 4096 4096 4096];
            N_CPs =         [18  36  54  72   72   108  108  144  144  144  216  216  288  288  288];
            % TS 38.101-1 Table F.5.3-2
            Ws101 =         [9   18  27  36   36   54   54   72   72   72   108  108  144  144  144];
            % TS 38.104 Table B.5.2-2
            Ws104 =         [8   14  22  28   36   54   54   72   72   72   130  130  172  172  172];
            Ws = [Ws101; Ws104];
        case 60
            % CBW =         [10  15  20  25  30    35   40    45   50   60   70   80   90   100  200] MHz
            % NOTE: For 100 MHz, NRB=135 for FR1, 132 for FR2
            NRBs =          [11  18  24  31  38    44   51    58   65   79   93   107  121  135  264];
            nfft_nominals = [256 384 512 512 768   768  1024  1024 1024 1536 1536 2048 2048 2048 4096];
            if (ECP)
                N_CPs =     [64  96  128 128 192   192  256   256  256  384  384  512  512  512  1024];
                % TS 38.101-1 Table F.5.4-1 / TS 38.101-2 Table F.5.4-1
                Ws101 =     [54  80  106 110 164   164  220   220  220  330  330  440  440  440  880];
                % TS 38.104 Table B.5.2-4
                Ws104FR1 =  [54  80  106 110 164   164  220   220  220  340  340  454  454  454  NaN];
                % TS 38.104 Table C.5.2-3
                Ws104FR2 =  [NaN NaN NaN NaN NaN   NaN  NaN   NaN  220  NaN  NaN  NaN  NaN  440  880];
            else
                N_CPs =     [18  27  36  36  54    54   72    72   72   108  108  144  144  144  288];
                % TS 38.101-1 Table F.5.3-3 / TS 38.101-2 Table F.5.3-1
                Ws101 =     [9   14  18  18  27    27   36    36   36   54   54   72   72   72   144];
                % TS 38.104 Table B.5.2-3
                Ws104FR1 =  [8   11  14  18  26    26   36    36   36   64   64   86   86   86   NaN];
                % TS 38.104 Table C.5.2-1
                Ws104FR2 =  [NaN NaN NaN NaN NaN   NaN  NaN   NaN  36   NaN  NaN  NaN  NaN  72   144];
            end
            Ws = [Ws101; Ws104FR1; Ws104FR2];
        case 120
            % CBW =         [50  100  200  400] MHz
            NRBs =          [32  66   132  264];
            nfft_nominals = [512 1024 2048 4096];
            N_CPs =         [36  72   144  288];
            % TS 38.101-2 Table F.5.3-2
            Ws101 =         [18  36   72   144];
            % TS 38.104 Table C.5.2-2
            Ws104 =         [18  36   72   144];
            Ws = [Ws101; Ws104];
        case 480
            % CBW =         [400  800 1600] MHz
            NRBs =          [66   124  248];
            nfft_nominals = [1024 2048 4096];
            N_CPs =         [72   144  288];
            % TS 38.104 Table C.5.2-2a
            Ws104 =         [36   72   144];
            Ws = Ws104;
        case 960
            % CBW =         [400 800  1600 2000] MHz
            NRBs =          [33  62   124  148];
            nfft_nominals = [512 1024 2048 2048];
            N_CPs =         [36  72   144  144];
            % TS 38.104 Table C.5.2-2b
            Ws104 =         [18  36   72   72];
            Ws = Ws104;
        otherwise
            % No windowing specified in 3GPP docs for 240 kHz (SSB only)
            % No windowing for any other SCS
            NRBs = 0;
            nfft_nominals = nfft;
            N_CPs = 0;
            Ws = 0;
    end

    [~,idx] = min(abs(NRBs - NRB));
    idx = idx(1);
    N_CP = N_CPs(idx);
    nfft_nominal = nfft_nominals(idx);
    W = max(Ws(:,idx),[],'omitnan');

    E = floor((N_CP - W) * nfft / nfft_nominal);

end

function out = setStruct(in,name,value)
    coder.internal.prefer_const(name,value)
    if ~coder.internal.isCompiled
        out = in;
        out.(name) = value;
    else
        out = set(in,name,value);
    end
end
