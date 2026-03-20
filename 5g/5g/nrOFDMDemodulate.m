function grid = nrOFDMDemodulate(varargin)
%nrOFDMDemodulate OFDM demodulation
%   GRID = nrOFDMDemodulate(CARRIER,WAVEFORM) performs OFDM demodulation of
%   the time domain waveform, WAVEFORM, given carrier configuration object
%   CARRIER.
%
%   CARRIER is a carrier configuration object, <a 
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these
%   object properties are relevant for this function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz (15, 30, 60, 120, 240, 480, 960)
%   CyclicPrefix      - Cyclic prefix ('normal', 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%   NSlot             - Slot number
%
%   WAVEFORM is a T-by-R matrix where T is the number of time-domain
%   samples in the waveform and R is the number of receive antennas. Only
%   complete OFDM symbols in WAVEFORM are demodulated. Any additional
%   samples corresponding to part of an OFDM symbol are discarded.
%
%   GRID is an array of size K-by-L-by-R where K is the number of
%   subcarriers and L is the number of OFDM symbols.
%
%   GRID = nrOFDMDemodulate(WAVEFORM,NRB,SCS,INITIALNSLOT) performs OFDM
%   demodulation of waveform WAVEFORM for NRB resource blocks with 
%   subcarrier spacing SCS and initial slot number INITIALNSLOT.
%
%   NRB is the number of resource blocks (1...275).
%
%   SCS is the subcarrier spacing in kHz (15, 30, 60, 120, 240, 480, 960).
%
%   INITIALNSLOT is the 0-based initial slot number, a non-negative scalar
%   integer. The function selects the appropriate cyclic prefix length for
%   the OFDM demodulation based on the initial slot number modulo the
%   number of slots per subframe.
%
%   GRID = nrOFDMDemodulate(...,NAME,VALUE) specifies additional options as
%   NAME,VALUE pairs to allow control over the OFDM demodulation:
%
%   CyclicPrefix         - Cyclic prefix ('normal' (default), 'extended').
%                          This option is only applicable for function
%                          syntaxes not using nrCarrierConfig
%   Nfft                 - Desired number of FFT points to use in the OFDM
%                          demodulator. If absent or set to [], a default 
%                          value is selected based on other parameters, see
%                          <a href="matlab: doc('nrOFDMDemodulate')"
%                          >nrOFDMDemodulate</a> for details
%   SampleRate           - Sample rate of input waveform. If absent or set
%                          to [], the default value is SampleRate = Nfft *
%                          SCS. If required, the input waveform is
%                          resampled from the specified sample rate to the
%                          sample rate used during OFDM demodulation, Nfft
%                          * SCS
%   CarrierFrequency     - Carrier frequency (in Hz) to calculate the phase
%                          decompensation applied for each OFDM symbol 
%                          (denoted f_0 in TS 38.211 Section 5.4). Default 
%                          is 0
%   CyclicPrefixFraction - Starting position of OFDM symbol demodulation 
%                          (FFT window position) within the cyclic prefix.
%                          Specified as a fraction of the cyclic prefix, in
%                          the range [0,1], with 0 representing the start
%                          of the cyclic prefix and 1 representing the end
%                          of the cyclic prefix. Default is 0.5
%
%   Note that for the numerologies specified in TS 38.211 Section 4.2, 
%   extended cyclic prefix length is only applicable for 60 kHz subcarrier
%   spacing.
%
%   % Example:
%
%   % Configure carrier for 20 MHz bandwidth
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 106;
%
%   % Configure PDSCH and create PDSCH DM-RS symbols and indices
%   pdsch = nrPDSCHConfig;
%   pdsch.NumLayers = 2;
%   sym = nrPDSCHDMRS(carrier,pdsch);
%   ind = nrPDSCHDMRSIndices(carrier,pdsch);
%
%   % Create a carrier resource grid and map PDSCH DM-RS symbols
%   txGrid = nrResourceGrid(carrier,pdsch.NumLayers);
%   txGrid(ind) = sym;
%
%   % Perform OFDM modulation
%   [txWaveform,info] = nrOFDMModulate(carrier,txGrid);
%
%   % Apply a simple 2-by-1 channel
%   H = [0.6; 0.4];
%   rxWaveform = txWaveform * H;
%
%   % Perform OFDM demodulation
%   rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
%
%   See also nrCarrierConfig, nrOFDMInfo, nrOFDMModulate, nrResourceGrid.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    narginchk(2,14);
    
    % Validate inputs and get OFDM information
    [waveform,info,nSlot,N,hasSampleRate] = validateInputs(varargin{:});
    
    % Perform OFDM demodulation
    grid = nr5g.internal.OFDMDemodulate(waveform,info,nSlot,N,hasSampleRate);
    
end

% Validate inputs
function [waveform,info,nSlot,N,hasSampleRate] = validateInputs(varargin)
    
    fcnName = 'nrOFDMDemodulate';
    
    isCarrierSyntax = isa(varargin{1},'nrCarrierConfig');
    if (isCarrierSyntax) % nrOFDMDemodulate(CARRIER,WAVEFORM,...)
        
        % Validate carrier input type
        carrier = varargin{1};
        validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},fcnName,'Carrier specific configuration object');
        
        % Get slot number
        nSlot = carrier.NSlot;
        
        % Get waveform
        waveform = varargin{2};
        
        % Parse options
        optNames = {'CarrierFrequency','Nfft','SampleRate','CyclicPrefixFraction'};
        opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{3:end});

        % If performing code generation, the presence of sample rate with the
        % function syntax using nrCarrierConfig triggers a compile-time error
        hasSampleRate = ~isempty(opts.SampleRate);
        coder.internal.errorIf(~coder.target('MATLAB') && hasSampleRate, ...
            'nr5g:nrOFDMDemodulate:CompilationCarrierSampleRate');
        
        % Get OFDM information
        info = nr5g.internal.OFDMInfo(carrier,opts);
        
    else % nrOFDMDemodulate(WAVEFORM,NRB,SCS,INITIALNSLOT,...)
        
        % Validate NRB
        NRB = varargin{2};
        validateattributes(NRB,{'numeric'},{'real','integer','scalar','>=',1,'<=',275},fcnName,'NRB');
        
        % Validate subcarrier spacing
        SCS = varargin{3};
        validateattributes(SCS,{'numeric'},{'real','integer','scalar'},fcnName,'SCS');
        validSCS = [15 30 60 120 240 480 960];
        coder.internal.errorIf(~any(SCS==validSCS),'nr5g:nrOFDMDemodulate:InvalidSCS',SCS,num2str(validSCS));
        
        % Validate slot number
        nSlot = varargin{4};
        validateattributes(nSlot,{'numeric'},{'real','nonnegative','scalar','integer'},fcnName,'INITIALNSLOT');
        
        % Get waveform
        waveform = varargin{1};
        
        % Parse options and get cyclic prefix length
        optNames = {'CyclicPrefix','CarrierFrequency','Nfft','SampleRate','CyclicPrefixFraction'};
        opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{5:end});
        ECP = strcmpi(opts.CyclicPrefix,'extended');
        
        % If performing code generation and 'SampleRate' is supplied, then
        % NRB, SCS, Nfft, and SampleRate must be constant at compile time.
        hasSampleRate = ~isempty(opts.SampleRate);
        coder.internal.errorIf(~coder.target('MATLAB') && hasSampleRate && ...
            (~coder.internal.isConst(NRB) || ~coder.internal.isConst(SCS) || ...
            ~coder.internal.isConst(opts.Nfft) || ~coder.internal.isConst(opts.SampleRate)), ...
            'nr5g:nrOFDMDemodulate:NonConstantNfftScsSampleRateNrb');
        
        % Get OFDM information
        info = nr5g.internal.OFDMInfo(NRB,SCS,ECP,opts);
        
    end
    
    % Validate waveform    
    validateattributes(waveform,{'double','single'},{},fcnName,'WAVEFORM');
    coder.internal.errorIf(~ismatrix(waveform),'nr5g:nrOFDMDemodulate:InvalidWaveformDims',ndims(waveform));
    
    % Calculate the total number of samples in the waveform
    T = size(waveform,1);
    
    % Establish the total number of subframes spanned by 'T', rounded up,
    % which determines the required number of repetitions of the cyclic
    % prefix lengths, and calculate the corresponding number of OFDM 
    % symbols 'N'
    samplesPerSubframe = info.SampleRate * 1e-3;
    nSubframes = ceil(T / samplesPerSubframe);
    N = nSubframes * info.SymbolsPerSlot * info.SlotsPerSubframe;
    
    % Update cyclic prefix lengths to span all subframes at least partially
    % spanned by the waveform, and to take into consideration the
    % initial slot number
    cpLengths = nr5g.internal.OFDMInfoRelativeNSlot(info,nSlot,max(N,1));
    
    % Establish the actual number of OFDM symbols in the waveform,
    % adjusting symbol end times if resampling is required during OFDM 
    % demodulation
    symbolLengths = cpLengths + info.Nfft;
    symbolEnds = cumsum(repmat(symbolLengths,1,max(nSubframes,1)));
    r = info.Resampling;
    if (any([r.L r.M]~=1))
        symbolEnds = ceil(symbolEnds * r.L / r.M);
    end
    N = find(T>=symbolEnds,1,'last');
    coder.internal.errorIf(isempty(N),'nr5g:nrOFDMDemodulate:TooFewWaveformSamples',T,symbolEnds(1));
    
end
