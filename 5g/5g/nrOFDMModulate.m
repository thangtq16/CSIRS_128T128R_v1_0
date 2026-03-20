function [waveform,info] = nrOFDMModulate(varargin)
%nrOFDMModulate OFDM modulation
%   [WAVEFORM,INFO] = nrOFDMModulate(CARRIER,GRID) performs OFDM modulation
%   of a carrier resource array, GRID, given carrier configuration object
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
%   GRID is a complex K-by-N-by-P array, where K is the number of
%   subcarriers, N is the number of OFDM symbols and P is the number of
%   transmit antennas.
%
%   WAVEFORM is a T-by-P matrix where T is the number of time-domain 
%   samples in the waveform.
%
%   INFO is a structure containing the fields:
%
%   Nfft                - Number of IFFT points used in the OFDM modulator
%   SampleRate          - Sample rate of the OFDM modulated waveform
%   CyclicPrefixLengths - Cyclic prefix length (in samples) of each OFDM 
%                         symbol in a subframe, starting at slot 0
%   SymbolLengths       - Total length (in samples) of each OFDM symbol in 
%                         a subframe, including the cyclic prefix and 
%                         starting at slot 0
%   Windowing           - Number of time-domain samples over which 
%                         windowing and overlapping of OFDM symbols is 
%                         applied
%   SymbolPhases        - Phase precompensation applied for each OFDM 
%                         symbol due to the phase term per OFDM symbol in 
%                         TS 38.211 Section 5.4. <a
%                         href="matlab:help('nrOFDMModulate')"
%                         >nrOFDMModulate</a> applies
%                         this precompensation during modulation and
%                         <a href="matlab:help('nrOFDMDemodulate')"
%                         >nrOFDMDemodulate</a> performs decompensation
%                         during demodulation.
%   SymbolsPerSlot      - Number of OFDM symbols in a slot
%   SlotsPerSubframe    - Number of slots in a 1 ms subframe
%   SlotsPerFrame       - Number of slots in a 10 ms frame
%
%   Note that the number of samples in the INFO.CyclicPrefixLengths,
%   INFO.SymbolLengths, and INFO.Windowing fields apply to the sample rate
%   of the IFFT of size INFO.Nfft used during OFDM symbol construction.
%   This may be different from the sample rate of the waveform in the case
%   that the 'SampleRate' NAME,VALUE pair below is specified. Note also 
%   that the IFFT size can be specified using the 'Nfft' NAME,VALUE pair.  
%
%   [WAVEFORM,INFO] = nrOFDMModulate(GRID,SCS,INITIALNSLOT) performs OFDM
%   modulation of carrier resource array GRID with subcarrier spacing SCS
%   and initial slot number INITIALNSLOT.
% 
%   SCS is the subcarrier spacing in kHz (15, 30, 60, 120, 240, 480, 960).
% 
%   INITIALNSLOT is the 0-based initial slot number, a non-negative scalar
%   integer. The function selects the appropriate cyclic prefix length for
%   the OFDM modulation based on the initial slot number modulo the number
%   of slots per subframe.
%
%   [WAVEFORM,INFO] = nrOFDMModulate(...,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the OFDM modulation:
%
%   CyclicPrefix        - Cyclic prefix ('normal' (default), 'extended').
%                         This option is only applicable for function
%                         syntaxes not using nrCarrierConfig
%   Nfft                - Desired number of IFFT points to use in the OFDM
%                         modulator. If absent or set to [], a default 
%                         value is selected based on other parameters, see
%                         <a href="matlab: doc('nrOFDMModulate')"
%                         >nrOFDMModulate</a> for details
%   SampleRate          - Desired sample rate of the OFDM modulated 
%                         waveform. If absent or set to [], the default
%                         value is SampleRate = Nfft * SCS. If required,
%                         the OFDM modulated waveform is resampled to this
%                         sample rate after OFDM symbol construction
%   Windowing           - Number of time-domain samples over which
%                         windowing and overlapping of OFDM symbols is 
%                         applied. If absent or set to [], a default value
%                         is selected based on other parameters, see
%                         <a href="matlab: doc('nrOFDMModulate')"
%                         >nrOFDMModulate</a> for details
%   CarrierFrequency    - Carrier frequency (in Hz) to calculate the phase 
%                         precompensation applied for each OFDM symbol 
%                         (denoted f_0 in TS 38.211 Section 5.4). Default 
%                         is 0
%
%   Note that for the numerologies specified in TS 38.211 Section 4.2,
%   extended cyclic prefix length is only applicable for 60 kHz subcarrier
%   spacing. Note that the number of samples specified in the 'Windowing'
%   option applies to the IFFT of size INFO.Nfft used during OFDM symbol
%   construction.
%
%   % Example 1:
%   % Perform OFDM modulation of a one-slot resource grid.
%
%   % Configure carrier for 20 MHz bandwidth
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 106;
%
%   % Create a carrier resource grid and fill it with random 16QAM symbols
%   grid = nrResourceGrid(carrier);
%   sym = nrSymbolModulate(randi([0 1],numel(grid)*4,1),'16QAM');
%   grid(:) = sym;
%
%   % Perform OFDM modulation
%   [waveform,info] = nrOFDMModulate(carrier,grid);
%
%   % Example 2:
%   % Perform OFDM modulation of resource grid containing SRS and spanning
%   % a whole frame.
%
%   % Configure carrier for 10 MHz bandwidth, 30 kHz subcarrier spacing
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 24;
%   carrier.SubcarrierSpacing = 30;
%
%   % Configure SRS and create empty resource grid
%   srs = nrSRSConfig;
%   srs.SRSPeriod = [2 0];
%   frameGrid = [];
%
%   % Get OFDM information
%   ofdmInfo = nrOFDMInfo(carrier);
%
%   % Create slot resource grid and concatenate to produce frame resource 
%   % grid
%   for nSlot = 0:(ofdmInfo.SlotsPerFrame-1)
%   
%       carrier.NSlot = nSlot;
%       slotGrid = nrResourceGrid(carrier);
%       srsInd = nrSRSIndices(carrier,srs);
%       srsSym = nrSRS(carrier,srs);
%       slotGrid(srsInd) = srsSym;
%       frameGrid = [frameGrid slotGrid];
%
%   end
%
%   % Perform OFDM modulation
%   waveform = nrOFDMModulate(carrier,frameGrid);
%
%   See also nrCarrierConfig, nrOFDMInfo, nrOFDMDemodulate, nrResourceGrid.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,13);
    
    % Validate inputs and get OFDM information
    [grid,internalinfo,nSlot,hasSampleRate] = validateInputs(varargin{:});
    
    % Perform OFDM modulation (including windowing and overlapping)
    overlapping = true;
    waveform = nr5g.internal.OFDMModulate(grid,internalinfo,nSlot,overlapping,hasSampleRate);
    
    % Create OFDM information output structure
    info = nr5g.internal.OFDMInfoOutput(internalinfo);
    
end

% Validate inputs
function [grid,info,nSlot,hasSampleRate] = validateInputs(varargin)
    
    fcnName = 'nrOFDMModulate';    
    
    isCarrierSyntax = isa(varargin{1},'nrCarrierConfig');
    if (isCarrierSyntax) % nrOFDMModulate(CARRIER,GRID,..)

        % Gather inputs other than GRID from remote
        if coder.target('MATLAB')
            if nargin > 2
                [varargin{3:nargin}] = gather(varargin{3:nargin});
            end
        end
        
        % Validate carrier
        carrier = varargin{1};
        validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},fcnName,'Carrier specific configuration object');

        % Get slot number
        nSlot = carrier.NSlot;
        
        % Parse options
        optNames = {'Nfft','SampleRate','Windowing','CarrierFrequency'};
        opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{3:end});
        
        % Get OFDM information
        info = nr5g.internal.OFDMInfo(carrier,opts);
        
        % Validate grid subcarriers
        grid = varargin{2};
        validateattributes(grid,{'double','single'},{'3d'},fcnName,'GRID');
        Kgrid = size(grid,1);
        Kinfo = info.NSubcarriers;
        coder.internal.errorIf(Kgrid~=Kinfo,'nr5g:nrOFDMModulate:InvalidGridSubcarriers',Kgrid,Kinfo);

        % If performing code generation, the presence of sample rate with the
        % function syntax using nrCarrierConfig triggers a compile-time error
        hasSampleRate = ~isempty(opts.SampleRate);
        coder.internal.errorIf(~coder.target('MATLAB') && hasSampleRate, ...
            'nr5g:nrOFDMModulate:CompilationCarrierSampleRate');
        
    else % nrOFDMModulate(GRID,SCS,INITIALNSLOT,...)

        % Validate that at least three inputs have been provided
        narginchk(3,13);

        % Gather inputs other than GRID from remote
        if coder.target('MATLAB')
            if nargin > 1
                [varargin{2:nargin}] = gather(varargin{2:nargin});
            end
        end

        % Validate grid subcarriers and calculate NRB
        grid = varargin{1};
        validateattributes(grid,{'double','single'},{'3d'},fcnName,'GRID');
        K = size(grid,1);
        coder.internal.errorIf(mod(K,12)~=0,'nr5g:nrOFDMModulate:InvalidGridSubcarriersMod12',K);
        NRB = K / 12;
        coder.internal.errorIf(NRB<1 || NRB>275,'nr5g:nrOFDMModulate:InvalidGridSubcarrierSize',K);
        
        % Validate subcarrier spacing
        SCS = varargin{2};
        validateattributes(SCS,{'numeric'},{'real','integer','scalar'},fcnName,'SCS');
        validSCS = [15 30 60 120 240 480 960];
        coder.internal.errorIf(~any(SCS==validSCS),'nr5g:nrOFDMModulate:InvalidSCS',SCS,num2str(validSCS));
        
        % Validate slot number
        nSlot = varargin{3};
        validateattributes(nSlot,{'numeric'},{'real','nonnegative','scalar','integer'},fcnName,'INITIALNSLOT');
        
        % Parse options and get cyclic prefix length
        optNames = {'CyclicPrefix','Nfft','SampleRate','Windowing','CarrierFrequency'};
        opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{4:end});
        ECP = strcmpi(opts.CyclicPrefix,'extended');
        
        % If performing code generation and 'SampleRate' is supplied, then
        % NRB, SCS, Nfft, and SampleRate must be constant at compile time.
        hasSampleRate = ~isempty(opts.SampleRate);
        coder.internal.errorIf(~coder.target('MATLAB') && hasSampleRate && ...
            (~coder.internal.isConst(NRB) || ~coder.internal.isConst(SCS) || ...
            ~coder.internal.isConst(opts.Nfft) || ~coder.internal.isConst(opts.SampleRate)), ...
            'nr5g:nrOFDMModulate:NonConstantNfftScsSampleRateNrb');
        
        % Get OFDM information
        info = nr5g.internal.OFDMInfo(NRB,SCS,ECP,opts);
        
    end
    
    
    % Validate grid OFDM symbols
    N = size(grid,2);
    coder.internal.errorIf(N==0,'nr5g:nrOFDMModulate:InvalidGridOFDMSymbolSize');
    
end
