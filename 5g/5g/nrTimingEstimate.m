function [offset,mag] = nrTimingEstimate(varargin)
%nrTimingEstimate Practical timing estimation
%   [OFFSET,MAG] = nrTimingEstimate(...,WAVEFORM,...) performs timing
%   estimation by cross-correlating known reference signals with the input
%   waveform WAVEFORM, a T-by-Nr matrix where T is the number of time
%   domain samples and Nr is the number of receive antennas. OFFSET is a
%   scalar indicating the estimated timing offset, an integer number of
%   samples relative to the first sample of the input waveform. MAG is a
%   T-by-Nr matrix giving the estimated impulse response magnitude for each
%   antenna in the input waveform.
%
%   [OFFSET,MAG] = nrTimingEstimate(CARRIER,WAVEFORM,REFIND,REFSYM)
%   performs timing estimation by correlating WAVEFORM with a reference
%   waveform. The reference waveform is created by OFDM modulating a
%   reference resource grid containing the symbols given by REFSYM at the
%   locations given by REFIND and according to the carrier configuration
%   given by CARRIER.
%   
%   CARRIER is a carrier configuration object, <a 
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these
%   object properties are relevant for this function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15, 30, 60, 120, 240, 480, 960)
%   CyclicPrefix      - Cyclic prefix ('normal', 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%   NSlot             - Slot number
%   
%   REFIND and REFSYM are the reference signal indices and symbols
%   respectively. REFIND is an array of 1-based linear indices addressing a
%   K-by-L-by-P resource array. K is the number of subcarriers, given by
%   CARRIER.NSizeGrid * 12. L is the number of OFDM symbols in one slot,
%   given by CARRIER.SymbolsPerSlot. P is the number of reference signal
%   ports and is inferred from the range of values in REFIND.
%
%   [OFFSET,MAG] = nrTimingEstimate(CARRIER,WAVEFORM,REFGRID) specifies a
%   predefined reference resource grid in REFGRID. REFGRID is an array with
%   nonzero elements representing the reference symbols in their
%   appropriate locations. It is of size K-by-N-by-P, where N is the number
%   of OFDM symbols.
%
%   [OFFSET,MAG] = nrTimingEstimate(WAVEFORM,NRB,SCS,INITIALNSLOT,REFIND,REFSYM)
%   and [OFFSET,MAG] = nrTimingEstimate(WAVEFORM,NRB,SCS,INITIALNSLOT,REFGRID)
%   perform timing estimation as above, except in place of the CARRIER
%   configuration object the OFDM modulation spans NRB resource blocks,
%   uses SCS kHz subcarrier spacing and assumes that the initial slot
%   number is INITIALNSLOT. NRB must be in the range (1...275), SCS is the
%   subcarrier spacing in kHz (15, 30, 60, 120, 240, 480, 960). 
%   INITIALNSLOT is the 0-based initial slot number, a nonnegative scalar
%   integer. INITIALNSLOT modulo the number of slots per subframe determines
%   the appropriate cyclic prefix lengths for OFDM modulation of the 
%   reference resource grid containing the known reference signals.
%
%   [OFFSET,MAG] = nrTimingEstimate(...,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the OFDM modulation
%   of the reference resource grid:
%
%   CyclicPrefix     - Cyclic prefix ('normal' (default), 'extended'). This
%                      option is only applicable for function syntaxes not
%                      using nrCarrierConfig
%   Nfft             - Desired number of IFFT points to use in the OFDM
%                      modulator. If absent or set to [], a default value
%                      is selected based on other parameters, see 
%                      <a href="matlab: doc('nrOFDMModulate')"
%                      >nrOFDMModulate</a> for details
%   SampleRate       - Desired sample rate of the OFDM modulated waveform.
%                      If absent or set to [], the default value is 
%                      SampleRate = Nfft * SCS. If required, the OFDM 
%                      modulated waveform is resampled to this sample rate
%                      after OFDM symbol construction, using an IFFT of
%                      size INFO.Nfft
%   CarrierFrequency - Carrier frequency (in Hz) to calculate the phase 
%                      precompensation applied for each OFDM symbol 
%                      (denoted f_0 in TS 38.211 Section 5.4). Default is 0
%
%   Note that for the numerologies specified in TS 38.211 Section 4.2, 
%   extended cyclic prefix length is only applicable for 60 kHz subcarrier
%   spacing. For normal cyclic prefix there are L=14 OFDM symbols in a 
%   slot. For extended cyclic prefix, L=12.
%
%   Example:
%   % Create a resource grid containing the PDSCH DM-RS, OFDM modulate the
%   % resource grid, pass the waveform through a TDL-C channel, and 
%   % estimate the timing offset:
%
%   NRB = 52;
%   SCS = 15;
%   nSlot = 0;
%
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = NRB;
%   carrier.SubcarrierSpacing = SCS;
%   carrier.NSlot = nSlot;
%   pdsch = nrPDSCHConfig;
%   dmrsInd = nrPDSCHDMRSIndices(carrier,pdsch);
%   dmrsSym = nrPDSCHDMRS(carrier,pdsch);
%   txGrid = complex(zeros([NRB*12 14 1]));
%   txGrid(dmrsInd) = dmrsSym;
%
%   [txWaveform,ofdmInfo] = nrOFDMModulate(txGrid,SCS,nSlot);
%
%   channel = nrTDLChannel;
%   channel.SampleRate = ofdmInfo.SampleRate;
%   channel.DelayProfile = 'TDL-C';
%   rxWaveform = channel(txWaveform);
%   
%   offset = nrTimingEstimate(carrier,rxWaveform,dmrsInd,dmrsSym);
%
%   See also nrChannelEstimate, nrPerfectTimingEstimate, 
%   nrPerfectChannelEstimate, nrCarrierConfig.

%   Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    narginchk(3,14);
    
    % Get optional inputs or inputs whose position depends upon the syntax
    [waveform,refGrid,ofdmInfo,initialNSlot,hasSampleRate] = getOptionalInputs(varargin{:});
    
    % Get the number of ports P in the reference resource grid
    P = size(refGrid,3);
    
    % Perform OFDM modulation
    overlapping = false;
    ref = nr5g.internal.OFDMModulate(refGrid,ofdmInfo,initialNSlot,overlapping,hasSampleRate);
    
    % Get the number of time samples T and receive antennas R in the 
    % waveform
    [T,R] = size(waveform);
    
    % Pad the input waveform if necessary to make it longer than the
    % correlation reference signal; this is required to normalize xcorr
    % behavior as it always pads the shorter input signal
    minlength = size(ref,1);
    if (T < minlength)
        waveformPad = [waveform; zeros([minlength-T R],'like',waveform)];
        T = minlength;
    else
        waveformPad = waveform;
    end
    
    % Create array 'mag' to store correlation magnitude for each time
    % sample, receive antenna and port
    mag = zeros([T R P],'like',waveformPad);
    
    % For each receive antenna
    for r = 1:R
        
        % For each port
        for p = 1:P
        
            % Correlate the given antenna of the received signal with the
            % given port of the reference signal
            refcorr = xcorr(waveformPad(:,r),ref(:,p));
            mag(:,r,p) = abs(refcorr(T:end));
            
        end
        
    end
    
    % Sum the magnitudes of the ports
    mag = sum(mag,3);
    
    % Find timing peak in the sum of the magnitudes of the receive antennas
    [~,peakindex] = max(sum(mag,2));
    offset = peakindex - 1;
    
end

% Parse optional inputs
function [waveform,refGrid,ofdmInfo,initialNSlot,hasSampleRate] = getOptionalInputs(varargin)

    fcnName = 'nrTimingEstimate';
    
    % Determine if syntax with nrCarrierConfig is being used and parse
    % relevant inputs
    isCarrierSyntax = isa(varargin{1},'nrCarrierConfig');
    if (isCarrierSyntax)
        carrier = varargin{1};
        validateattributes(carrier,{'nrCarrierConfig'}, ...
            {'scalar'},fcnName,'Carrier specific configuration object');
        waveform = varargin{2};
        initialNSlot = carrier.NSlot;
        firstrefarg = 3;
    else
        narginchk(5,14);
        waveform = varargin{1};
        NRB = varargin{2};
        SCS = varargin{3};
        initialNSlot = varargin{4};
        firstrefarg = 5;
    end
    
    % Validate waveform
    validateattributes(waveform,{'double','single'}, ...
        {},fcnName,'WAVEFORM');
    coder.internal.errorIf(~ismatrix(waveform), ...
        'nr5g:nrTimingEstimate:InvalidWaveformDims',ndims(waveform));
    
    if (~isCarrierSyntax)
        % Validate the number of resource blocks (1...275)
        validateattributes(NRB,{'numeric'}, ...
            {'real','integer','scalar','>=',1,'<=',275},fcnName,'NRB');

        % Validate subcarrier spacing input in kHz (15/30/60/120/240/480/960)
        validateattributes(SCS,{'numeric'}, ...
            {'real','integer','scalar'},fcnName,'SCS');
        validSCS = [15 30 60 120 240 480 960];
        coder.internal.errorIf(~any(SCS==validSCS), ...
            'nr5g:nrTimingEstimate:InvalidSCS', ...
            SCS,num2str(validSCS));

        % Validate zero-based initial slot number
        validateattributes(initialNSlot,{'numeric'}, ...
            {'real','nonnegative','scalar','integer'}, ...
            fcnName,'INITIALNSLOT');
    end
    
    % Determine whether the refInd,refSym syntax or refGrid syntax is being
    % used
    isRefGridSyntax = ...
        (nargin==firstrefarg) || ischar(varargin{firstrefarg + 1}) ...
            || isstring(varargin{firstrefarg + 1}) ...
            || isstruct(varargin{firstrefarg + 1});
    if (isRefGridSyntax)
        % nrTimingEstimate(...,refGrid,...)
        firstoptarg = firstrefarg + 1;
    else
        % nrTimingEstimate(...,refInd,refSym,...)
        firstoptarg = firstrefarg + 2;
    end
    
    % Parse options and get OFDM information
    if (isCarrierSyntax)
        optNames = {'Nfft','SampleRate','CarrierFrequency'};
        opts = nr5g.internal.parseOptions( ...
            fcnName,optNames,varargin{firstoptarg:end});
        
        % If performing code generation, the presence of sample rate with the
        % function syntax using nrCarrierConfig triggers a compile-time error
        hasSampleRate = ~isempty(opts.SampleRate);
        coder.internal.errorIf(~coder.target('MATLAB') && hasSampleRate, ...
            'nr5g:nrTimingEstimate:CompilationCarrierSampleRate');

        ofdmInfo = nr5g.internal.OFDMInfo(carrier,opts);
    else
        optNames = {'CyclicPrefix','Nfft','SampleRate','CarrierFrequency'};
        opts = nr5g.internal.parseOptions( ...
            fcnName,optNames,varargin{firstoptarg:end});
        ECP = strcmpi(opts.CyclicPrefix,'extended');

        % If performing code generation and 'SampleRate' is supplied, then
        % NRB, SCS, Nfft, and SampleRate must be constant at compile time.
        hasSampleRate = ~isempty(opts.SampleRate);
        coder.internal.errorIf(~coder.target('MATLAB') && hasSampleRate && ...
            (~coder.internal.isConst(NRB) || ~coder.internal.isConst(SCS) || ...
            ~coder.internal.isConst(opts.Nfft) || ~coder.internal.isConst(opts.SampleRate)), ...
            'nr5g:nrTimingEstimate:NonConstantNfftScsSampleRateNrb');

        ofdmInfo = nr5g.internal.OFDMInfo(NRB,SCS,ECP,opts);
    end
    
    % Get the number of subcarriers K and OFDM symbols L from the OFDM 
    % information
    K = ofdmInfo.NSubcarriers;
    L = ofdmInfo.SymbolsPerSlot;

    % Validate reference inputs
    if (~isRefGridSyntax)
    
        refInd = varargin{firstrefarg};
        refSym = varargin{firstrefarg + 1};

        % Validate reference indices
        validateattributes(refInd,{'numeric'}, ...
            {'positive','finite','2d'},fcnName,'REFIND');
        refIndColumns = size(refInd,2);

        % Validate reference symbols
        validateattributes(refSym,{'double','single'}, ...
            {'finite','2d'},fcnName,'REFSYM');
        refSymColumns = size(refSym,2);

        % Get the number of ports, based on the range of the reference
        % symbol indices
        if (isempty(refInd) && isempty(refSym) && refIndColumns==refSymColumns)
            P = max(refIndColumns,1);
        else
            P = ceil(max(double(refInd(:))/(K*L)));
        end
        
        % Create the reference resource grid
        refGrid = zeros([K L P],'like',waveform);
        
        % Map the reference symbols to the reference grid
        refGrid(refInd) = refSym;
        
    else % One optional input, not including 'cp' if present
        
        refGrid = varargin{firstrefarg};
        
        % Validate reference grid
        validateattributes(refGrid, ...
            {'double','single'},{'finite','3d'},fcnName,'REFGRID');
        coder.internal.errorIf(size(refGrid,1)~=K, ...
            'nr5g:nrTimingEstimate:InvalidRefGridSubcarriers', ...
        size(refGrid,1),K);
        coder.internal.errorIf(size(refGrid,2)==0, ...
            'nr5g:nrTimingEstimate:InvalidRefGridOFDMSymbols');
        
    end

end
