function info = nrOFDMInfo(varargin)
%nrOFDMInfo OFDM modulation related information
%   INFO = nrOFDMInfo(CARRIER) provides dimensional information related to
%   OFDM modulation, given carrier configuration object CARRIER.
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
%   INFO = nrOFDMInfo(NRB,SCS) provides dimensional information related to
%   OFDM modulation for NRB resource blocks with subcarrier spacing SCS.
%
%   NRB is the number of resource blocks (1...275).
%
%   SCS is the subcarrier spacing in kHz (15, 30, 60, 120, 240, 480, 960).
%
%   INFO = nrOFDMInfo(...,NAME,VALUE) specifies additional options as
%   NAME,VALUE pairs to allow control over the OFDM modulation:
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
%                         sample rate after OFDM symbol construction, using 
%                         an IFFT of size INFO.Nfft
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
%   % Example:
%   % Create OFDM information related to a carrier for 20 MHz bandwidth
%
%   % Configure carrier for 20 MHz bandwidth
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 106;
%
%   % Create OFDM information
%   info = nrOFDMInfo(carrier)
%
%   See also nrCarrierConfig, nrOFDMModulate, nrOFDMDemodulate, 
%   nrResourceGrid.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    narginchk(1,12);
    
    % Validate inputs and get OFDM information
    internalinfo = validateInputs(varargin{:});
    
    % Create output structure
    info = nr5g.internal.OFDMInfoOutput(internalinfo);
    
end

% Validate inputs
function info = validateInputs(varargin)
    
    fcnName = 'nrOFDMInfo';

    isCarrierSyntax = isa(varargin{1},'nrCarrierConfig');
    if (isCarrierSyntax) % nrOFDMInfo(CARRIER,...)
    
        % Validate carrier input type
        carrier = varargin{1};
        validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},fcnName,'Carrier specific configuration object');
    
        % Parse options
        optNames = {'Nfft','SampleRate','Windowing','CarrierFrequency'};
        opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{2:end});
        
        % Get OFDM information
        info = nr5g.internal.OFDMInfo(carrier,opts);
        
    else % nrOFDMInfo(NRB,SCS,...)
        
         % Validate NRB
        NRB = varargin{1};
        validateattributes(NRB,{'numeric'},{'real','integer','scalar','>=',1,'<=',275},fcnName,'NRB');
        
        % Validate subcarrier spacing
        SCS = varargin{2};
        validateattributes(SCS,{'numeric'},{'real','integer','scalar'},fcnName,'SCS');
        validSCS = [15 30 60 120 240 480 960];
        coder.internal.errorIf(~any(SCS==validSCS),'nr5g:nrOFDMInfo:InvalidSCS',SCS,num2str(validSCS));
        
        % Parse options and get cyclic prefix length
        optNames = {'CyclicPrefix','Nfft','SampleRate','Windowing','CarrierFrequency'};
        opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{3:end});
        ECP = strcmpi(opts.CyclicPrefix,'extended');
        
        % Get OFDM information
        info = nr5g.internal.OFDMInfo(NRB,SCS,ECP,opts);
    
    end
    
end
