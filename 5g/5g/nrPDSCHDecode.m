function [cws,symbols] = nrPDSCHDecode(varargin)
%nrPDSCHDecode Physical downlink shared channel decoding
%   [CWS,SYMBOLS] = nrPDSCHDecode(SYM,MODULATION,NID,RNTI,NVAR) returns
%   a cell array CWS of soft bit vectors (codewords) and cell array SYMBOLS
%   of received constellation symbol vectors resulting from performing the
%   inverse of physical downlink shared channel (PDSCH) processing as
%   defined in TS 38.211 Sections 7.3.1.1 - 7.3.1.3. The decoding consists
%   of layer demapping, symbol demodulation, and descrambling.
%
%   SYM is a matrix of size NRE-by-NLAYERS, containing the received PDSCH
%   symbols for each layer. NRE is the number of QAM symbols (resource
%   elements) per layer assigned to the PDSCH. NLAYERS is the number of
%   layers.
%
%   The number of layers determines the number of codewords generated. For
%   NLAYERS (1...4), the function returns one codeword in CWS. For NLAYERS
%   (5...8), CWS will contain two codewords. MODULATION is a string or
%   character vector specifying the modulation scheme, one of
%   ('QPSK','16QAM','64QAM','256QAM','1024QAM'). A string array or cell array of
%   character vectors can be used to specify different modulation schemes
%   for each codeword in the two codeword case.
%
%   NID is the scrambling identity, representing either the cell identity
%   NCellID (0...1007) or the higher-layer parameter
%   dataScramblingIdentityPDSCH (0...1023).
%
%   RNTI is the Radio Network Temporary Identifier (0...65535).
%
%   NVAR is an optional nonnegative real scalar specifying the variance
%   of additive white Gaussian noise on the received PDSCH symbols. The
%   default value is 1e-10.
%
%   [CWS,SYMBOLS] = nrPDSCHDecode(CARRIER,PDSCH,SYM,NVAR) returns a cell
%   array CWS of soft bit vectors (codewords) and cell array SYMBOLS of
%   received constellation symbol vectors resulting from performing the
%   inverse of physical downlink shared channel processing as defined in TS
%   38.211 Sections 7.3.1.1 - 7.3.1.3, given the carrier configuration
%   CARRIER, downlink shared channel configuration PDSCH, received symbols
%   for each layer SYM, and optional noise variance NVAR.
%
%   CARRIER is a carrier configuration object as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with the following properties:
%      NCellID - Physical layer cell identity (0...1007) (default 1)
%
%   PDSCH is the physical downlink shared channel configuration object as
%   described in <a href="matlab:help('nrPDSCHConfig')">nrPDSCHConfig</a> with the following properties:
%      Modulation - Modulation scheme(s) of codeword(s)
%                   ('QPSK' (default), '16QAM', '64QAM', '256QAM', '1024QAM')
%      NID        - PDSCH scrambling identity (0...1023) (default []). Use
%                   empty ([]) to set the value to NCellID
%      RNTI       - Radio network temporary identifier (0...65535)
%                   (default 1)
%
%   Example 1:
%   % Generate PDSCH symbols for a single codeword of 8000 bits, using 
%   % 256QAM modulation and 4 layers.
%
%   modulation = '256QAM';
%   nlayers = 4;
%   ncellid = 42;
%   rnti = 6143;
%   data = randi([0 1],8000,1);
%   txsym = nrPDSCH(data,modulation,nlayers,ncellid,rnti);
%
%   % Add noise to the PDSCH symbols and demodulate to produce soft bit 
%   % estimates
%
%   SNR = 30; % SNR in dB
%   rxsym = awgn(txsym,SNR);
%   rxbits = nrPDSCHDecode(rxsym,modulation,ncellid,rnti);
%
%   Example 2:
%   % Generate the PDSCH symbols and decode the data bits for the same
%   % configuration specified in example 1 with the usage of objects.
%
%   carrier = nrCarrierConfig;
%   carrier.NCellID = 42;
%
%   pdsch = nrPDSCHConfig;
%   pdsch.Modulation = '256QAM';
%   pdsch.NumLayers = 4;
%   pdsch.RNTI = 6143;
%   pdsch.NID = []; % Defaults to NCellID
%   data = randi([0 1],8000,1);
%   txsym = nrPDSCH(carrier,pdsch,data);
%
%   % Add noise to the PDSCH symbols and demodulate to produce soft bit 
%   % estimates
%
%   SNR = 30; % SNR in dB
%   rxsym = awgn(txsym,SNR);
%   rxbits = nrPDSCHDecode(carrier,pdsch,rxsym);
%
%   See also nrPDSCH, nrPDSCHPRBS, nrDLSCHDecoder, nrCarrierConfig,
%   nrPDSCHConfig.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(3,5);

    % Initialize inputs
    if nargin == 3 || (nargin == 4 && isa(varargin{1},'nrCarrierConfig'))
        % CARRIER,PDSCH,SYM,[NVAR]
        carrier = varargin{1};           % Carrier configuration object
        pdsch = varargin{2};             % PDSCH configuration object
        coder.internal.errorIf(~(isa(carrier,'nrCarrierConfig') && isscalar(carrier)),'nr5g:nrPXSCH:InvalidCarrierInput');
        coder.internal.errorIf(~(isa(pdsch,'nrPDSCHConfig') && isscalar(pdsch)),'nr5g:nrPDSCH:InvalidPDSCHInput');
        sym = varargin{3};               % Received PDSCH symbols 
        if nargin == 4
            % Noise variance
            nVar = varargin{4};
        else
            nVar = 1e-10;
        end
        modulation = pdsch.Modulation;   % Modulation scheme(s)
        rnti = pdsch.RNTI;               % Radio network temporary identifier
        if isempty(pdsch.NID)
            % If PDSCH scrambling identity is empty, use physical layer
            % cell identity
            nid = carrier.NCellID;
        else
            nid = pdsch.NID(1);
        end
    else
        % Flat signature
        % SYM,MODULATION,NID,RNTI,[NVAR]
        sym = varargin{1};              % Received PDSCH symbols
        modulation = varargin{2};       % Modulation scheme(s)
        nid = varargin{3};              % Scrambling identity
        rnti = varargin{4};             % Radio network temporary identifier
        if nargin == 5
            % Noise variance
            nVar = varargin{5};
        else
            nVar = 1e-10;
        end
    end
    
    % Validate or default noise variance
    fcnName = 'nrPDSCHDecode';
    validateattributes(nVar,{'double','single'},...
        {'scalar','real','nonnegative','nonnan','finite'}, ...
        fcnName,'NVAR');
    
    % Layer demapping, inverse of TS 38.211 Section 7.3.1.3
    symbols = nrLayerDemap(sym);
    
    % Establish number of codewords from output of layer demapping
    ncw = size(symbols,2);
    
    % Validate modulation scheme or schemes, and if only one modulation
    % scheme is specified for two codewords then apply it to both
    modlist = {'QPSK','16QAM','64QAM','256QAM','1024QAM'};
    mods = nr5g.internal.validatePXSCHModulation( ...
        fcnName,modulation,ncw,modlist);
    
    demodulated = coder.nullcopy(cell(1,ncw));
    cws = coder.nullcopy(cell(1,ncw));
    opts.MappingType = 'signed';
    opts.OutputDataType = 'double';
    for q = 1:ncw

        % Demodulation, inverse of TS 38.211 Section 7.3.1.2
        demodulated{q} = nrSymbolDemodulate(symbols{q},mods{q},nVar);

        % Descrambling, inverse of TS 38.211 Section 7.3.1.1
        c = nrPDSCHPRBS(nid,rnti,q-1,length(demodulated{q}),opts);
        cws{q} = demodulated{q} .* c;

    end

end
