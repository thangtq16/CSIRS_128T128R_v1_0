function sym = nrPDSCH(varargin)
%nrPDSCH Physical downlink shared channel
%   SYM = nrPDSCH(CWS,MODULATION,NLAYERS,NID,RNTI) returns a complex matrix
%   SYM containing the physical downlink shared channel (PDSCH) modulation
%   symbols as defined in TS 38.211 Sections 7.3.1.1 - 7.3.1.3. The
%   processing consists of scrambling, symbol modulation and layer mapping.
%
%   CWS represents one or two DL-SCH codewords as described in TS 38.212
%   Section 7.2.6. CWS can be a column vector (representing one codeword)
%   or a cell array of one or two column vectors (representing one or two
%   codewords).
%
%   MODULATION specifies the modulation scheme for the codeword or
%   codewords in CWS. MODULATION can be specified as one of
%   'QPSK','16QAM','64QAM','256QAM','1024QAM'. If CWS contains two codewords,
%   this modulation scheme will apply to both codewords. Alternatively,
%   a string array or cell array of character vectors can be used to specify
%   the different modulation schemes for each codeword.
%
%   NLAYERS is the number of transmission layers (1...4 for one codeword,
%   5...8 for two codewords).
%
%   NID is the scrambling identity, representing either the cell identity
%   NCellID (0...1007) or the higher-layer parameter
%   dataScramblingIdentityPDSCH (0...1023).
%
%   RNTI is the Radio Network Temporary Identifier (0...65535).
%
%   SYM = nrPDSCH(CARRIER,PDSCH,CWS) returns a complex matrix SYM
%   containing the physical downlink shared channel modulation symbols as
%   defined in TS 38.211 Sections 7.3.1.1 - 7.3.1.3, given the carrier
%   configuration CARRIER, downlink shared channel configuration PDSCH, and
%   DL-SCH codeword(s) CWS.
%
%   CARRIER is a carrier configuration object as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with the following properties:
%      NCellID - Physical layer cell identity (0...1007) (default 1)
%
%   PDSCH is the physical downlink shared channel configuration object as
%   described in <a href="matlab:help('nrPDSCHConfig')">nrPDSCHConfig</a> with the following properties:
%      Modulation - Modulation scheme(s) of codeword(s)
%                   ('QPSK' (default), '16QAM', '64QAM', '256QAM', '1024QAM')
%      NumLayers  - Number of transmission layers (1...8) (default 1)
%      NID        - PDSCH scrambling identity (0...1023) (default []). Use
%                   empty ([]) to set the value to NCellID
%      RNTI       - Radio network temporary identifier (0...65535)
%                   (default 1)
%
%   SYM = nrPDSCH(...,NAME,VALUE) specifies an additional option as a
%   NAME,VALUE pair to allow control over the format of the symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
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
%   sym = nrPDSCH(data,modulation,nlayers,ncellid,rnti);
%
%   Example 2:
%   % Generate PDSCH symbols for two codewords with different modulation 
%   % orders and a total of 8 layers.
%
%   modulation = {'64QAM' '256QAM'};
%   nlayers = 8;
%   ncellid = 1;
%   rnti = 6143;
%   data = {randi([0 1],6000,1) randi([0 1],8000,1)};
%   sym = nrPDSCH(data,modulation,nlayers,ncellid,rnti);
%
%   Example 3:
%   % Generate PDSCH symbols for a single codeword of 8000 bits, using 
%   % 256QAM modulation, 4 layers and physical layer cell identity 42 with
%   % the PDSCH configuration object and carrier configuration object.
%
%   carrier = nrCarrierConfig;
%   carrier.NCellID = 42;
%   pdsch = nrPDSCHConfig;
%   pdsch.Modulation = '256QAM';
%   pdsch.NumLayers = 4;
%   pdsch.NID = [];
%   pdsch.RNTI = 6143;
%   data = randi([0 1],8000,1);
%   sym = nrPDSCH(carrier,pdsch,data);
%
%   See also nrPDSCHDecode, nrPDSCHPRBS, nrDLSCH, nrCarrierConfig,
%   nrPDSCHConfig.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    narginchk(3,7);
    
    % This prevents implicit expansion from being used in: xor(cellcws{q},c)
    coder.noImplicitExpansionInFunction;

    % Initialize inputs
    fcnName = 'nrPDSCH';
    if nargin == 3 || (nargin == 5 && isa(varargin{1},'nrCarrierConfig'))
        % CARRIER,PDSCH,CWS,[NAME,VALUE]
        carrier = varargin{1};            % Carrier configuration object
        pdsch = varargin{2};              % PDSCH configuration object
        coder.internal.errorIf(~(isa(carrier,'nrCarrierConfig') && isscalar(carrier)),'nr5g:nrPXSCH:InvalidCarrierInput');
        coder.internal.errorIf(~(isa(pdsch,'nrPDSCHConfig') && isscalar(pdsch)),'nr5g:nrPDSCH:InvalidPDSCHInput');
        cws = varargin{3};                % Codeword(s)
        modulation = pdsch.Modulation;    % Modulation scheme(s)
        nlayers = pdsch.NumLayers;        % Number of layers
        ncw = pdsch.NumCodewords;         % Number of codewords
        if isempty(pdsch.NID)
            % If PDSCH scrambling identity is empty, use physical layer
            % cell identity
            nid = carrier.NCellID;
        else
            nid = pdsch.NID(1);
        end
        rnti = pdsch.RNTI;                % Radio network temporary identifier
        nvarg = 4;
    else
        narginchk(5,7);
        % Flat signature
        % CWS,MODULATION,NLAYERS,NID,RNTI,[NAME,VALUE]
        cws = varargin{1};                % Codeword(s)
        modulation = varargin{2};         % Modulation scheme(s)
        nlayers = varargin{3};            % Number of layers
        nid = varargin{4};                % Scrambling identity
        rnti = varargin{5};               % Radio network temporary identifier
        nvarg = 6;

        % Establish number of codewords from number of layers
        validateattributes(nlayers,{'numeric'},{'nonempty','real','scalar',...
            'finite','integer','>=',1,'<=',8},fcnName,'NLAYERS');
        ncw = 1 + (nlayers > 4);
    end

    % Validate modulation scheme or schemes, and if only one modulation
    % scheme is specified for two codewords then apply it to both
    modlist = {'QPSK','16QAM','64QAM','256QAM','1024QAM'};
    mods = nr5g.internal.validatePXSCHModulation( ...
        fcnName,modulation,ncw,modlist);

    % Validate number of data codewords
    if ~iscell(cws)
        cellcws = {cws};
    else
        if ncw==1 && numel(cws)==2 && isempty(cws{2})
            % The input looks like 2 codewords but the second codeword is
            % empty so treat it as a single codeword
            cellcws = {cws{1}};
        else
            cellcws = cws;
        end
    end
    coder.internal.errorIf(ncw~=numel(cellcws), ...
        'nr5g:nrPXSCH:InvalidDataNCW',nlayers,numel(cellcws),ncw);

    % Check if either codeword is on the GPU
    isGPU = false;
    for q = 1:ncw
        if isa(cellcws{q},'gpuArray')
            isGPU = true;
        end
    end

    scrambled = coder.nullcopy(cell(1,ncw));
    modulated = coder.nullcopy(cell(1,ncw));
    for q = 1:ncw
        if isGPU
            cellcws{q} = gpuArray(cellcws{q});
        end

        % Scrambling, TS 38.211 Section 7.3.1.1
        validateattributes(cellcws{q},{'double','int8','logical'}, ...
            {},fcnName,'CWS');
        c = nrPDSCHPRBS(nid,rnti,q-1,length(cellcws{q}));
        scrambled{q} = xor(cellcws{q},c);

        % Modulation, TS 38.211 Section 7.3.1.2
        modulated{q} = nrSymbolModulate(scrambled{q},mods{q},varargin{nvarg:end});

    end

    % Layer mapping, TS 38.211 Section 7.3.1.3
    sym = nrLayerMap(modulated,nlayers);

end
