function cws = nrPUSCHDescramble(in,nid,rnti,varargin)
%nrPUSCHDescramble Physical uplink shared channel descrambling
%   CWS = nrPUSCHDescramble(...) returns a vector or cell array of vectors
%   of soft bits CWS resulting from performing the inverse of physical
%   uplink shared channel (PUSCH) scrambling, as defined in TS 38.211
%   Section 6.3.1.1.
%
%   CWS = nrPUSCHDescramble(IN,NID,RNTI) performs PUSCH descrambling given
%   soft bits IN, scrambling identity NID and radio network temporary
%   identifier RNTI. Note that this syntax will only descramble the data
%   bits, as the placeholder bit locations for any UCI, if present, are
%   unknown.
%
%   IN is a column vector of approximate log-likelihood ratio (LLR) soft
%   bits of one codeword, or a cell array of one or two column vectors of
%   approximate LLR soft bits of one or two codewords. When IN is a vector,
%   CWS is a vector. When IN is a cell array, CWS will be a cell array.
%
%   NID is the scrambling identity (0...1023).
%
%   RNTI is the radio network temporary identifier (0...65535).
%
%   CWS = nrPUSCHDescramble(...,NRAPID) performs PUSCH descrambling
%   considering the random access preamble index (0...63), NRAPID, used to
%   initialize the scrambling sequence for msgA on PUSCH. Set NRAPID to []
%   to indicate that the scrambling initialization does not consider msgA
%   on PUSCH. When IN represents the approximate LLR soft bits of two
%   codewords, NRAPID is ignored.
%
%   CWS = nrPUSCHDescramble(...,XIND,YIND) performs PUSCH descrambling
%   accounting for the uplink control information (UCI) placeholder 'x' bit
%   locations, XIND and the UCI placeholder 'y' bit locations YIND. XIND
%   and YIND are 1-based column vectors within the codeword to indicate the
%   respective placeholder locations. The descrambling of input codeword IN
%   at the locations of XIND is ignored, and the locations of YIND are
%   descrambled with the previous values of the scrambling sequence. When
%   IN specifies two codewords, the function assumes that the UCI is
%   multiplexed on the first codeword.
%
%   CWS = nrPUSCHDescramble(IN,NID,RNTI,NRAPID,XIND,YIND,QUCI) performs
%   PUSCH descrambling accounting for the UCI placeholder bit locations,
%   XIND and YIND, and the codeword number of the codeword on which UCI is
%   multiplexed, QUCI (0 (default) or 1). When IN specifies only one
%   codeword, QUCI is ignored. When IN specified two codewords, NRAPID is
%   ignored.
%
%   Example 1:
%   % Perform PUSCH scrambling, symbol modulation, symbol demodulation and 
%   % descrambling of a codeword containing 3000 data bits.
%
%   cw = randi([0 1],3000,1);
%   ncellid = 42;
%   rnti = 101;
%   modulation = '16QAM';
%
%   scrambled = nrPUSCHScramble(cw,ncellid,rnti);
%   sym = nrSymbolModulate(scrambled,modulation);
%   demod = nrSymbolDemodulate(sym,modulation);
%
%   descrambled = nrPUSCHDescramble(demod,ncellid,rnti);
%   rxcw = double(descrambled<0);
%   isequal(cw,rxcw)
%
%   Example 2:
%   % Perform PUSCH scrambling, symbol modulation, symbol demodulation and
%   % descrambling of a codeword with placeholders.
%
%   cw = [1 -2 -1 -1]'; % 1 bit with value 1, encoding with '16QAM'
%   nid = 100;
%   rnti = 65350;
%   modulation = '16QAM';
%   xind = find(cw == -1);
%   yind = find(cw == -2);
%
%   scrambled = nrPUSCHScramble(cw,nid,rnti);
%   sym = nrSymbolModulate(scrambled,modulation);
%   demod = nrSymbolDemodulate(sym,modulation);
%
%   descrambled = nrPUSCHDescramble(demod,nid,rnti,xind,yind);
%   isequal(descrambled(1)<0,cw(1))
%
%   See also nrPUSCHScramble, nrPUSCHPRBS.
 
%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    narginchk(3,7);
    if nargin == 4 || nargin == 6 || nargin == 7
        % The fourth input is NRAPID, used to initialize the scrambling
        % sequence for msgA on PUSCH
        nrapid = varargin{1};
        firstoptarg = 2;
        nrapidSyntax = true;
    else
        % The scrambling sequence initialization only uses NID and RNTI
        nrapid = [];
        firstoptarg = 1;
        nrapidSyntax = false;
    end
    if nargin == 7
        % The last input is q
        Quci = varargin{4};
    else
        Quci = 0;
    end
    inCell = coder.nullcopy(cell(1,2));
    if iscell(in)
        if numel(in)==2 && ~isempty(in{1}) && isempty(in{2})
            % The input looks like 2 codewords but the second codeword is
            % empty so treat it as a single codeword
            inCell{1} = in{1};
            ncw = 1;
        else
            inCell = in;
            ncw = numel(inCell);
        end
        outputCell = true;
    else
        inCell{1} = in;
        ncw = 1;
        outputCell = false;
    end 

    % Input validations
    fcnName = 'nrPUSCHDescramble';
    coder.internal.errorIf(ncw>2, ...
        'nr5g:nrPUSCHScramble:InvalidNumCW',ncw);
    for i = 1:ncw
        if (isempty(inCell{i}) && isnumeric(inCell{i}))
            inCell{i} = zeros([0 1],'like',inCell{i});
        end
        validateattributes(inCell{i},{'double','single'},...
            {'finite','column'},fcnName,'IN');
    end
    if ncw == 2
        nrapid = []; % ignore nrapid for 2CW
        coder.internal.errorIf(~strcmpi(class(inCell{1}),class(inCell{2})), ...
            'nr5g:nrPUSCHScramble:Invalid2CWDataType');
    else
        Quci = 0; % ignore Quci for 1CW
    end
    validateattributes(nid,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',65535},fcnName,'RNTI');
    if ~isempty(nrapid)
        validateattributes(nrapid,{'numeric'}, {'scalar', ...
            'integer','>=',0,'<=',63},fcnName,'NRAPID');
    end
    validateattributes(Quci,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',1},fcnName,'Quci');

    cwsCell = cell(1,ncw);
    for i = 1:ncw

        softBits = inCell{i};
        % Create scrambling sequence
        opts.MappingType = 'signed';
        opts.OutputDataType = underlyingType(softBits);
        cwLength = length(softBits);
        c = nrPUSCHPRBS(nid,rnti,nrapid,cwLength,i-1,opts);

        % Find the locations of data and UCI placeholder bits
        if nargin > (3 + firstoptarg - 1) && i == Quci+1
            numReqInputs = 3 + firstoptarg + 1;
            narginchk(numReqInputs,numReqInputs+nrapidSyntax); % without nrapid: nargin = 5; with nrapid: nargin = 6 or 7
            xInd = varargin{firstoptarg};
            yInd = varargin{firstoptarg+1};
            validateInputWithEmpty(xInd,{'numeric'},...
                {'column','integer','positive','<=',cwLength},fcnName,'XIND');
            validateInputWithEmpty(yInd,{'numeric'},...
                {'column','integer','positive','<=',cwLength},fcnName,'YIND');

            % Get the unique 'x' and 'y' placeholder locations
            map = zeros(cwLength,1);
            map(yInd(:)) = 3;
            map(xInd(:)) = 2;
            dataIndex = find(map == 0);
            xIndSort = find(map == 2);
            yIndSort = find(map == 3);

            % Find the UCI placeholder locations which require special handling
            % Get the start and last value of consecutive 'y' placeholder
            % indices
            consInd = diff([0; diff(yIndSort)==1; 0]);
            startInd = yIndSort(consInd>0);
            lastInd = yIndSort(consInd<0);

            % Check for the 'y' placeholder locations which has 'x' placeholder
            % locations previous to it
            numYInd = numel(startInd);
            if numYInd
                [STARTIND,XINDSORT] = meshgrid(startInd-1,[0;xIndSort(:)]);
                logicalMatrix = any(STARTIND==XINDSORT);
            else
                logicalMatrix = false(1,numYInd);
            end
            yc = yIndSort; % The placeholder locations to be used in scrambling sequence
            for j = 1:numYInd
                si = find(yIndSort == startInd(j));
                li = find(yIndSort == lastInd(j));
                if logicalMatrix(j)
                    yc(si(1):li(1)) = -1;
                else
                    yc(si(1):li(1)) = yc(si(1));
                end
            end
            yIndSort = yIndSort(yc ~= -1);
            yc = yc(yc ~= -1);
            if any(yc == 1)
                % Ignore 'y' placeholder location, if present at the starting
                % location
                yc(1) = [];
                yIndSort(1) = [];
            end
            % Check if locations previous to yc include any 'x' placeholder
            % locations
            if ~isempty(yc)
                % Get the locations which has 'x' placeholder locations
                % previous to yc
                map(yc-1) = map(yc-1)-1;
                mapInd = (map == 1);
                yMapInd = mapInd(yIndSort-1);
                % Get the effective 'y' placeholder locations
                yc = yc(~yMapInd);
                yIndSort = yIndSort(~yMapInd);
            end
        else
            % Assume no placeholders
            dataIndex = reshape(1:cwLength,[],1); % Ensure this is a column vector
            yIndSort = zeros(0,1);
            yc = zeros(0,1);
        end

        % Descramble input soft bits, ignore 'x' and take the previous for 'y'
        % placeholder bit locations, if any
        descrambled = softBits;
        descrambled(dataIndex) = softBits(dataIndex) .* c(dataIndex);
        % Descramble soft bits at 'y' placeholder locations
        descrambled(yIndSort) = descrambled(yIndSort).*c(yc-1); % yc-1 are previous bit locations
        cwsCell{i} = descrambled;

    end

    % Output handling
    if outputCell
        cws = cwsCell;
    else
        cws = cwsCell{1};
    end

end

function validateInputWithEmpty(in,classes,attributes,fcnName,varname)
%Validates input

    if ~isempty(in)
        % Check for type and attributes
        validateattributes(in,classes,attributes,fcnName,varname);
    else
        % Check for type when input is empty
        validateattributes(in,classes,{'2d'},fcnName,varname);
    end

end
