function out = nrPUSCHScramble(cws,nid,rnti,varargin)
%nrPUSCHScramble Physical uplink shared channel scrambling
%   OUT = nrPUSCHScramble(CWS,NID,RNTI) returns the scrambled sequence(s)
%   OUT resulting from physical uplink shared channel (PUSCH) scrambling
%   according to TS 38.211 Section 6.3.1.1 given the codeword(s) CWS,
%   scrambling identity NID, and radio network temporary identifier RNTI.
%
%   CWS is a column vector representing one UL-SCH codeword or a cell array
%   of one or two column vectors representing one or two UL-SCH codewords,
%   as described in TS 38.211 Section 6.2.7. Bit values should be
%   represented by 0 and 1. Specify placeholders for UCI with tags 'x' and
%   'y' by values -1 and -2, respectively. When CWS is a cell array, the
%   output is also a cell array.
%
%   NID is the scrambling identity (0...1023).
%
%   RNTI is the radio network temporary identifier (0...65535).
%
%   OUT = nrPUSCHScramble(...,NRAPID) performs PUSCH scrambling considering
%   the random access preamble index, NRAPID, used to initialize the
%   scrambling sequence for msgA on PUSCH. When CWS is a cell array
%   representing two UL-SCH codewords, NRAPID is ignored.
%
%   Example:
%   % Perform PUSCH scrambling of a codeword containing 5000 bits.
%
%   cw = randi([0 1],5000,1);
%   ncellid = 42;
%   rnti = 101;
%   scrambled = nrPUSCHScramble(cw,ncellid,rnti);
%
%   See also nrPUSCHDescramble, nrPUSCHPRBS.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    narginchk(3,4);

    if nargin == 4
        % The fourth input is NRAPID, used to initialize the scrambling
        % sequence for msgA on PUSCH
        nrapid = varargin{1};
    else
        % The scrambling sequence initialization only uses NID and RNTI
        nrapid = [];
    end
    cwsCell = coder.nullcopy(cell(1,2));
    if iscell(cws)
        if numel(cws)==2 && ~isempty(cws{1}) && isempty(cws{2})
            % The input looks like 2 codewords but the second codeword is
            % empty so treat it as a single codeword
            cwsCell{1} = cws{1};
            ncw = 1;
        else
            cwsCell = cws;
            ncw = numel(cwsCell);
        end
        outputCell = true;
    else
        cwsCell{1} = cws;
        ncw = 1;
        outputCell = false;
    end

    % Input validations
    fcnName = 'nrPUSCHScramble';
    coder.internal.errorIf(ncw>2, ...
        'nr5g:nrPUSCHScramble:InvalidNumCW',ncw);
    uciOnCW = [false false]; % Flag to indicate whether UCI, if any, is on the first or second codeword
    for i = 1:ncw
        if (isempty(cwsCell{i}) && isnumeric(cwsCell{i}))
            cwsCell{i} = zeros([0 1],'like',cwsCell{i});
        end
        validateattributes(cwsCell{i},{'double','int8'}, ...
            {'column'},fcnName,'CWS');

        thisCW = cwsCell{i};
        uciOnCW(i) = any(thisCW(:,1)==-1) || any(thisCW(:,1)==-2);
    end
    coder.internal.errorIf(all(uciOnCW), 'nr5g:nrPUSCHScramble:InvalidUCIPlaceholder');
    if ncw == 2
        nrapid = []; % ignore nrapid for 2CW
    end
    validateattributes(nid,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',65535},fcnName,'RNTI');
    if ~isempty(nrapid)
        validateattributes(nrapid,{'numeric'}, {'scalar', ...
            'integer','>=',0,'<=',63},fcnName,'NRAPID');
    end

    outCell = cell(1,ncw);
    for i = 1:ncw

        cw = cwsCell{i};
        % Create scrambling sequence
        cwLen = length(cw);
        c = nrPUSCHPRBS(nid,rnti,nrapid,cwLen,i-1);

        % Get the data locations excluding the UCI placeholders having tag 'x'
        % (input value is -1) and tag 'y' (input value is -2)
        xLogical = (cw == -1);
        yLogical = (cw == -2);
        dataLogical = ~(xLogical | yLogical);

        % Scramble all input data bits (excluding UCI placeholders)
        scrambled = cast(ones(cwLen,1,'like',cw),'logical');
        scrambled(dataLogical) = xor(cw(dataLogical),c(dataLogical));

        % Replace UCI placeholders having tag 'y'. If the first input value is
        % 'y' placeholder, assume that the previous bit is 0
        yidx = find(yLogical);
        yfirst = (yidx==1);
        scrambled(yidx(yfirst)) = 0;
        start = any(yfirst)+1;
        for idx = start:numel(yidx)
            scrambled(yidx(idx)) = scrambled(yidx(idx) - 1);
        end
        outCell{i} = scrambled;

    end

    % Output format handling
    if ~outputCell
        out = outCell{1};
    else
        out = outCell;
    end

end
