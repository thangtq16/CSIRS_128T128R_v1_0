function sym = nrPUCCH4(uciCW,modulation,nid,rnti,sf,occi,varargin)
%nrPUCCH4 Physical uplink control channel format 4
%   SYM = nrPUCCH4(UCICW,MODULATION,NID,RNTI,SF,OCCI) returns a complex
%   column vector SYM containing physical uplink control channel format 4
%   encoded symbols as per TS 38.211 Section 6.3.2.6, by considering the
%   following inputs:
%   UCICW      - Encoded UCI codeword as per TS 38.212 Section 6.3.1. It
%                must be a column vector.
%   MODULATION - Modulation scheme. It must be one of the set
%                {'pi/2-BPSK', 'QPSK'}.
%   NID        - Scrambling identity. It is equal to the higher-layer
%                parameter dataScramblingIdentityPUSCH (0...1023), if
%                configured, else, it is equal to the physical layer cell
%                identity, NCellID (0...1007).
%   RNTI       - Radio Network Temporary Identifier (0...65535).
%   SF         - Spreading factor for PUCCH format 4. It must be either 2
%                or 4.
%   OCCI       - Orthogonal cover code sequence index. It must be greater
%                than or equal to zero and less than SF.
%
%   The encoding process involves scrambling, symbol modulation, blockwise
%   spreading and transform precoding.
%
%   SYM = nrPUCCH4(...,MRB) also specifies the number of resource blocks
%   associated with the PUCCH format 4 transmission. If MRB is not
%   specified, the function uses the default value of 1.
%
%   Note that the transform precoding is done by considering the number of
%   subcarriers associated with the PUCCH format 4 transmission as MRB*12.
%   The length of UCICW must be an integer multiple of Qm*12, where Qm is 1
%   for pi/2-BPSK modulation and 2 for QPSK modulation.
%
%   SYM = nrPUCCH4(UCICW,MODULATION,NID,RNTI,SF,OCCI,NAME,VALUE) specifies
%   an additional option as a NAME,VALUE pair to allow control over the
%   data type of the output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example 1:
%   % Generate QPSK modulated PUCCH format 4 symbols with nid as 148,
%   % rnti as 160, sf as 2 and orthogonal cover code sequence index as 1.
%
%   uciCW = randi([0 1],96,1);
%   modulation = 'QPSK';
%   nid = 148;
%   rnti = 160;
%   sf = 2;
%   occi = 1;
%   sym = nrPUCCH4(uciCW,modulation,nid,rnti,sf,occi);
%
%   Example 2:
%   % Generate pi/2-BPSK modulated PUCCH format 4 symbols of single
%   % data type with nid as 285, rnti as 897, sf as 4 and orthogonal cover
%   % code sequence index as 3.
%
%   uciCW = randi([0 1],192,1);
%   modulation = 'pi/2-BPSK';
%   nid = 285;
%   rnti = 897;
%   sf = 4;
%   occi = 3;
%   sym = nrPUCCH4(uciCW,modulation,nid,rnti,sf,occi,...
%                  'OutputDataType','single');
%
%   See also nrPUCCH0, nrPUCCH1, nrPUCCH2, nrPUCCH3, nrPUCCHPRBS,
%   nrSymbolModulate, nrTransformPrecode.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(6,9);

    % Parse and validate inputs
    [modulation,Mrb,optargs] = parseAndValidateInputs(uciCW,modulation,nid,rnti,sf,occi,varargin{:});

    % Scrambling, TS 38.211 Section 6.3.2.6.1
    c = nrPUCCHPRBS(nid,rnti,length(uciCW));
    btilde = xor(uciCW,c);

    % Modulation, TS 38.211 Section 6.3.2.6.2
    d = nrSymbolModulate(btilde,modulation,optargs{:});

    % Validate input size and blockwise spreading configuration
    nRE = 12;
    formatPUCCH = 4;
    nr5g.internal.pucch.validateSpreadingConfig(length(d),modulation,Mrb,nRE,sf,formatPUCCH);

    % Blockwise spreading TS 38.211 Section 6.3.2.6.3
    y = nr5g.internal.pucch.blockWiseSpread(d,Mrb,sf,occi);

    % Transform precoding, TS 38.211 Section 6.3.2.6.4
    sym = nrTransformPrecode(y,Mrb);

end

% Parse and validate inputs
function [modulation,Mrb,optargs] = parseAndValidateInputs(uciCW,modulation,nid,rnti,sf,occi,varargin)

    fcnName = 'nrPUCCH4';
    validateattributes(uciCW,{'double','int8','logical'},{'real'},...
        fcnName,'UCICW');
    coder.internal.errorIf(~(iscolumn(uciCW) || isempty(uciCW)),...
        'nr5g:nrPUCCH:InvalidInputDim');
    validateattributes(nid,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',65535},fcnName,'RNTI');

    validateattributes(sf,{'numeric'},{'scalar',...
        'real','positive','integer'},fcnName,'SF');
    coder.internal.errorIf(~any(sf == [2 4]),'nr5g:nrPUCCH:InvalidSFFormats24',sf)

    validateattributes(occi,{'numeric'},{'scalar',...
        'real','nonnegative','integer'},fcnName,'OCCI');
    coder.internal.errorIf(occi >= sf,'nr5g:nrPUCCH:InvalidOCCIPUCCH234',occi,sf)

    modlist = {'pi/2-BPSK','QPSK'};
    modulation = validatestring(modulation,modlist,fcnName,'MODULATION');

    if (nargin>6 && isnumeric(varargin{1}))
        % Mrb is an input
        Mrb = varargin{1};
        validateattributes(Mrb,{'numeric'},{'scalar',...
            'real','positive','integer'},fcnName,'MRB');
        Mrb = double(Mrb);
        firstoptarg = 2;
    else
        Mrb = 1;
        firstoptarg = 1;
    end
    optargs = {varargin{firstoptarg:end}};

end