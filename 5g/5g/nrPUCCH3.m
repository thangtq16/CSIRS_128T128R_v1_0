function sym = nrPUCCH3(uciCW,modulation,nid,rnti,Mrb,varargin)
%nrPUCCH3 Physical uplink control channel format 3
%   SYM = nrPUCCH3(UCICW,MODULATION,NID,RNTI,MRB) returns a complex column
%   vector SYM containing physical uplink control channel format 3 encoded
%   symbols as per TS 38.211 Section 6.3.2.6, by considering the following
%   inputs:
%   UCICW      - Encoded UCI codeword as per TS 38.212 Section 6.3.1. It
%                must be a column vector.
%   MODULATION - Modulation scheme. It must be one of the set
%                {'pi/2-BPSK', 'QPSK'}.
%   NID        - Scrambling identity. It is equal to the higher-layer
%                parameter dataScramblingIdentityPUSCH (0...1023), if
%                configured, else, it is equal to the physical layer cell
%                identity, NCellID (0...1007).
%   RNTI       - Radio Network Temporary Identifier (0...65535).
%   MRB        - The number of resource blocks associated with the PUCCH
%                format 3 transmission. Nominally the value of MRB will be
%                one of the set {1,2,3,4,5,6,8,9,10,12,15,16}.
%
%   The encoding process involves scrambling, symbol modulation and
%   transform precoding.
%
%   SYM = nrPUCCH3(UCICW,MODULATION,NID,RNTI,MRB,NAME,VALUE) specifies an
%   additional option as a NAME,VALUE pair to allow control over the
%   data type of the output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example 1:
%   % Generate QPSK modulated PUCCH format 3 symbols with nid as 148, rnti
%   % as 160 and Mrb as 2.
%
%   uciCW = randi([0 1],96,1);
%   modulation = 'QPSK';
%   nid = 148;
%   rnti = 160;
%   Mrb = 2;
%   sym = nrPUCCH3(uciCW,modulation,nid,rnti,Mrb);
%
%   Example 2:
%   % Generate pi/2-BPSK modulated PUCCH format 3 symbols of single
%   % data type with nid as 512, rnti as 2563, Mrb as 2.
%
%   uciCW = randi([0 1],96,1);
%   modulation = 'pi/2-BPSK';
%   nid = 512;
%   rnti = 2563;
%   Mrb = 2;
%   sym = nrPUCCH3(uciCW,modulation,nid,rnti,Mrb,...
%                  'OutputDataType','single');
%
%   See also nrPUCCH0, nrPUCCH1, nrPUCCH2, nrPUCCH4, nrPUCCHPRBS,
%   nrSymbolModulate, nrTransformPrecode.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(5,7);

    % Validate inputs
    modulation = parseAndValidateInputs(uciCW,modulation,nid,rnti,Mrb,varargin{:});

    % Set interlacing specific parameters to empty as they are not
    % supported in this function.
    sf = [];
    occi = [];
    sym = nr5g.internal.pucch.hPUCCH3(uciCW,modulation,nid,rnti,Mrb,sf,occi,varargin{:});

end

function modulation = parseAndValidateInputs(uciCW,modulation,nid,rnti,Mrb,varargin)

    fcnName = 'nrPUCCH3';
    validateattributes(uciCW,{'double','int8','logical'},{'real'},...
        fcnName,'UCICW');
    coder.internal.errorIf(~(iscolumn(uciCW) || isempty(uciCW)),...
        'nr5g:nrPUCCH:InvalidInputDim');
    validateattributes(nid,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',65535},fcnName,'RNTI');
    validateattributes(Mrb,{'numeric'},{'scalar',...
        'real','positive','integer'},fcnName,'MRB');

    modlist = {'pi/2-BPSK','QPSK'};
    modulation = validatestring(modulation,modlist,fcnName,'MODULATION');

end