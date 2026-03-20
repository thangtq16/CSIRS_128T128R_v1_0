function [dciBits,mask] = nrDCIDecode(dciCW,Kout,L,varargin)
%nrDCIDecode Downlink control information decoding
%   DCIBITS = nrDCIDecode(DCICW,K,L) decodes the input soft bits, DCICW, as
%   per TS 38.212 Sections 7.3.4, 7.3.3 and 7.3.2 to output the decoded DCI
%   bits, DCIBITS of length K. The processing includes rate recovery, polar
%   decoding and CRC decoding.
%   L is the specified list length used for polar decoding.
%   The input DCICW must be a column vector of soft bits (LLRs) and the
%   output DCIBITS is the output DCI message of length K.
%
%   [DCIBITS,MASK] = nrDCIDecode(...) also outputs the masked value. MASK
%   equals the RNTI value when there is no CRC error for the decoded block.
%
%   [DCIBITS,MASK] = nrDCIDecode(...,RNTI) also accepts an RNTI value that
%   may have been used at the transmit end for masking. When an RNTI is
%   specified, MASK equals 0 when there is no CRC error for the decoded
%   block.
%
%   % Example:
%   % Decode an encoded DCI block and check the recovered information
%   % elements.
%
%   K = 32;
%   RNTI = 100;
%   Edci = 240;
%   L = 8;
%   dciBits = randi([0 1],K,1);
%   dciCW   = nrDCIEncode(dciBits,RNTI,Edci);
%   [recBits,mask] = nrDCIDecode(1-2*dciCW,K,L,RNTI);
%   isequal(recBits,dciBits)
%   mask
%
%   See also nrDCIEncode, nrPDCCHDecode, nrPDCCH.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

%   Reference:
%   [1] 3GPP TS 38.212, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Multiplexing and channel
%   coding (Release 15). Section 7.3.

    narginchk(3,4)
    if nargin>3
        rnti = varargin{1};
    else
        rnti = 0;
    end

    % Validate inputs
    validateInputs(dciCW,Kout,L,rnti);

    Edci = length(dciCW);
    K = Kout+24;            % K includes CRC bits
    nMax = 9;               % for downlink
    N = nr5g.internal.polar.getN(K,Edci,nMax);

    % Rate recovery, Section 7.3.4, [1]
    recBlk = nrRateRecoverPolar(dciCW,K,N);

    % Polar decoding, Section 7.3.3, [1]
    padCRC = true;              % signifies input prepadding with ones
    decBlk = nrPolarDecode(recBlk,K,Edci,L,padCRC,rnti);

    % CRC decoding, Section 7.3.2, [1]
    [padDCIBits,mask] = nrCRCDecode([ones(24,1);decBlk],'24C',rnti);
    dciBits = cast(padDCIBits(25:end,1),'int8'); % remove the prepadding

end


function validateInputs(dciCW,Kout,L,rnti)
% Check inputs

    fcnName = 'nrDCIDecode';

    % Validate input soft bits, length must be less than or equal to 8192
    % and greater than 36
    validateattributes(dciCW,{'single','double'},{'real','column'}, ...
        fcnName,'DCICW');
    Edci = length(dciCW);
    coder.internal.errorIf(Edci>8192 || Edci<=36, ...
        'nr5g:nrDCIDecode:InvalidInputLength',Edci);

    % Validate length of output bits which must be greater than or equal to
    % 12 and less than or equal to the minimum of Edci-24 and 140
    validateattributes(Kout,{'numeric'}, ...
        {'scalar','integer','<=',min(Edci-24,140),'>=',12},fcnName,'K');

    % Validate decoding list length
    nr5g.internal.validateParameters('ListLength',L,fcnName);

    % Validate RNTI
    validateattributes(rnti,{'numeric'},{'scalar','integer','>=',0, ...
        '<=',65535},fcnName,'RNTI');

end
