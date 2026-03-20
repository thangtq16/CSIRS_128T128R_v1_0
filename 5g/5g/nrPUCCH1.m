function sym = nrPUCCH1(ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,occi,varargin)
%nrPUCCH1 Physical uplink control channel format 1
%   SYM = nrPUCCH1(ACK,SR,SYMALLOCATION,CP,NSLOT,NID,GROUPHOPPING,INITIALCS,FREQHOPPING,OCCI)
%   returns the PUCCH format 1 modulated symbols SYM as per TS 38.211
%   Section 6.3.2.4, by considering the following inputs:
%   ACK           - Acknowledgment bits of hybrid automatic repeat request
%                   (HARQ-ACK). It is a column vector of length 0, 1 or 2
%                   HARQ-ACK bits. The bit value of 1 stands for positive
%                   acknowledgment and bit value of 0 stands for negative
%                   acknowledgment. Use empty ([]) to indicate no HARQ-ACK
%                   transmission.
%   SR            - Scheduling request (SR). It is a column vector of
%                   length 0 or 1 SR bits. The bit value of 1 stands for
%                   positive SR and bit value of 0 stands for negative SR.
%                   Use empty ([]) to indicate no SR transmission. The
%                   output SYM is empty when there is only negative SR
%                   transmission. For positive SR with HARQ-ACK information
%                   bits, only HARQ-ACK transmission happens.
%   SYMALLOCATION - Symbol allocation for PUCCH transmission. It is a
%                   two-element vector, where first element is the symbol
%                   index corresponding to first OFDM symbol of the PUCCH
%                   transmission in the slot and second element is the
%                   number of OFDM symbols allocated for PUCCH
%                   transmission, which is in range 4 and 14.
%   CP            - Cyclic prefix ('normal','extended').
%   NSLOT         - Slot number in radio frame. It is in range 0 to 159 for
%                   normal cyclic prefix for different numerologies. For
%                   extended cyclic prefix, it is in range 0 to 39, as
%                   specified in TS 38.211 Section 4.3.2.
%   NID           - Scrambling identity. It is in range 0 to 1023 if
%                   higher-layer parameter hoppingId is provided, else, it
%                   is in range 0 to 1007, equal to the physical layer cell
%                   identity NCellID.
%   GROUPHOPPING  - Group hopping configuration. It is one of the set
%                   {'neither','enable','disable'} provided by higher-layer
%                   parameter pucch-GroupHopping.
%   INITIALCS     - Initial cyclic shift (m_0). It is in range 0 to 11,
%                   provided by higher-layer parameter initialCyclicShift.
%   FREQHOPPING   - Intra-slot frequency hopping. It is one of the set
%                   {'enabled','disabled'} provided by higher-layer
%                   parameter intraSlotFrequencyHopping.
%   OCCI          - Orthogonal cover code index. It is in range 0 to 6,
%                   provided by higher-layer parameter timeDomainOCC. The
%                   valid range depends on the number of OFDM symbols per
%                   hop which contain control information.
%
%   Note that when GROUPHOPPING is set to 'disable', sequence hopping is
%   enabled which might result in selecting a sequence number that is not
%   appropriate for short base sequences.
%
%   SYM = nrPUCCH1(...,MRB) also specifies the number of resource blocks
%   associated with the PUCCH format 1 transmission. If MRB is not
%   specified, the function uses the default value of 1.
%
%   The output symbols SYM is a column vector of length given by product of
%   number of subcarriers in the MRB resource blocks and floor of half the
%   number of OFDM symbols allocated for PUCCH transmission in
%   SYMALLOCATION.
%
%   SYM = nrPUCCH1(...,NAME,VALUE) specifies an additional option as a
%   NAME,VALUE pair to allow control over the format of the symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example 1:
%   % Get the PUCCH format 1 modulated symbols for 2-bit HARQ-ACK with a
%   % positive SR, when number of PUCCH symbols is 14, orthogonal cover
%   % code index is 2, hopping identity is 512, slot number is 3, initial
%   % cyclic shift is 5, with normal cyclic prefix, intra-slot frequency
%   % hopping and group hopping enabled.
%
%   ack = [0;1];
%   sr = 1;
%   symAllocation = [0 14];
%   cp = 'normal';
%   nslot = 3;
%   nid = 512;
%   groupHopping = 'enable';
%   initialCS = 5;
%   freqHopping = 'enabled';
%   occi = 2;
%   sym = nrPUCCH1(ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,occi);
%
%   Example 2:
%   % Get the PUCCH format 1 modulated symbols for 1-bit HARQ-ACK with a
%   % negative SR, when the starting symbol in a slot is 3, number of PUCCH
%   % symbols is 9, orthogonal cover code index is 1, hopping identity is
%   % 512, slot number is 7, initial cyclic shift is 9, with extended
%   % cyclic prefix, intra-slot frequency hopping and group hopping
%   % enabled.
%
%   ack = 1;
%   sr = 0;
%   symAllocation = [3 9];
%   cp = 'extended';
%   nslot = 7;
%   nid = 512;
%   groupHopping = 'enable';
%   initialCS = 9;
%   freqHopping = 'enabled';
%   occi = 1;
%   sym = nrPUCCH1(ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,occi);
%
%   See also nrPUCCH0, nrPUCCH2, nrPUCCH3, nrPUCCH4, nrPUCCHHoppingInfo,
%   nrLowPAPRS.

% Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(10,13);

    % Parse and validate inputs
    fcnName = 'nrPUCCH1';
    [cp,groupHopping,freqHopping,Mrb,optargs] = ...
        nr5g.internal.pucch.validatePUCCHInputs(...
        ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,fcnName,varargin{:});
    validateattributes(occi,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'OCCI');

    % Set resource block indices in the interlace to empty as interlacing
    % is not supported in this function.
    nIRB = [];
    sym = nr5g.internal.pucch.hPUCCH1(ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,occi,Mrb,nIRB,optargs{:});

end
