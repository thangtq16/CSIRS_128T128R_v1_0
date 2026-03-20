function info = nrPUCCHHoppingInfo(cp,nslot,nid,groupHopping,initialCS,seqCS,varargin)
%nrPUCCHHoppingInfo PUCCH hopping structural information
%   INFO = nrPUCCHHoppingInfo(...) returns the structural information of
%   sequence and cyclic shift hopping, used in physical uplink control
%   channel as defined in TS 38.211 Section 6.3.2.2.
%
%   INFO = nrPUCCHHoppingInfo(CP,NSLOT,NID,GROUPHOPPING,INITIALCS,SEQCS)
%   returns the structural information of sequence and cyclic shift hopping
%   in INFO, considering the intra-slot frequency hopping is enabled, with
%   the following inputs:
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
%   INITIALCS     - Initial cyclic shift (m_0) which is in range (0...11).
%                   For PUCCH formats 0 and 1, the value is provided by
%                   higher-layer parameter initialCyclicShift. For DMRS on
%                   PUCCH format 3, the value is 0 and for DMRS on format 4
%                   it can be any of 0,3,6,9.
%   SEQCS         - Sequence cyclic shift (m_cs) which is in range
%                   (0...11). The value is zero for all PUCCH formats
%                   except PUCCH format 0.
%
%   INFO = nrPUCCHHoppingInfo(...,NIRB) specifies the resource block
%   indices in the interlace as a vector of nonnegative integers.
%
%   The output structure INFO contains the following fields:
%   U       - Vector of base sequence group numbers, where each element
%             corresponds to each hop in a slot.
%   V       - Vector of base sequence numbers, where each element
%             corresponds to each hop in a slot.
%   Alpha   - Vector of cyclic shifts of all OFDM symbols in a given slot.
%             When NIRB is specified, Alpha is a matrix and each row
%             contains the cyclic shifts for each resource block in the
%             interlace.
%   FGH     - Vector of sequence-group hopping patterns, where each element
%             corresponds to each hop in a slot.
%   FSS     - Sequence-group shift offset.
%   Hopping - Information of hopping considered {'neither', 'groupHopping',
%             'sequenceHopping'}.
%   NCS     - Vector of hopping/cell identity specific cyclic shifts (n_cs)
%             of all symbols in a given slot.
%
%   Note that the values of group number and sequence number for the first
%   hop are the values when intra-slot frequency hopping is disabled.
%
%   Example 1:
%   % Get the PUCCH hopping parameters when slot number is 3, hopping
%   % identity is 512, initial cyclic shift is 5, sequence cyclic shift is
%   % 0, with normal cyclic prefix, group hopping and intra-slot frequency
%   % hopping enabled.
%
%   cp = 'normal';
%   nslot = 3;
%   nid = 512;
%   groupHopping = 'enable';
%   initialCS = 5;
%   seqCS = 0;
%   info = nrPUCCHHoppingInfo(cp,nslot,nid,groupHopping,initialCS,seqCS)
%
%   % The above example returns:
%   %           U: [13 22]
%   %           V: [0 0]
%   %       Alpha: [2.0944 2.0944 0 5.7596 2.6180 3.6652 4.1888 5.7596 1.5708 5.2360 5.2360 3.1416 0.5236 5.2360]
%   %         FGH: [11 20]
%   %         FSS: 2
%   %     Hopping: 'groupHopping'
%   %         NCS: [239 107 223 6 24 2 3 66 238 125 209 145 44 233]
%
%   Example 2:
%   % Get the PUCCH hopping parameters when slot number is 7, physical
%   % layer cell identity is 12, initial cyclic shift is 9, sequence cyclic
%   % shift is 0, symbol index is 4, with extended cyclic prefix, group
%   % hopping enabled and intra-slot frequency hopping disabled.
%
%   cp = 'extended';
%   nslot = 7;
%   nid = 12;
%   groupHopping = 'enable';
%   initialCS = 9;
%   seqCS = 0;
%   symInd = 4;
%   info = nrPUCCHHoppingInfo(cp,nslot,nid,groupHopping,initialCS,seqCS);
%   u = info.U(1);
%   v = info.V(1);
%   alpha = info.Alpha(symInd+1);
%
%   See also nrPUCCH0, nrPUCCH1, nrPUCCH0Config, nrPUCCH1Config.

% Copyright 2018-2023 The MathWorks, Inc.

%#codegen

    narginchk(6,7);

    % Validate inputs
    fcnName = 'nrPUCCHHoppingInfo';
    cp = validatestring(cp,{'normal','extended'},fcnName,'CP');
    validateattributes(nslot,{'numeric'},{'scalar','integer','nonnegative','<=',159},fcnName,'NSLOT');
    validateattributes(nid,{'numeric'},{'scalar','integer','nonnegative','<=',1023},fcnName,'NID');
    groupHopping = validatestring(groupHopping,{'neither','enable','disable'},fcnName,'GROUPHOPPING');
    validateattributes(initialCS,{'numeric'},{'scalar','integer','nonnegative','<=',11},fcnName,'INITIALCS');
    validateattributes(seqCS,{'numeric'},{'scalar','integer','nonnegative','<=',11},fcnName,'SEQCS');

    % Validate NIRB if provided
    if nargin == 6 || (nargin == 7 && isempty(varargin{1}))
        nIRB = 0;
    else
        validateattributes(varargin{1},{'numeric'},{'vector','integer','nonnegative'},fcnName,'NIRB');
        nIRB = double(varargin{1});
    end

    % Cast to double to enable floating-point operations
    m0 = double(initialCS);
    mcs = double(seqCS);
    NIRB = double(nIRB);
    
    % Get the number of symbols in a slot based on cyclic prefix
    if strcmpi(cp,'extended')
        nSlotSymb = 12;
    else
        nSlotSymb = 14;
    end

    % Get the group number and sequence number for both the hops in a slot
    nslot = cast(nslot,'double');
    nid = cast(nid,'double');
    v = zeros(1,2);
    fgh = zeros(1,2);
    switch groupHopping
        case 'neither'
            hopping = 'neither';
        case 'enable'
            cinit = floor(nid/30);
            fgh(1,:) = mod((2.^(0:7))*reshape(nrPRBS(cinit,[8*2*nslot 16]),8,[]),30);
            hopping = 'groupHopping';
        otherwise
            cinit = 32*floor(nid/30) + mod(nid,30);
            v(1,:) = double(nrPRBS(cinit,[2*nslot 2])');
            hopping = 'sequenceHopping';
    end
    fss = mod(nid,30);
    u = mod(fgh+fss,30);

    % Get the value of ncs for all the symbols in a slot
    ncs = (2.^(0:7))*reshape(nrPRBS(nid,nSlotSymb*8*[nslot 1]),8,[]);

    % Cyclic shift for interlacing
    mint = 5*NIRB(:);

    % Get the cyclic shift for all the symbols in a slot.
    nRBSC = 12;
    alpha = 2*pi*mod(m0 + mcs + mint + ncs, nRBSC)/nRBSC;

    % Combine the information into a structure
    info.U        = u;
    info.V        = v;
    info.Alpha    = alpha;
    info.FGH      = fgh;
    info.FSS      = fss;
    info.Hopping  = hopping;
    info.NCS      = ncs;

end