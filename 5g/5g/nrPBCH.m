function sym = nrPBCH(cw,ncellid,v,varargin)
%nrPBCH Physical broadcast channel
%   SYM = nrPBCH(CW,NCELLID,V) returns a complex column vector SYM
%   containing the physical broadcast channel (PBCH) modulation symbols as
%   defined in TS 38.211 Section 7.3.3. CW is the BCH codeword consisting
%   of 864 bits as described in TS 38.212 Section 7.1.5, NCELLID is the
%   physical layer cell identity (0...1007), and V is the 2 or 3 LSBs
%   (0...7) of the SS/PBCH block index (used to select a portion of the
%   scrambling sequence).
%  
%   For SS burst configurations with L as 4 SS/PBCH blocks per half frame,
%   V is the 2 LSBs of the SS/PBCH block index (0...3). For SS burst
%   configurations with L as 8 or 64 SS/PBCH blocks per half frame, V is 3
%   LSBs of the SS/PBCH block index (0...7).
%
%   SYM = nrPBCH(CW,NCELLID,V,NAME,VALUE) specifies an additional option as
%   a NAME,VALUE pair to allow control over the format of the symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example:
%   % Generate the 432 PBCH symbols (QPSK) for the first SS/PBCH block in a 
%   % burst (ssbindex is 0) from random bits representing encoded BCH bits.
%
%   ncellid = 17;
%   ssbindex = 0;
%   v = mod(ssbindex,4);    % assuming L as 4 SS/PBCH blocks per half frame
%   E = 864;                % PBCH bit capacity, TS 38.212 Section 7.1.5
%   cw = randi([0 1],E,1);
%
%   sym = nrPBCH(cw,ncellid,v);
%
%   See also nrPBCHDecode, nrPBCHIndices, nrPBCHDMRS, nrPRBS, nrPBCHPRBS,
%   nrPBCHDMRSIndices, nrPSS, nrSSS.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(3,5);

    % Validate inputs
    fcnName = 'nrPBCH';
    validateattributes(ncellid,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',1007},fcnName,'NCELLID');
    validateattributes(v,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',7},fcnName,'V');
    validateattributes(cw,{'double','int8','logical'}, ...
        {'column'},fcnName,'CW');

    % Scrambling, Section 7.3.3.1, TS 38.211
    c = nrPBCHPRBS(ncellid,v,length(cw));
    scrambled = double(xor(cw,c));

    % Modulation, Section 7.3.3.2, TS 38.211
    sym = nrSymbolModulate(scrambled,'QPSK',varargin{:});

end
