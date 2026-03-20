function sym = nrPBCHDMRS(ncellid,ibar_SSB,varargin)
%nrPBCHDMRS PBCH demodulation reference signal
%   SYM = nrPBCHDMRS(NCELLID,IBAR_SSB) returns a complex column vector SYM
%   containing the physical broadcast channel (PBCH) demodulation reference
%   signal (DM-RS) symbols as defined in TS 38.211 Section 7.4.1.4.1.
%   NCELLID is the physical layer cell identity (0...1007) and IBAR_SSB is
%   the time-dependent part of the DM-RS scrambling initialization based on
%   SS/PBCH block index and half-frame number (0...7).
%
%   For SS burst configurations with L as 4 SS/PBCH blocks per half frame,
%   IBAR_SSB should equal (I_SSB + 4*N_HF) where I_SSB is the 2 LSBs of the
%   SS/PBCH block index (0...3) and N_HF is the half frame number within
%   the frame (0,1). For SS burst configurations with L as 8 or 64 SS/PBCH
%   blocks per half frame, IBAR_SSB should equal I_SSB where I_SSB is the 3
%   LSBs of the SS/PBCH block index (0...7).
%
%   SYM = nrPBCHDMRS(NCELLID,IBAR_SSB,NAME,VALUE) specifies an additional
%   option as a NAME,VALUE pair to allow control over the format of the
%   symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   % Example:
%   % Generate the sequence of 144 PBCH DM-RS symbols associated with the
%   % third SSB block (i_SSB is 2) in the 2nd half frame (n_hf is 1) of a
%   % frame.
%
%   ncellid = 17;
%   i_SSB = 2;
%   n_hf = 1;
%   ibar_SSB = i_SSB + (4 * n_hf);
%
%   dmrs = nrPBCHDMRS(ncellid,ibar_SSB);
% 
%   See also nrPBCHDMRSIndices, nrPBCH, nrPSS, nrSSS, nrPRBS.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(2,4);

    % Validate inputs
    fcnName = 'nrPBCHDMRS';
    validateattributes(ncellid,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',1007},fcnName,'NCELLID');
    validateattributes(ibar_SSB,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',7},fcnName,'IBAR_SSB');

    cinit = 2^11*(double(ibar_SSB) + 1)*(fix(double(ncellid)/4) + 1) + ...
             2^6*(double(ibar_SSB) + 1) + mod(double(ncellid),4);

    % Get DM-RS sequence
    prbs = double(nrPRBS(cinit,2*144));

    % Convert sequence to symbols
    sym = nrSymbolModulate(prbs,"QPSK",varargin{:});

end
