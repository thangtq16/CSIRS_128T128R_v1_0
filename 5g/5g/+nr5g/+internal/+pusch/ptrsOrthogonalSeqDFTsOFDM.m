function w = ptrsOrthogonalSeqDFTsOFDM(nRNTI,nGroupSamp)
%ptrsOrthogonalSeqDFTsOFDM Provides the orthogonal sequence of PT-RS
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   W = ptrsOrthogonalSeqDFTsOFDM(NRNTI,NGROUPSAMP) returns the orthogonal
%   sequence to be applied for PT-RS when transform precoding is enabled
%   according to TS 38.211 Table 6.4.1.2.1.2-1, based on the inputs NRNTI
%   and NGROUPSAMP.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    if nGroupSamp == 2
        ws = [1  1;...
              1 -1];
    else
        ws = [1  1  1  1;...
              1 -1  1 -1;...
              1  1 -1 -1;...
              1 -1 -1  1];
    end

    w = ws(mod(double(nRNTI),nGroupSamp)+1,:)';

end