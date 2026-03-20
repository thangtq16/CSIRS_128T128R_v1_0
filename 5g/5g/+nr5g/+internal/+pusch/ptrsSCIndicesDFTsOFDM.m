function m = ptrsSCIndicesDFTsOFDM(nGroupSamp,nPTRSGroup,msc)
%ptrsSCIndicesDFTsOFDM Provides the subcarrier indices of PT-RS
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   M = ptrsSCIndicesDFTsOFDM(NGROUPSAMP,NPTRSGROUP,MSC) returns the
%   subcarrier indices of PT-RS when transform precoding is enabled,
%   according to TS 38.211 Table 6.4.1.2.2.2-1, based on the inputs:
%
%   NGROUPSAMP  - Number of samples in a group (2 or 4)
%   NPTRSGROUP  - Number of PTRS groups (2,4,8)
%   MSC         - Scheduled number of subcarriers for PUSCH
%
%   The possible pairs of nGroupSamp and nPTRSGroup are
%   {(2,2),(4,2),(2,4),(4,4),(4,8)}.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

     switch nPTRSGroup
         case 2
             if nGroupSamp == 2
                 index = repmat([1 3]*floor(msc/4),2,1)+repmat([0 1]',1,2)-1;
                 m = index(:);
             else
                 m = [0:3 msc-4:msc-1]';
             end
         case 4
             if nGroupSamp == 2
                 index = repmat(floor([1 3 5 7]*msc/8),2,1)+repmat([0 1]',1,4)-1;
                 m = index(:);
             else
                 m = [0:3 ((msc/4)+floor(msc/8)+(-2:1)) ((msc/2)+floor(msc/8)+(-2:1)) msc-4:msc-1]';
             end
         otherwise
             index = repmat(floor((1:6)*msc/8),4,1)+floor(msc/16)+repmat((-2:1)',1,6);
             m1 = index(:);
             m = [0:3 m1' msc-4:msc-1]';
     end
end
