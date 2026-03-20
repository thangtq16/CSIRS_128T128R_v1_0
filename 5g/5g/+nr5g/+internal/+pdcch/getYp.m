function Yp = getYp(ssType,crstID,rnti,slotNum)
%getYp Determine Y_p for the current slot for CCE indices computation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   YP = nr5g.internal.pdcch.getYp(SSTYPE,CRSTID,RNTI,NSLOT) outputs YP for
%   the specified SearchSpaceType, SSTYPE, CORESET ID, CRSTID, RNTI and
%   relative slot number, NSLOT.
%
%   See also nrPDCCHResources, nrPDCCHSpace, nrSearchSpaceConfig,
%   nrPDCCHConfig.

%   Copyright 2019-2020 The MathWorks, Inc.

%#codegen

    switch ssType
        case 'ue'
            p = crstID;
            switch mod(p,3)
                case 0
                    Ap = 39827;
                case 1
                    Ap = 39829;
                otherwise
                    Ap = 39839;
            end
            D = 65537;

            Yp = rnti;         % Y_{p,-1}
            for n = 0:slotNum
                Yp = mod(Ap*Yp,D);
            end
        otherwise    % For Common SS
            Yp = 0;
    end
end
