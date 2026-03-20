function [u,v] = getHoppingParameters(dmrs,mzc,nslot,nsym,nslotsymb)
%getHoppingParameters Provides the group number and sequence number
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [U,V] = getHoppingParameters(DMRS,MZC,NSLOT,NSYM,NSLOTSYMB) returns
%   the sequence group number U and sequence number V according to TS
%   38.211 Section 6.4.1.1.1.2, given the inputs, PUSCH-specific DM-RS
%   configuration DMRS, length of sequence MZC, slot number NSLOT, symbol
%   number NSYM and number of OFDM symbols in a slot NSLOTSYMB.

% Copyright 2019 The MathWorks, Inc.

%#codegen

    % Get the scrambling identity for hopping DM-RS
    nrsid = dmrs.NRSID;
    % Calculate u and v values
    v = 0;      % Sequence number in group
    fgh = 0;    % Group hopping part
    if dmrs.GroupHopping
            cinit = floor(nrsid/30);
            fgh = mod(sum((2.^(0:7)').*nrPRBS(cinit,[8*(nslotsymb*nslot+nsym) 8])),30);
    elseif dmrs.SequenceHopping
            cinit = nrsid;
            if mzc >= 72  % If sequence length is greater than 6*12 DM-RS subcarriers
                v = double(nrPRBS(cinit,[nslotsymb*nslot+nsym 1]));
            end
    end
    u = mod(fgh+nrsid,30);   % Sequence group number

end