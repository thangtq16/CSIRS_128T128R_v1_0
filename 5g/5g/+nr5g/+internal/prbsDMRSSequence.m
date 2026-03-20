function symbols = prbsDMRSSequence(dmrs,ndmrssc,prbset,prbrefpoint,nslot,nsym,nslotsymb,portcdmgroup)
%prbsDMRSSequence DM-RS sequence for CP-OFDM
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   SYMBOLS = prbsDMRSSequence(DMRS,NDMRSSC,PRBSET,PRBREFPOINT,NSLOT,NSYM,NSLOTSYMB,CDMGRP)
%   returns the DM-RS sequence SYMBOLS according to TS 38.211 Section
%   7.4.1.1, given the inputs, a structure/object DMRS with fields NIDNSCID
%   and NSCID, the number of DM-RS subcarrier locations in a resource block
%   NDMRSSC, set of physical resource blocks PRBSET, the reference point
%   PRBREFPOINT, slot number NSLOT, symbol number NSYM and number of
%   symbols in a slot NSLOTSYMB. Optional CDMGRP is the CDM group of the
%   DM-RS port, if applicable to that DM-RS (defaults to 0).
%
%   Example:
%   % Provide the DM-RS sequence for a single resource block, with
%   % reference point set to 0, slot number set to 0, DM-RS symbol number
%   % set to 2, DM-RS scrambling identity set to 100, DM-RS scrambling
%   % initialization set to 0, number of DM-RS subcarriers in a resource
%   % block set to 4 and number of OFDM symbols in a slot set to 12.
%
%   prbset = 0;           % PRB set (0-based)
%   prbrefpoint = 0;      % PRB reference point relative to CRB0
%   nslot = 0;            % Slot number
%   nsym = 2;             % DM-RS symbol location (0-based)
%   pxsch.NIDNSCID = 100; % DM-RS scrambling identity (0...65535)
%   pxsch.NSCID = 0;      % DM-RS scrambling initialization (0 or 1)
%   ndmrssc = 4;          % Number of DM-RS subcarrier locations in an RB (4 or 6)
%   nslotsymb = 12;       % Number of symbols in a slot (14 or 12)
%   seq = ...
%   nr5g.internal.prbsDMRSSequence(pxsch,ndmrssc,prbset,prbrefpoint,nslot,nsym,nslotsymb)

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    if nargin < 8
        portcdmgroup = 0;  % Usage relates to Release 16 PDSCH/PUSCH DM-RS CDM dependency option
    end

    % Select active scrambling ID
    nidselected = min(dmrs.NSCID+1,length(dmrs.NIDNSCID));

    % Cache the scrambling IDs
    nidnscid = double(dmrs.NIDNSCID(nidselected));
    nscid = double(dmrs.NSCID);

    if ~isempty(prbset)
        % Generate PRBS for DM-RS sequence which covers the PRB allocation set range
        [minprb,maxprb] = bounds(prbset);
        cinit = mod(2^17*(nslotsymb*nslot + nsym + 1)*(2*nidnscid(1) + 1) + 2^17*floor(portcdmgroup/2) + 2*nidnscid + nscid,2^31);
        prbs = reshape(nrPRBS(cinit,2*ndmrssc*[prbrefpoint+minprb maxprb-minprb+1]),2*ndmrssc,[]);

        % Extract PRBS values associated with PRB and turn into complex DM-RS symbols
        bpsk = 1/sqrt(2)*(1-2*reshape(prbs(:,prbset-minprb+1),2,[])');
        symbols = complex(bpsk(:,1),bpsk(:,2));
    else
        symbols = complex([]);
    end
end