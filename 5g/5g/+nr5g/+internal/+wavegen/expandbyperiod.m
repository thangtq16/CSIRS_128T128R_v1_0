function [sp,initslot] = expandbyperiod(in,period,nsf,scs,initnsf)
% Expand 'in' values with respect to 'period', up to value 'nsf' (optionally accounting for the SCS)
%
% If the optional input initnsf is specified, the expansion starts from the
% initial subframe initnsf (0-based) (default 0)
%
% initslot is the slot number of the initial slot, depending on initnsf and
% scs.

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    if nargin < 4
        initslot = 0;
        ts = nsf;
    else
        ts = nsf*1*fix(scs/15);
        if nargin < 5
            initnsf = 0;
        end
        initslot = initnsf*fix(scs/15);
    end
    endslot = initslot +ts;
    
    % Only consider unique values of the input in
    if ~isempty(in)
        tmpIn = unique(in(:));
    else
        tmpIn = in(:); % for codegen
    end
    % Is the period is empty then the pattern doesn't repeat, so doesn't need extending
    if isempty(period)
        tmp = tmpIn;
    else
      % This code can be more compact in MATLAB, but this supports code
      % generation:
      withinPeriod = reshape(tmpIn(tmpIn<period(1)),[],1);
      periodInstances = period(1)*(floor(initslot/max(period(1),1)):ceil(endslot/max(period(1),1))-1);
      tmp = zeros(length(withinPeriod), length(periodInstances));
      for idx1 = 1:length(withinPeriod)
        for idx2 = 1:length(periodInstances)
          tmp(idx1, idx2) = withinPeriod(idx1) + periodInstances(idx2);
        end
      end
    end
    if ~isempty(ts)
      tmp2 = tmp(:); % for codegen
      sp = reshape(tmp2(tmp2 < endslot & tmp2 >= initslot),1,[]); % Trim any excess
    else
      sp = ones(1,0);
    end
end