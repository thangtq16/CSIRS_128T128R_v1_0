function numRB = getNumRB(fr, scs, bw)
% This is an internal function that can change any time. It serves a
% codegen-friendly functionality of getFR1/2BandwidthTable.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

numRB = NaN;
if strcmp(fr, 'FR1')
    % BW MHz    5   10 15 20  25  30  35  40  45  50  60  70  80  90  100
    nrbtable = [25  52 79 106 133 160 188 216 242 270 NaN NaN NaN NaN NaN;     % 15 kHz
                11  24 38 51  65  78  92  106 119 133 162 189 217 245 273;     % 30 kHz
                NaN 11 18 24  31  38  44  51  58  65  79  93  107 121 135];    % 60 kHz
    scsIdx = log2(scs/15)+1;
    bws = [5 10 15 20 25 30 35 40 45 50 60 70 80 90 100];
else
    % FR2 (FR2-1, FR2-2)
    % BW MHz    50  100 200 400 800 1600 2000
    nrbtable = [66  132 264 NaN NaN NaN  NaN;    % 60 kHz
                32  66  132 264 NaN NaN  NaN;    % 120 kHz
                NaN NaN NaN 66  124 248  NaN;    % 480 kHz
                NaN NaN NaN 33  62  124  148];   % 960 kHz
    scsIdx = log2(scs/60)+1*(scs<480);  % There are no entries for 240 kHz so adjust down index for larger SCS
    bws = [50 100 200 400 800 1600 2000];
end

bwIdx = find(bws==bw, 1);
if ~isempty(bwIdx)
  numRB = nrbtable(scsIdx, bwIdx);
end


end

