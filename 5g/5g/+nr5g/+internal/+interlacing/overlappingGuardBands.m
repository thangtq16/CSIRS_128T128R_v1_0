function [ogb,str] = overlappingGuardBands(guardBands)
%overlappingGuardBands Pairs of indices of the guard bands that overlap
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    % Sort guard bands
    [~,sorted] = sort(guardBands(:,1),'ascend');
    sortedGuardBands = guardBands(sorted,:);
    
    % Find overlaps between adjacent intracell guard bands
    % Zero-sized guard bands need special care, as they may give false
    % alarm of overlapping, or mask non-adjacent overlapping guard bands.
    % So, zero-sized guard bands should be ignored while the actual index
    % of guard bands should be recorded.
    %                R(1)                            actualIndex
    %   |--------------|                                  1
    %              || (zero-sized guard bands)           N/A
    %                                 R(2)
    %             |--------------------|                  3
    %           L(1)                     |----------|     4
    %                                  L(2)
    actualIndex = find(sortedGuardBands(:,2)~=0);                           % Index of non-empty guard bands in sortedGuardBands
    nonemptySortedGuardBands = sortedGuardBands(actualIndex,:);
    L = nonemptySortedGuardBands(2:end,1);
    R = sum(nonemptySortedGuardBands(1:end-1,:),2);
    overlapIndex = find(R > L);                                             % Index of overlapping guard bands in nonemptySortedGuardBands
   
    if ~isempty(overlapIndex)
        % Find the real index of overlapping guard bands in the original
        % matrix, guardBands
        indexInSorted = actualIndex(overlapIndex.' + [0;1]);
        ogb = sorted(indexInSorted);
    
        % Create string with pairs of overlapping guard bands
        coder.varsize('str',[1 Inf],[0 1]);
        str = '';
        for i=1:size(ogb,2)
            g = int8(sort(ogb(:,i)));
            str = [str, sprintf('(%d,%d), ',g(1),g(2))]; %#ok<AGROW>
        end
        str = str(1:end-2);
    else
        ogb = zeros(2,0);
        str = '';
    end

end