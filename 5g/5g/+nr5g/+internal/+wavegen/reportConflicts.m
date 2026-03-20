function reportConflicts(conflicts)
% Error out if conflicts among channels/signals exist
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

% Copyright 2021-2024 The MathWorks, Inc.

%#codegen

    isConflict = ~isempty(conflicts(1).Grid{1});
    if ~isConflict
        return
    end

    % Formatting functions for channel index and subindex
    subindexString = @(idx) repmat(sprintf(':%d',int16(idx)),1,int8(~isnan(idx)));
    indexString = @(idx) sprintf('{%d}',int16(idx));

    % Build error message string with pairs of channels/signal in conflict
    str = [];
    for c = 1:length(conflicts)

        cfl = conflicts(c);

        % Type and index of the first channel in conflict.
        type1 = char(cfl.ChannelType{1});
        c1 = cfl.ChannelIdx(1);
        s1 = cfl.ChannelSubidx(1);

        chText1 = type1;
        if ~strcmpi(type1,'SSBurst') % Add index and subindex for channels other than SSB
            chText1 = [type1 indexString(c1) subindexString(s1)];
        end

        % Type and index of the second channel in conflict.
        type2 = char(cfl.ChannelType{2});
        c2 = cfl.ChannelIdx(2);
        s2 = cfl.ChannelSubidx(2);

        chText2 = type2;
        if ~strcmpi(type2,'SSBurst') % Add index and subindex for channels other than SSB
            chText2 = [type2 indexString(c2) subindexString(s2)];
        end

        str = [str, sprintf(' (%s, %s),',chText1,chText2)];

    end

    coder.internal.error('nr5g:nrWaveformGenerator:Conflict', str(1:end-1));

end
