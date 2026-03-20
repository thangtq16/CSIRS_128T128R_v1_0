function [dmrssymbolset,ldash] = lookupPUSCHDMRSSymbols(symbolset,typeB,dmrsTypeAPos,dmrsLength,dmrsAddPos,freqHopping)
%lookupPUSCHDMRSSymbols DM-RS symbol locations of PUSCH
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [DMRSSYMBOLSET,LDASH] = lookupPUSCHDMRSSymbols(SYMBOLSET,TYPEB,DMRSTYPEAPOS,DMRSLENGTH,DMRSADDPOS,FREQHOPPING)
%   returns the 0-based DM-RS OFDM symbol locations DMRSSYMBOLSET and
%   indication of DM-RS double-symbol according to TS 38.211 tables
%   7.4.1.1.2-3 and 7.4.1.1.2-4, given the inputs, vector of OFDM symbols
%   allocated for shared channel SYMBOLSET, indication of mapping type B
%   TYPEB, DM-RS type A position DMRSTYPEAPOS, DM-RS duration DMRSLENGTH,
%   DM-RS additional position DMRSADDPOS and intra-slot frequency hopping
%   flag FREQHOPPING.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    % lbar are the DM-RS positions (first symbol of pairs in the double symbol case)
    % but defined relative to the allocation start for mapping type B and
    % relative to the starting symbol of slot for mapping type A

    % Create static tables for mapping type (A/B) and single or
    % double-symbol as defined TS 38.211 tables 6.4.1.1.3-3 and
    % 6.4.1.1.3-4, and 6.4.1.1.3-6
    persistent dmrs_add_pos dmrs_add_pos_hopping;
    if isempty(dmrs_add_pos)

        % Additional position tables
        % Single-symbol, 0,1,2,3 *additional* symbols
        dmrs_singleA = {
            [],    [],      [],        [];  %  1 symbol duration
            [],    [],      [],        [];  %  2 symbol duration
            [],    [],      [],        [];  %  3 symbol duration
             0,     0,       0,         0;  %  4 symbol duration
             0,     0,       0,         0;  %  5 symbol duration
             0,     0,       0,         0;  %  6 symbol duration
             0,     0,       0,         0;  %  7 symbol duration
             0, [0,7],   [0,7],     [0,7];  %  8 symbol duration
             0, [0,7],   [0,7],     [0,7];  %  9 symbol duration
             0, [0,9], [0,6,9],   [0,6,9];  % 10 symbol duration
             0, [0,9], [0,6,9],   [0,6,9];  % 11 symbol duration
             0, [0,9], [0,6,9],[0,5,8,11];  % 12 symbol duration
             0,[0,11],[0,7,11],[0,5,8,11];  % 13 symbol duration
             0,[0,11],[0,7,11],[0,5,8,11];  % 14 symbol duration
        };
        dmrs_singleB = {
              0,      0,       0,        0;   %  1 symbol duration
              0,      0,       0,        0;   %  2 symbol duration
              0,      0,       0,        0;   %  3 symbol duration
              0,      0,       0,        0;   %  4 symbol duration
              0,  [0,4],   [0,4],    [0,4];   %  5 symbol duration
              0,  [0,4],   [0,4],    [0,4];   %  6 symbol duration
              0,  [0,4],   [0,4],    [0,4];   %  7 symbol duration
              0,  [0,6], [0,3,6],  [0,3,6];   %  8 symbol duration
              0,  [0,6], [0,3,6],  [0,3,6];   %  9 symbol duration
              0,  [0,8], [0,4,8],[0,3,6,9];   % 10 symbol duration
              0,  [0,8], [0,4,8],[0,3,6,9];   % 11 symbol duration
              0, [0,10],[0,5,10],[0,3,6,9];   % 12 symbol duration
              0, [0,10],[0,5,10],[0,3,6,9];   % 13 symbol duration
              0, [0,10],[0,5,10],[0,3,6,9];   % 14 symbol duration
        };
        % Double-symbol, 0,1,2,3 *additional* symbol *pairs*
        dmrs_doubleA = {
            [],    [],[],[];    %  1 symbol duration
            [],    [],[],[];    %  2 symbol duration
            [],    [],[],[];    %  3 symbol duration
             0,     0,[],[];    %  4 symbol duration
             0,     0,[],[];    %  5 symbol duration
             0,     0,[],[];    %  6 symbol duration
             0,     0,[],[];    %  7 symbol duration
             0,     0,[],[];    %  8 symbol duration
             0,     0,[],[];    %  9 symbol duration
             0, [0,8],[],[];    % 10 symbol duration
             0, [0,8],[],[];    % 11 symbol duration
             0, [0,8],[],[];    % 12 symbol duration
             0,[0,10],[],[];    % 13 symbol duration
             0,[0,10],[],[];    % 14 symbol duration
        };
        dmrs_doubleB = {
            [],   [],[],[];    %  1 symbol duration
            [],   [],[],[];    %  2 symbol duration
            [],   [],[],[];    %  3 symbol duration
            [],   [],[],[];    %  4 symbol duration
             0,    0,[],[];    %  5 symbol duration
             0,    0,[],[];    %  6 symbol duration
             0,    0,[],[];    %  7 symbol duration
             0,[0,5],[],[];    %  8 symbol duration
             0,[0,5],[],[];    %  9 symbol duration
             0,[0,7],[],[];    % 10 symbol duration
             0,[0,7],[],[];    % 11 symbol duration
             0,[0,9],[],[];    % 12 symbol duration
             0,[0,9],[],[];    % 13 symbol duration
             0,[0,9],[],[];    % 14 symbol duration
        };

        % Combined tables, indexed as tables{type,length}
        dmrs_add_pos = { dmrs_singleA, dmrs_doubleA;
                         dmrs_singleB, dmrs_doubleB };

        % Frequency hopping cases (dimensioned for a max half slot hop)
        % Single symbol only, no double symbol configurations defined
        %
        % Type A, starting symbol 2 case
        % 0 add pos (first/second hop) / 1 add pos (first/second hop)
        dmrs_singleA_2FreqHop = {
            [],[],   [],   [];     % 1 symbol duration
            [],[],   [],   [];     % 2 symbol duration
            [],[],   [],   [];     % 3 symbol duration
             2, 0,    2,    0;     % 4 symbol duration
             2, 0,    2,[0,4];     % 5 symbol duration
             2, 0,    2,[0,4];     % 6 symbol duration
             2, 0,[2,6],[0,4];     % 7 symbol duration
        };
        % Type A, starting symbol 3 case
        % 0 add pos (first/second hop) / 1 add pos (first/second hop)
        dmrs_singleA_3FreqHop = {
            [],[],[],   [];     % 1 symbol duration
            [],[],[],   [];     % 2 symbol duration
            [],[],[],   [];     % 3 symbol duration
             3, 0, 3,    0;     % 4 symbol duration
             3, 0, 3,[0,4];     % 5 symbol duration
             3, 0, 3,[0,4];     % 6 symbol duration
             3, 0, 3,[0,4];     % 7 symbol duration
        };
        % Type B
        % 0 add pos (first/second hop) / 1 add pos (first/second hop)
        dmrs_singleB_FreqHop = {
            0,0,    0,    0;  % 1 symbol duration
            0,0,    0,    0;  % 2 symbol duration
            0,0,    0,    0;  % 3 symbol duration
            0,0,    0,    0;  % 4 symbol duration
            0,0,[0,4],[0,4];  % 5 symbol duration
            0,0,[0,4],[0,4];  % 6 symbol duration
            0,0,[0,4],[0,4];  % 7 symbol duration
        };
        dmrs_add_pos_hopping = { dmrs_singleA_2FreqHop, dmrs_singleA_3FreqHop, dmrs_singleB_FreqHop };
    end

    if ~isempty(symbolset)
        [lb,ub] = bounds(symbolset);
        if ~typeB
            lb = 0;
        end
        nsymbols = ub - lb + 1;
    else
        nsymbols = 0;
    end

    % Different processing is required depending on whether frequency
    % hopping is enabled or not
    dmrssymbolset = [];
    pos1 = 0;
    if nsymbols
        if freqHopping
            % Get the relevant single symbol hopping table
            if dmrsLength == 1 || dmrsAddPos <= 1  % No DM-RS for double symbols or addpos > 1 defined
                positionstable = dmrs_add_pos_hopping{ ~typeB*(dmrsTypeAPos-1) + typeB*3 };
                % Get the hop duration dependent symbol DM-RS position information
                n1 = floor(nsymbols/2); % Number of symbols in first hop
                n2 = nsymbols - n1;     % Number of symbols in second hop
                % First/second hop positions, defined relative to start of each hop
                if n1
                    pos1 = positionstable{n1,2*dmrsAddPos+1};
                else
                    pos1 = [];   % Degenerate case of no first hop
                end
                pos2 = positionstable{n2,2*dmrsAddPos+2};
                dmrssymbolset = [pos1,n1+pos2];  % Combine and adjust second position
            end
        else
            % Otherwise get the relevant non-hopping table
            positionstable = dmrs_add_pos{ typeB+1, dmrsLength };
            % Get the duration dependent symbol DM-RS position information
            if dmrsAddPos < size(positionstable,2)
                dmrssymbolset = positionstable{nsymbols,1+dmrsAddPos};
            end
        end
    end

    ldash = zeros(1,length(dmrssymbolset)*dmrsLength);
    % Adjust table information
    if ~isempty(dmrssymbolset)
        % Adjust indices for the relative offset of the mapping type
        if typeB
            dmrssymbolset = dmrssymbolset+symbolset(1); % If type B (non slot-wise)
        else
            if ~isempty(pos1)
                dmrssymbolset(1) = dmrsTypeAPos;        % If type A (slot-wise) then 2 or 3
            end
        end
        % Adjust for double-symbol DM-RS
        % In the operational system, if RRC configured with max length 2
        % then the actual length is DCI signaled
        if dmrsLength == 2
            dmrssymbolset = reshape([dmrssymbolset; dmrssymbolset+1],1,[]);
            ldash(2:2:end) = 1;
        end
        % For non-standard set-ups, only return the DM-RS symbol indices
        % that overlap the actual allocation indices
        logicalMatrix = repmat(symbolset(:),1,numel(dmrssymbolset)) == repmat(dmrssymbolset,numel(symbolset),1);
        ind = sum(logicalMatrix,1)==1;
        dmrssymbolset = dmrssymbolset(ind);
        ldash = ldash(ind);
    else
        dmrssymbolset = zeros(1,0);
    end
end
