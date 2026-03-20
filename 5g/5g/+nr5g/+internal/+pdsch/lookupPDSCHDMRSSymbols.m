function [dmrssymbolset,ldash] = lookupPDSCHDMRSSymbols(symbolset,typeB,dmrsTypeAPos,dmrsLength,dmrsAddPos,varargin)
%lookupPDSCHDMRSSymbols DM-RS symbol locations of PDSCH
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [DMRSSYMBOLSET,LDASH] = lookupPDSCHDMRSSymbols(SYMBOLSET,TYPEB,DMRSTYPEAPOS,DMRSLENGTH,DMRSADDPOS)
%   returns the 0-based DM-RS OFDM symbol locations DMRSSYMBOLSET and
%   indication of DM-RS double-symbol according to TS 38.211 tables
%   7.4.1.1.2-3 and 7.4.1.1.2-4, given the inputs, vector of OFDM symbols
%   allocated for shared channel SYMBOLSET, indication of mapping type B
%   TYPEB, DM-RS type A position DMRSTYPEAPOS, DM-RS duration DMRSLength
%   and DM-RS additional position DMRSADDPOS.
%
%   Example:
%   % Get the DM-RS symbol locations for the physical shared channel with
%   % symbols allocated 0 to 10, mapping type A, DM-RS type A position set
%   % to 2, single-symbol DM-RS and DM-RS additional position set to 2.
%
%   symbolset = 0:10;
%   typeB = 0; % 1 for mapping type B, 0 for mapping type A
%   dmrsTypeAPos = 2;
%   dmrsLen = 1; % 1 for single-symbol DM-RS, 2 for double-symbol DM-RS
%   dmrsAddPos = 2;
%   dmrssymbolset = ...
%   nr5g.internal.pdsch.lookupPDSCHDMRSSymbols(symbolset,typeB,dmrsTypeAPos,dmrsLen,dmrsAddPos)

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    % lbar (tables below) are the DM-RS positions (first symbol of pairs in
    % the double symbol case) but defined relative to the allocation start
    %
    % Create static tables for mapping type (A/B) and single or
    % double-symbol TS 38.211 Tables 7.4.1.1.2-3 and 7.4.1.1.2-4
    persistent dmrs_add_pos;
    if isempty(dmrs_add_pos)

        % Additional position tables
        % Type A, single-symbol, 0,1,2,3 *additional* symbols - TS 38.211 Table 7.4.1.1.2-3, type A part
        dmrs_singleA = {
            [],[],  [],  [];                %  1 symbol duration
            [],[],  [],  [];                %  2 symbol duration
            0,  0,  0,    0;                %  3 symbol duration
            0,  0,  0,    0;                %  4 symbol duration
            0,  0,  0,    0;                %  5 symbol duration
            0,  0,  0,    0;                %  6 symbol duration
            0,  0,  0,    0;                %  7 symbol duration
            0,  [0,7],  [0,7],  [0,7];      %  8 symbol duration
            0,  [0,7],  [0,7],  [0,7];      %  9 symbol duration
            0,  [0,9], [0,6,9], [0,6,9];    % 10 symbol duration
            0,  [0,9], [0,6,9], [0,6,9];    % 11 symbol duration
            0,  [0,9], [0,6,9], [0,5,8,11]; % 12 symbol duration
            0, [0,-1], [0,7,11],[0,5,8,11]; % 13 symbol duration,  -1 represents l_1 = 11/12
            0, [0,-1], [0,7,11],[0,5,8,11]; % 14 symbol duration,  -1 represents l_1 = 11/12
        };
        % Type B, single-symbol, 0,1,2,3 *additional* symbols - TS 38.211 Table 7.4.1.1.2-3, type B part
        dmrs_singleB = {
            [],[],  [],  [];            %  1 symbol duration
             0, 0,   0,   0;            %  2 symbol duration
             0, 0,   0,   0;            %  3 symbol duration
             0, 0,   0,   0;            %  4 symbol duration
             0,[0,4],[0,4],  [0,4];     %  5 symbol duration
             0,[0,4],[0,4],  [0,4];     %  6 symbol duration (extended CP, half slot)
             0,[0,4],[0,4],  [0,4];     %  7 symbol duration (normal CP, half slot)
             0,[0,6],[0,3,6],[0,3,6];   %  8 symbol duration
             0,[0,7],[0,4,7],[0,4,7];   %  9 symbol duration     
             0,[0,7],[0,4,7],[0,4,7];   % 10 symbol duration            
             0,[0,8],[0,4,8],[0,3,6,9]; % 11 symbol duration         
             0,[0,9],[0,5,9],[0,3,6,9]; % 12 symbol duration            
             0,[0,9],[0,5,9],[0,3,6,9]; % 13 symbol duration
            [],[],  [],  [];            % 14 symbol duration
        };
        % Type A, double-symbol, 0,1(,2) *additional* symbol *pairs* - TS 38.211 Table 7.4.1.1.2-4, type A part
        dmrs_doubleA = {
            [],[];     %  1 symbol duration
            [],[];     %  2 symbol duration
            [],[];     %  3 symbol duration
             0, 0;     %  4 symbol duration
             0, 0;     %  5 symbol duration
             0, 0;     %  6 symbol duration
             0, 0;     %  7 symbol duration
             0, 0;     %  8 symbol duration
             0, 0;     %  9 symbol duration
             0,[0,8];  % 10 symbol duration
             0,[0,8];  % 11 symbol duration
             0,[0,8];  % 12 symbol duration
             0,[0,10]; % 13 symbol duration
             0,[0,10]; % 14 symbol duration
        };
        % Type B, double-symbol, 0,1(,2) *additional* symbol *pairs* - TS 38.211 Table 7.4.1.1.2-4, type B part
        dmrs_doubleB = {
            [],[];    %  1 symbol duration
            [],[];    %  2 symbol duration
            [],[];    %  3 symbol duration
            [],[];    %  4 symbol duration
             0, 0;    %  5 symbol duration -
             0, 0;    %  6 symbol duration R15 (ECP, half slot)
             0, 0;    %  7 symbol duration R15 (NCP, half slot)
             0,[0,5]; %  8 symbol duration -
             0,[0,5]; %  9 symbol duration -
             0,[0,7]; % 10 symbol duration -
             0,[0,7]; % 11 symbol duration -
             0,[0,8]; % 12 symbol duration -
             0,[0,8]; % 13 symbol duration -
            [],[];    % 14 symbol duration
        };

        % Combined tables, indexed as tables{mapping type,length}
        %                   Single        Double
        dmrs_add_pos = { dmrs_singleA, dmrs_doubleA;        % Type A mapping
                         dmrs_singleB, dmrs_doubleB };      % Type B mapping
    end

    % Look up relevant table from the set
    positionstable = dmrs_add_pos{typeB+1, dmrsLength};

    if ~isempty(symbolset)
        [lb,ub] = bounds(symbolset);
        if ~typeB
            lb = 0;
        end
        nsymbols = ub - lb + 1;
    else
        nsymbols = 0;
    end

    % Get the duration dependent symbol DM-RS position information
    if (dmrsAddPos < size(positionstable,2)) && nsymbols
        dmrssymbolset = positionstable{nsymbols, 1+dmrsAddPos};
    else
        dmrssymbolset = [];
    end

    % Adjust for l_1 case (introduced in TS 38.211 v15.4.0)
    % Last DM-RS position is 11, or can also be 12 in the presence of LTE
    % CRS and other conditions defined in TS 38.211 Section 7.4.1.1.2. Here
    % the value is fixed to 11.
    if ~isempty(dmrssymbolset) && dmrssymbolset(end)==-1
        dmrssymbolset(end) = 11;
    end
    ldash = zeros(1,length(dmrssymbolset)*dmrsLength);
    % Adjust table information
    if ~isempty(dmrssymbolset)
        % Adjust indices for the relative offset of the mapping type
        if typeB
            dmrssymbolset = dmrssymbolset+symbolset(1); % If type B (non slot-wise)
        else
            dmrssymbolset(1) = dmrsTypeAPos;            % If type A (slot-wise) then 2 or 3
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