function [csirslocations,csirsParams] = getCSIRSLocations(carrier,csirs)
% getCSIRSLocations returns CSI-RS locations within a slot
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    [CSIRSLOCATIONS,CSIRSPARAMS] = nr5g.internal.getCSIRSLocations(CARRIER,CSIRS)
%    returns CSI-RS locations within a slot as defined in 
%    TS 38.211 Table 7.4.1.5.3-1, given carrier specific configuration
%    object CARRIER and CSI-RS specific configuration object CSIRS. The
%    function also returns updated parameters of CSI-RS in a structure
%    CSIRSPARAMS.

%  Copyright 2019-2021 The MathWorks, Inc.

    %#codegen

    % Validate CSI-RS specific configuration object
    csirsParams = validateConfig(csirs);

    % Get the number of CSI-RS resources configured
    numCSIRSRes = numel(csirsParams.CSIRSType);

    % Initialize the CSI-RS locations within a slot
    csirslocations = coder.nullcopy(cell(1,numCSIRSRes));
    for resIdx = 1:numCSIRSRes

        % Extract the following parameters for a CSI-RS resource based
        % on the resource index
        rowIndex     = csirsParams.RowNumber(resIdx);
        scLocations  = csirsParams.SubcarrierLocations{resIdx};
        symLocations = csirsParams.SymbolLocations{resIdx};
        numRB        = csirsParams.NumRB(resIdx);
        RBOffset     = csirsParams.RBOffset(resIdx);

        % Extract size of the grid and cyclic prefix
        NSizeGrid = double(carrier.NSizeGrid);
        cp        = carrier.CyclicPrefix;

        % Validate NumRB
        coder.internal.errorIf(numRB > NSizeGrid,'nr5g:nrCSIRS:InvalidNumRB',rowIndex,numRB,NSizeGrid);

        % Validate RBOffset
        maxOffset = (NSizeGrid - numRB);
        coder.internal.errorIf(RBOffset > maxOffset,'nr5g:nrCSIRS:InvalidRBOffset',rowIndex,RBOffset,maxOffset);

        % Validate the OFDM symbol locations based on row index and cyclic
        % prefix type
        validateSymbolLocations(cp,rowIndex,symLocations);

        % Get CSI-RS locations within a slot
        csirslocations{resIdx} =  getLocations(rowIndex,scLocations,symLocations);

    end

end

function locations =  getLocations(rowIndex,scLocations,symLocations)
%   LOCATIONS = getLocations(ROWINDEX,SCLOCATIONS,SYMLOCATIONS)
%   returns CSI-RS locations within a slot as defined in 
%   TS 38.211 Table 7.4.1.5.3-1 given the following inputs:
%
%   ROWINDEX      - Row index of TS 38.211 Table 7.4.1.5.3-1
%   SCLOCATIONS   - Subcarrier locations
%   SYMLOCATIONS  - OFDM symbol locations

    k = zeros(1,6); % For codegen purpose
    k(1:numel(scLocations)) = scLocations;

    % Cast the input 'symLocations' to double
    symLocations = double(symLocations);

    if isscalar(symLocations)
        l0 = symLocations(1);
        l1 = 0; % For codegen purpose
    else
        l0 = symLocations(1);
        l1 = symLocations(2);
    end

    % Get CSI-RS locations within a slot as defined in TS 38.211 Table 7.4.1.5.3-1
    CSIRSTable = {
             {[k(1),l0],[k(1)+4,l0],[k(1)+8,l0]},                {[0,0,0]},    {0},       {0};...    % Row 1
             {[k(1),l0]},                                           {0},       {0},       {0};...    % Row 2
             {[k(1),l0]},                                           {0},     {(0:1)},     {0};...    % Row 3
             {[k(1),l0],[k(1)+2,l0]},                             {(0:1)},   {(0:1)},     {0};...    % Row 4
             {[k(1),l0],[k(1),l0+1]},                             {(0:1)},   {(0:1)},     {0};...    % Row 5
             {[k(1),l0],[k(2),l0],[k(3),l0],[k(4),l0]},           {(0:3)},   {(0:1)},     {0};...    % Row 6
             {[k(1),l0],[k(2),l0],[k(1),l0+1],[k(2),l0+1]},       {(0:3)},   {(0:1)},     {0};...    % Row 7
             {[k(1),l0],[k(2),l0]},                               {(0:1)},   {(0:1)},   {(0:1)};...  % Row 8
             {[k(1),l0],[k(2),l0],[k(3),l0],...                   
              [k(4),l0],[k(5),l0],[k(6),l0]},                     {(0:5)},   {(0:1)},     {0};...    % Row 9
             {[k(1),l0],[k(2),l0],[k(3),l0]},                     {(0:2)},   {(0:1)},   {(0:1)};...  % Row 10
             {[k(1),l0],[k(2),l0],[k(3),l0],[k(4),l0],...
              [k(1),l0+1],[k(2),l0+1],[k(3),l0+1],[k(4),l0+1]},   {(0:7)},   {(0:1)},     {0};...    % Row 11
             {[k(1),l0],[k(2),l0],[k(3),l0],[k(4),l0]},           {(0:3)},   {(0:1)},   {(0:1)};...  % Row 12
             {[k(1),l0],[k(2),l0],[k(3),l0],...
              [k(1),l0+1],[k(2),l0+1],[k(3),l0+1],...
              [k(1),l1],[k(2),l1],[k(3),l1],...
              [k(1),l1+1],[k(2),l1+1],[k(3),l1+1]},               {(0:11)},  {(0:1)},     {0};...    % Row 13
             {[k(1),l0],[k(2),l0],[k(3),l0],...
              [k(1),l1],[k(2),l1],[k(3),l1]},                     {(0:5)},   {(0:1)},   {(0:1)};...  % Row 14
             {[k(1),l0],[k(2),l0],[k(3),l0]},                     {(0:2)},   {(0:1)},   {(0:3)};...  % Row 15
             {[k(1),l0],[k(2),l0],[k(3),l0],[k(4),l0],...
              [k(1),l0+1],[k(2),l0+1],[k(3),l0+1],[k(4),l0+1],...
              [k(1),l1],[k(2),l1],[k(3),l1],[k(4),l1],...
              [k(1),l1+1],[k(2),l1+1],[k(3),l1+1],[k(4),l1+1]},   {(0:15)},   {(0:1)},    {0};...    % Row 16
             {[k(1),l0],[k(2),l0],[k(3),l0],[k(4),l0],...
              [k(1),l1],[k(2),l1],[k(3),l1],[k(4),l1]},           {(0:7)},   {(0:1)},   {(0:1)};...  % Row 17
             {[k(1),l0],[k(2),l0],[k(3),l0],[k(4),l0]},           {(0:3)},   {(0:1)},   {(0:3)}      % Row 18
             };

    locations.kbar_lbar = CSIRSTable{rowIndex,1};
    locations.CDMGroupIndex_j = CSIRSTable{rowIndex,2}{1};
    locations.k_prime = CSIRSTable{rowIndex,3}{1};
    locations.l_prime = CSIRSTable{rowIndex,4}{1};

end

function validateSymbolLocations(cp,rowIndex,symLocations)
%   validateSymbolLocations(CP,ROWINDEX,SYMLOCATIONS) validates CSI-RS
%   symbol locations given the following inputs:
%
%   CP            - Cyclic prefix type
%   ROWINDEX      - Row index of TS 38.211 Table 7.4.1.5.3-1
%   SYMLOCATIONS  - OFDM symbol locations

    % Modified l_0 and l_1 ranges based on the row number and cyclic prefix
    % type
    if strcmpi(cp,'Normal')
        % Row:        1      2      3      4      5      6      7      8      9      10     11     12     13     14     15     16     17     18
        l0_range = {[0 13],[0 13],[0 13],[0 13],[0 12],[0 13],[0 12],[0 12],[0 13],[0 12],[0 12],[0 12],[0 12],[0 12],[0 10],[0 12],[0 12],[0 10]};
        l1_range = {[],[],[],[],[],[],[],[],[],[],[],[],[2 12],[2 12],[],[2 12],[2 12],[]};
    else
        l0_range = {[0 11],[0 11],[0 11],[0 11],[0 10],[0 11],[0 10],[0 10],[0 11],[0 10],[0 10],[0 10],[0 10],[0 10],[0 8],[0 10],[0 10],[0 8]};
        l1_range = {[],[],[],[],[],[],[],[],[],[],[],[],[2 10],[2 10],[],[2 10],[2 10],[]};
    end

    % Consider the number of OFDM symbols based on row index of
    % TS 38.211 Table 7.4.1.5.3-1
    numSymLocations = [1 1 1 1 1 1 1 1 1 1 1 1 2 2 1 2 2 1];

    % Validate CSI-RS symbol locations
    validateattributes(symLocations,{'numeric'},{'vector','numel',numSymLocations(rowIndex)},mfilename,'SymbolLocations');
    valid_l0_range = l0_range{rowIndex};
    if ~(symLocations(1) >= valid_l0_range(1) && symLocations(1) <= valid_l0_range(2))
        coder.internal.error('nr5g:nrCSIRS:Invalidl0',rowIndex,symLocations(1),valid_l0_range(1),valid_l0_range(2));
    end
    if numel(symLocations) == 2 % This case applies only for the rows 13,14,16 and 17
        valid_l1_range = l1_range{rowIndex};
        % check isempty for codegen, valid_l1_range should not be empty here at runtime
        if ~isempty(valid_l1_range) && ~(symLocations(2) >= valid_l1_range(1) && symLocations(2) <= valid_l1_range(2))
            coder.internal.error('nr5g:nrCSIRS:Invalidl1',rowIndex,symLocations(2),valid_l1_range(1),valid_l1_range(2));
        end
    end

end