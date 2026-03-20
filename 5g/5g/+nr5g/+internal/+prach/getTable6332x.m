function out = getTable6332x(tableIdx)
%getTable6332x Get TS 38.211 Tables 6.3.3.2-1 to 6.3.3.2-4.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    OUT = nr5g.internal.prach.getTable6332x(TABLEIDX) returns TS 38.211
%    Tables 6.3.3.2-x, in which 'x' is defined by the input parameter
%    'TABLEIDX'.

%    Copyright 2019-2022 The MathWorks, Inc.

%#codegen

persistent table63321;
persistent table63322;
persistent table63323;
persistent table63324;

if isempty(table63321)
    
    %% Define TS 38.211 Table 6.3.3.2-1
    % Each column represents the following:
    %   Column 1: Length of the preamble sequence
    %   Column 2: PRACH subcarrier spacing
    %   Column 3: PUSCH subcarrier spacing
    %   Column 4: Allocation expressed in number of RBs for PUSCH (N_RB^RA)
    %   Column 5: kbar
    
    t63321 = {
               839  1.25   15   6    7;
               839  1.25   30   3    1;
               839  1.25   60   2  133;
               839     5   15  24   12;
               839     5   30  12   10;
               839     5   60   6    7;
               139    15   15  12    2;
               139    15   30   6    2;
               139    15   60   3    2;
               139    30   15  24    2;
               139    30   30  12    2;
               139    30   60   6    2;
               139    60   60  12    2;
               139    60  120   6    2;
               139   120   60  24    2;
               139   120  120  12    2;
               139   120  480   3    1;
               139   120  960   2   23;
               139   480  120  48    2;
               139   480  480  12    2;
               139   480  960   6    2;
               139   960  120  96    2;
               139   960  480  24    2;
               139   960  960  12    2;
               571    30   15  96    2;
               571    30   30  48    2;
               571    30   60  24    2;
               571   120  120  48    2;
               571   120  480  12    1;
               571   120  960   7   47;
               571   480  120 192    2;
               571   480  480  48    2;
               571   480  960  24    2;
              1151    15   15  96    1;
              1151    15   30  48    1;
              1151    15   60  24    1;
              1151   120  120  97    6;
              1151   120  480  25   23;
              1151   120  960  13   45;
             };
    table63321 = cell2table(t63321,'VariableNames',{'LRA','PRACHSubcarrierSpacing','PUSCHSubcarrierSpacing','NRBAllocation','kbar'});
    table63321.Properties.Description = 'TS 38.211 Table 6.3.3.2-1: Supported combinations of subcarrier spacing between PRACH and PUSCH';
    
    %% Define TS 38.211 Table 6.3.3.2-2
    % Each column represents the following:
    %   Column 1: Configuration index
    %   Column 2: Preamble format
    %   Column 3: x
    %   Column 4: y
    %   Column 5: Subframe number
    %   Column 6: Starting symbol
    %   Column 7: Number of PRACH slots within a subframe
    %   Column 8: Number of time-domain PRACH occasions within a PRACH slot (Nslot_t)
    %   Column 9: PRACH duration (N_dur)
    
    [ConfigurationIndex,Format,x,y,sfn,StartingSymbol,SlotsPerSF,NumTimeOccasions,PRACHDuration] = ...
        nr5g.internal.prach.getConfigurationTables(2);
    table63322 = table(ConfigurationIndex,Format,x,y,sfn,StartingSymbol,SlotsPerSF,NumTimeOccasions,PRACHDuration,...
        'VariableNames',{'ConfigurationIndex','PreambleFormat','x','y','SubframeNumber','StartingSymbol','PRACHSlotsPerSubframe','NumTimeOccasions','PRACHDuration'});
    table63322.Properties.Description = 'TS 38.211 Table 6.3.3.2-2: Random access configurations for FR1 and paired spectrum/supplementary uplink';
    
    %% Define TS 38.211 Table 6.3.3.2-3
    % Each column represents the following:
    %   Column 1: Configuration index
    %   Column 2: Preamble format
    %   Column 3: x
    %   Column 4: y
    %   Column 5: Subframe number
    %   Column 6: Starting symbol
    %   Column 7: Number of PRACH slots within a subframe
    %   Column 8: Number of time-domain PRACH occasions within a PRACH slot (Nslot_t)
    %   Column 9: PRACH duration (N_dur)
    
    [ConfigurationIndex,Format,x,y,sfn,StartingSymbol,SlotsPerSF,NumTimeOccasions,PRACHDuration] = ...
        nr5g.internal.prach.getConfigurationTables(3);
    table63323 = table(ConfigurationIndex,Format,x,y,sfn,StartingSymbol,SlotsPerSF,NumTimeOccasions,PRACHDuration,...
        'VariableNames',{'ConfigurationIndex','PreambleFormat','x','y','SubframeNumber','StartingSymbol','PRACHSlotsPerSubframe','NumTimeOccasions','PRACHDuration'});
    table63323.Properties.Description = 'TS 38.211 Table 6.3.3.2-3: Random access configurations for FR1 and unpaired spectrum';
    
    %% Define TS 38.211 Table 6.3.3.2-4
    % Each column represents the following:
    %   Column 1: Configuration index
    %   Column 2: Preamble format
    %   Column 3: x
    %   Column 4: y
    %   Column 5: Slot number
    %   Column 6: Starting symbol
    %   Column 7: Number of PRACH slots within a 60 kHz slot
    %   Column 8: Number of time-domain PRACH occasions within a PRACH slot (Nslot_t)
    %   Column 9: PRACH duration (N_dur)
    
    [ConfigurationIndex,Format,x,y,sfn,StartingSymbol,SlotsPerSF,NumTimeOccasions,PRACHDuration] = ...
        nr5g.internal.prach.getConfigurationTables(4);
    table63324 = table(ConfigurationIndex,Format,x,y,sfn,StartingSymbol,SlotsPerSF,NumTimeOccasions,PRACHDuration,...
        'VariableNames',{'ConfigurationIndex','PreambleFormat','x','y','SlotNumber','StartingSymbol','PRACHSlotsPer60kHzSlot','NumTimeOccasions','PRACHDuration'});
    table63324.Properties.Description = 'TS 38.211 Table 6.3.3.2-4: Random access configurations for FR2 and unpaired spectrum';
end

%% Output
switch tableIdx
    
    case 1 % TS 38.211 Table 6.3.3.2-1
        out = table63321;
    case 2 % TS 38.211 Table 6.3.3.2-2
        out = table63322;
    case 3 % TS 38.211 Table 6.3.3.2-3
        out = table63323;
    case 4 % TS 38.211 Table 6.3.3.2-4
        out = table63324;
    otherwise
        coder.internal.error('nr5g:nrPRACH:InvalidTable6332x');
end

end