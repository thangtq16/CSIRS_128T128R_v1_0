function [ConfigurationIndex,Format,x,y,sfn,StartingSymbol,SlotsPerSF,NumTimeOccasions,PRACHDuration] = getConfigurationTables(tableIdx)
%getConfigurationTables Get TS 38.211 Tables 6.3.3.2-2 to 6.3.3.2-4.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    OUT = nr5g.internal.prach.getConfigurationTables(TABLEIDX) returns
%    TS 38.211 Table 6.3.3.2-2, 6.3.3.2-3, and 6.3.3.2-4. The actual table
%    returned is defined by the input TABLEIDX.
%
%    Note that the output is a structure containing the following fields,
%    which are one for each column of the table:
%
%       * ConfigurationIndex
%       * Format
%       * x
%       * y
%       * sfn
%       * StartingSymbol
%       * SlotsPerSF
%       * NumTimeOccasions
%       * PRACHDuration

%    Copyright 2019-2021 The MathWorks, Inc.

%#codegen
%#ok<*NBRAK>

persistent t63322;
persistent t63323;
persistent t63324;

if isempty(t63322)
    
    %% Define TS 38.211 Table 6.3.3.2-2
    % Each field represents the following:
    %   * Configuration index
    %   * Preamble format
    %   * x
    %   * y
    %   * Subframe number
    %   * Starting symbol
    %   * Number of PRACH slots within a subframe
    %   * Number of time-domain PRACH occasions within a PRACH slot (Nslot_t)
    %   * PRACH duration (N_dur)
    
    t63322.ConfigurationIndex = [  0;   1;   2;   3;   4;   5;   6;   7;   8;   9;  10;  11;  12;  13;  14;  15;  16;  17;  18;  19; ... % ConfigurationIndex 0:19
                                  20;  21;  22;  23;  24;  25;  26;  27;  28;  29;  30;  31;  32;  33;  34;  35;  36;  37;  38;  39; ... % ConfigurationIndex 20:39
                                  40;  41;  42;  43;  44;  45;  46;  47;  48;  49;  50;  51;  52;  53;  54;  55;  56;  57;  58;  59; ... % ConfigurationIndex 40:59
                                  60;  61;  62;  63;  64;  65;  66;  67;  68;  69;  70;  71;  72;  73;  74;  75;  76;  77;  78;  79; ... % ConfigurationIndex 60:79
                                  80;  81;  82;  83;  84;  85;  86;  87;  88;  89;  90;  91;  92;  93;  94;  95;  96;  97;  98;  99; ... % ConfigurationIndex 80:99
                                 100; 101; 102; 103; 104; 105; 106; 107; 108; 109; 110; 111; 112; 113; 114; 115; 116; 117; 118; 119; ... % ConfigurationIndex 100:119
                                 120; 121; 122; 123; 124; 125; 126; 127; 128; 129; 130; 131; 132; 133; 134; 135; 136; 137; 138; 139; ... % ConfigurationIndex 120:139
                                 140; 141; 142; 143; 144; 145; 146; 147; 148; 149; 150; 151; 152; 153; 154; 155; 156; 157; 158; 159; ... % ConfigurationIndex 140:159
                                 160; 161; 162; 163; 164; 165; 166; 167; 168; 169; 170; 171; 172; 173; 174; 175; 176; 177; 178; 179; ... % ConfigurationIndex 160:179
                                 180; 181; 182; 183; 184; 185; 186; 187; 188; 189; 190; 191; 192; 193; 194; 195; 196; 197; 198; 199; ... % ConfigurationIndex 180:199
                                 200; 201; 202; 203; 204; 205; 206; 207; 208; 209; 210; 211; 212; 213; 214; 215; 216; 217; 218; 219; ... % ConfigurationIndex 200:219
                                 220; 221; 222; 223; 224; 225; 226; 227; 228; 229; 230; 231; 232; 233; 234; 235; 236; 237; 238; 239; ... % ConfigurationIndex 220:239
                                 240; 241; 242; 243; 244; 245; 246; 247; 248; 249; 250; 251; 252; 253; 254; 255; 256; 257; 258; 259; ... % ConfigurationIndex 240:259
                                 260; 261; 262];                                                                                         % ConfigurationIndex 260:262

    t63322.Format = {    '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0'; ... % ConfigurationIndex 0:19
                         '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1'; ... % ConfigurationIndex 20:39
                         '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '1';     '2';     '2';     '2';     '2';     '2';     '2';     '2'; ... % ConfigurationIndex 40:59
                         '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3'; ... % ConfigurationIndex 60:79
                         '3';     '3';     '3';     '3';     '3';     '3';     '3';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1'; ... % ConfigurationIndex 80:99
                        'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1';    'A2';    'A2';    'A2'; ... % ConfigurationIndex 100:119
                        'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; ... % ConfigurationIndex 120:139
                     'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3'; ... % ConfigurationIndex 140:159
                        'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3';    'B1';    'B1';    'B1'; ... % ConfigurationIndex 160:179
                        'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B4';    'B4'; ... % ConfigurationIndex 180:199
                        'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'C0'; ... % ConfigurationIndex 200:219
                        'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C2';    'C2';    'C2';    'C2'; ... % ConfigurationIndex 220:239
                        'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';     '-';     '-';     '-';     '-'; ... % ConfigurationIndex 240:259
                         '-';     '-';    '-'};                                                                                                                                                              % ConfigurationIndex 260:262

    t63322.x = [ 16;  16;  16;  16;   8;   8;   8;   8;   4;   4;   4;   4;   2;   2;   2;   2;   1;   1;   1;   1; ... % ConfigurationIndex 0:19
                  1;   1;   1;   1;   1;   1;   1;   1;  16;  16;  16;  16;   8;   8;   8;   8;   4;   4;   4;   4; ... % ConfigurationIndex 20:39
                  2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   1;   1; ... % ConfigurationIndex 40:59
                 16;  16;  16;  16;   8;   8;   8;   4;   4;   4;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1; ... % ConfigurationIndex 60:79
                  1;   1;   1;   1;   1;   1;   1;  16;  16;   8;   8;   4;   4;   4;   2;   2;   2;   2;   1;   1; ... % ConfigurationIndex 80:99
                  1;   1;   1;   1;   1;   1;   1;   1;   2;   2;   1;   1;   1;   1;   1;   1;   1;  16;  16;   8; ... % ConfigurationIndex 100:119
                  8;   4;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   2;   2;   1; ... % ConfigurationIndex 120:139
                  1;   1;   1;   1;   1;   1;   1;  16;  16;   8;   8;   4;   4;   2;   2;   2;   2;   1;   1;   1; ... % ConfigurationIndex 140:159
                  1;   1;   1;   1;   1;   1;   1;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;  16;  16;   8; ... % ConfigurationIndex 160:179
                  8;   4;   4;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;  16; ... % ConfigurationIndex 180:199
                  8;   8;   4;   4;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   8; ... % ConfigurationIndex 200:219
                  4;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;  16;   8;   8; ... % ConfigurationIndex 220:239
                  4;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63322.y = { 1;   1;   1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;   0;   0;   0;   0; ... % ConfigurationIndex 0:19
                 0;   0;   0;  0;  0;  0;  0;  0;  1;  1;  1;  1;  1;  1;  1;  1;   1;   1;   1;   1; ... % ConfigurationIndex 20:39
                 1;   1;   1;  1;  0;  0;  0;  0;  0;  0;  0;  0;  0;  1;  1;  0;   0;   0;   0;   0; ... % ConfigurationIndex 40:59
                 1;   1;   1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  0;   0;   0;   0;   0; ... % ConfigurationIndex 60:79
                 0;   0;   0;  0;  0;  0;  0;  0;  1;  0;  1;  0;  1;  0;  0;  0;   0;   0;   0;   0; ... % ConfigurationIndex 80:99
                 0;   0;   0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;   0;   1;   1;   1; ... % ConfigurationIndex 100:119
                 1;   0;   0;  1;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;   0;   1;   0;   0; ... % ConfigurationIndex 120:139
                 0;   0;   0;  0;  0;  0;  0;  1;  1;  1;  1;  0;  0;  1;  0;  0;   0;   0;   0;   0; ... % ConfigurationIndex 140:159
                 0;   0;   0;  0;  0;  0;  0;  1;  0;  0;  0;  0;  0;  0;  0;  0;   0;   0;   1;   0; ... % ConfigurationIndex 160:179
                 1;   0;   1;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;   0;   0;   0;   1; ... % ConfigurationIndex 180:199
                 0;   1;   0;  0;  1;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;   0;   0;   0;   1; ... % ConfigurationIndex 200:219
                 1;   0;   0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;   1;   1;   1;   1; ... % ConfigurationIndex 220:239
                 0;   0;   1;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
               NaN; NaN; NaN};                                                                            % ConfigurationIndex 260:262

    t63322.sfn = {   1;       4;       7;       9;           1;       4;       7;       9;     1;     4;       7;       9;       1;       4;       7;       9;       1;       4;       7;   [1 6]; ... % ConfigurationIndex 0:19
                 [2 7];   [3 8]; [1 4 7]; [2 5 8];     [3 6 9]; [0:2:8]; [1:2:9];   [0:9];     1;     4;       7;       9;       1;       4;       7;       9;       1;       4;       7;       9; ... % ConfigurationIndex 20:39
                     1;       4;       7;       9;           1;       4;       7;   [1 6]; [2 7]; [3 8]; [1 4 7]; [2 5 8]; [3 6 9];       1;       1;       1;       1;       5;       1;       5; ... % ConfigurationIndex 40:59
                     1;       4;       7;       9;           1;       4;       7;       1;     4;     7;       9;       1;       4;       7;       9;       1;       4;       7;   [1 6];   [2 7]; ... % ConfigurationIndex 60:79
                 [3 8]; [1 4 7]; [2 5 8]; [3 6 9]; [0 2 4 6 8]; [1:2:9];   [0:9];   [4 9];     4; [4 9];       4;   [4 9];   [4 9];       4;   [4 9];       1;       4;       7;       4;   [1 6]; ... % ConfigurationIndex 80:99
                 [4 9];       1;       7;   [2 7];     [1 4 7]; [0:2:8];   [0:9]; [1:2:9]; [4 9];     4;       4;   [1 6];   [4 9];       1;       7; [1 4 7]; [0:2:8]; [2 6 9];       4; [2 6 9]; ... % ConfigurationIndex 100:119
                     4; [2 6 9];       4; [2 6 9];           1;       4;       7;       4; [1 6]; [4 9];       1;       7;   [2 7]; [1 4 7]; [0:2:8];   [0:9]; [1:2:9]; [2 6 9];       4;       4; ... % ConfigurationIndex 120:139
                 [1 6];   [4 9];       1;       7;     [1 4 7]; [0:2:8];   [0:9];   [4 9];     4; [4 9];       4;   [4 9];       4; [2 6 9];       1;       4;       7;       4;   [1 6];   [4 9]; ... % ConfigurationIndex 140:159
                     1;       7;   [2 7]; [1 4 7]; [0 2 4 6 8];   [0:9]; [1:2:9]; [2 6 9];     4;     4;   [1 6];   [4 9];       1;       7; [1 4 7]; [0:2:8];   [0:9];   [4 9];       4;   [4 9]; ... % ConfigurationIndex 160:179
                     4;   [4 9];   [4 9];       4;       [4 9];       1;       4;       7;     4; [1 6];   [4 9];       1;       7;   [2 7]; [1 4 7]; [0:2:8];   [0:9]; [1:2:9];   [4 9];       4; ... % ConfigurationIndex 180:199
                 [4 9];       4;   [4 9];       4;       [4 9];   [4 9];       1;       4;     7;     1;       4;       7;   [1 6];   [2 7];   [4 9]; [1 4 7]; [0:2:8];   [0:9]; [1:2:9];       4; ... % ConfigurationIndex 200:219
                 [4 9];       4;   [4 9];       1;           4;       7;       4;   [1 6]; [4 9];     1;       7;   [2 7]; [1 4 7]; [0:2:8];   [0:9]; [1:2:9];   [4 9];       4;   [4 9];       4; ... % ConfigurationIndex 220:239
                 [4 9];       4; [2 6 9];       1;           4;       7;       4;   [1 6]; [4 9];     1;       7;   [2 7]; [1 4 7]; [0:2:8];   [0:9]; [1:2:9];     NaN;     NaN;     NaN;     NaN; ... % ConfigurationIndex 240:259
                   NaN;     NaN;     NaN};                                                                                                                                                             % ConfigurationIndex 260:262

    t63322.StartingSymbol = [  0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 0:19
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 20:39
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 40:59
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 60:79
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 80:99
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 100:119
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 120:139
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 140:159
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 160:179
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 180:199
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 200:219
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 220:239
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                             NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63322.SlotsPerSF = [NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 0:19
                         NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 20:39
                         NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 40:59
                         NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 60:79
                         NaN; NaN; NaN; NaN; NaN; NaN; NaN;   1;   2;   1;   2;   1;   1;   2;   1;   2;   2;   2;   1;   1; ... % ConfigurationIndex 80:99
                           1;   2;   2;   2;   2;   2;   2;   2;   1;   2;   1;   1;   1;   2;   2;   2;   2;   1;   2;   1; ... % ConfigurationIndex 100:119
                           2;   1;   2;   1;   2;   2;   2;   1;   1;   1;   2;   2;   2;   2;   2;   2;   2;   1;   2;   1; ... % ConfigurationIndex 120:139
                           1;   1;   2;   2;   2;   2;   2;   1;   2;   1;   2;   1;   2;   2;   2;   2;   2;   1;   1;   1; ... % ConfigurationIndex 140:159
                           2;   2;   2;   2;   2;   2;   2;   2;   2;   1;   1;   1;   2;   2;   2;   2;   2;   1;   2;   1; ... % ConfigurationIndex 160:179
                           2;   1;   1;   2;   1;   2;   2;   2;   1;   1;   1;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 180:199
                           2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 200:219
                           1;   2;   1;   2;   2;   2;   1;   1;   1;   2;   2;   2;   2;   2;   2;   2;   1;   2;   1;   2; ... % ConfigurationIndex 220:239
                           1;   2;   2;   2;   2;   2;   1;   1;   1;   2;   2;   2;   2;   2;   2;   2; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                         NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63322.NumTimeOccasions = [NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 0:19
                               NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 20:39
                               NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 40:59
                               NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 60:79
                               NaN; NaN; NaN; NaN; NaN; NaN; NaN;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; ... % ConfigurationIndex 80:99
                                 6;   6;   6;   6;   6;   6;   6;   6;   7;   7;   7;   7;   7;   7;   7;   7;   7;   3;   3;   3; ... % ConfigurationIndex 100:119
                                 3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3; ... % ConfigurationIndex 120:139
                                 3;   3;   3;   3;   3;   3;   3;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 140:159
                                 2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   7;   7;   7; ... % ConfigurationIndex 160:179
                                 7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   1;   1; ... % ConfigurationIndex 180:199
                                 1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   7; ... % ConfigurationIndex 200:219
                                 7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   2;   2;   2;   2; ... % ConfigurationIndex 220:239
                                 2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                               NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63322.PRACHDuration = [  0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 0:19
                              0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 20:39
                              0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 40:59
                              0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 60:79
                              0;   0;   0;   0;   0;   0;   0;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 80:99
                              2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   4;   4;   4; ... % ConfigurationIndex 100:119
                              4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4; ... % ConfigurationIndex 120:139
                              4;   4;   4;   4;   4;   4;   4;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; ... % ConfigurationIndex 140:159
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   2;   2;   2; ... % ConfigurationIndex 160:179
                              2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;  12;  12; ... % ConfigurationIndex 180:199
                             12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;   2; ... % ConfigurationIndex 200:219
                              2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   6;   6;   6;   6; ... % ConfigurationIndex 220:239
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                            NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    %% Define TS 38.211 Table 6.3.3.2-3
    % Each column represents the following:
    %   * Configuration index
    %   * Preamble format
    %   * x
    %   * y
    %   * Subframe number
    %   * Starting symbol
    %   * Number of PRACH slots within a subframe
    %   * Number of time-domain PRACH occasions within a PRACH slot (Nslot_t)
    %   * PRACH duration (N_dur)
    
    t63323.ConfigurationIndex = [  0;   1;   2;   3;   4;   5;   6;   7;   8;   9;  10;  11;  12;  13;  14;  15;  16;  17;  18;  19; ... % ConfigurationIndex 0:19
                                  20;  21;  22;  23;  24;  25;  26;  27;  28;  29;  30;  31;  32;  33;  34;  35;  36;  37;  38;  39; ... % ConfigurationIndex 20:39
                                  40;  41;  42;  43;  44;  45;  46;  47;  48;  49;  50;  51;  52;  53;  54;  55;  56;  57;  58;  59; ... % ConfigurationIndex 40:59
                                  60;  61;  62;  63;  64;  65;  66;  67;  68;  69;  70;  71;  72;  73;  74;  75;  76;  77;  78;  79; ... % ConfigurationIndex 60:79
                                  80;  81;  82;  83;  84;  85;  86;  87;  88;  89;  90;  91;  92;  93;  94;  95;  96;  97;  98;  99; ... % ConfigurationIndex 80:99
                                 100; 101; 102; 103; 104; 105; 106; 107; 108; 109; 110; 111; 112; 113; 114; 115; 116; 117; 118; 119; ... % ConfigurationIndex 100:119
                                 120; 121; 122; 123; 124; 125; 126; 127; 128; 129; 130; 131; 132; 133; 134; 135; 136; 137; 138; 139; ... % ConfigurationIndex 120:139
                                 140; 141; 142; 143; 144; 145; 146; 147; 148; 149; 150; 151; 152; 153; 154; 155; 156; 157; 158; 159; ... % ConfigurationIndex 140:159
                                 160; 161; 162; 163; 164; 165; 166; 167; 168; 169; 170; 171; 172; 173; 174; 175; 176; 177; 178; 179; ... % ConfigurationIndex 160:179
                                 180; 181; 182; 183; 184; 185; 186; 187; 188; 189; 190; 191; 192; 193; 194; 195; 196; 197; 198; 199; ... % ConfigurationIndex 180:199
                                 200; 201; 202; 203; 204; 205; 206; 207; 208; 209; 210; 211; 212; 213; 214; 215; 216; 217; 218; 219; ... % ConfigurationIndex 200:219
                                 220; 221; 222; 223; 224; 225; 226; 227; 228; 229; 230; 231; 232; 233; 234; 235; 236; 237; 238; 239; ... % ConfigurationIndex 220:239
                                 240; 241; 242; 243; 244; 245; 246; 247; 248; 249; 250; 251; 252; 253; 254; 255; 256; 257; 258; 259; ... % ConfigurationIndex 240:259
                                 260; 261; 262];                                                                                         % ConfigurationIndex 260:262

    t63323.Format = {    '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0'; ... % ConfigurationIndex 0:19
                         '0';     '0';     '0';     '0';     '0';     '0';     '0';     '0';     '1';     '1';     '1';     '1';     '1';     '1';     '2';     '2';     '2';     '2';     '2';     '2'; ... % ConfigurationIndex 20:39
                         '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3';     '3'; ... % ConfigurationIndex 40:59
                         '3';     '3';     '3';     '3';     '3';     '3';     '3';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1'; ... % ConfigurationIndex 60:79
                        'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2'; ... % ConfigurationIndex 80:99
                        'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3'; ... % ConfigurationIndex 100:119
                        'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1'; ... % ConfigurationIndex 120:139
                        'B1';    'B1';    'B1';    'B1';    'B1';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4'; ... % ConfigurationIndex 140:159
                        'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0'; ... % ConfigurationIndex 160:179
                        'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2'; ... % ConfigurationIndex 180:199
                        'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; ... % ConfigurationIndex 200:219
                     'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; ... % ConfigurationIndex 220:239
                     'A2/B2'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3';     '0';     '0';     '0';     '0'; ... % ConfigurationIndex 240:259
                         '0';     '0';     '0'};                                                                                                                                                             % ConfigurationIndex 260:262

    t63323.x = [ 16;   8;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 0:19
                  1;   1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   1;  16;   8;   4;   2;   2;   1; ... % ConfigurationIndex 20:39
                 16;   8;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 40:59
                  1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   2;   2;   2;   2;   2;   1;   1;   1; ... % ConfigurationIndex 60:79
                  1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   2;   2;   2;   2;   1;   1;   2;   1; ... % ConfigurationIndex 80:99
                  1;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   2;   2;   2;   2;   1; ... % ConfigurationIndex 100:119
                  1;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   4;   2;   2;   2;   2;   1;   1; ... % ConfigurationIndex 120:139
                  1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   2;   2;   2;   2;   2;   1;   1;   1;   1;   1; ... % ConfigurationIndex 140:159
                  1;   1;   1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   2;   2;   2;   2;   2;   1; ... % ConfigurationIndex 160:179
                  1;   1;   1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2;   2;   2;   2;   2;   2;   2;   8; ... % ConfigurationIndex 180:199
                  4;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   2;   2;   2;   2;   2;   2;   1;   1;   1; ... % ConfigurationIndex 200:219
                  1;   1;   1;   1;   1;   1;   2;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 220:239
                  1;   2;   2;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;   8;   4;   2; ... % ConfigurationIndex 240:259
                  2;   2;   2];                                                                                         % ConfigurationIndex 260:262

    t63323.y = {1; 1; 1; 0; 1; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; ... % ConfigurationIndex 0:19
                0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 0; 1; 0; 1; 1; 1; 0; 1; 0; ... % ConfigurationIndex 20:39
                1; 1; 1; 0; 1; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; ... % ConfigurationIndex 40:59
                0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 0; 0; 0; ... % ConfigurationIndex 60:79
                0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 1; 1; 1; 0; 0; 1; 0; ... % ConfigurationIndex 80:99
                0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 1; 1; 1; 0; ... % ConfigurationIndex 100:119
                0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 0; 0; ... % ConfigurationIndex 120:139
                0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 0; 0; 0; 0; 0; ... % ConfigurationIndex 140:159
                0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 0; ... % ConfigurationIndex 160:179
                0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; 1; ... % ConfigurationIndex 180:199
                1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 0; 0; 0; ... % ConfigurationIndex 200:219
                0; 0; 0; 0; 0; 0; 1; 1; 1; 1; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; ... % ConfigurationIndex 220:239
                0; 1; 1; 1; 1; 1; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 1; 1; 0; ... % ConfigurationIndex 240:259
                1; 0; 1};                                                       % ConfigurationIndex 260:262

    t63323.sfn = {   9;       9;       9;         9;         9;         4;         4;         9;         8;         7;         6;       5;     4;         3;             2;         [1 6];         [1 6];     [4 9];         [3 8];   [2 7]; ... % ConfigurationIndex 0:19
                 [8 9]; [4 8 9]; [3 4 9];   [7 8 9]; [3 4 8 9]; [6 7 8 9]; [1 4 6 9];   [1:2:9];         7;         7;         7;       7;     7;         7;             6;             6;             6;         6;             6;       6; ... % ConfigurationIndex 20:39
                     9;       9;       9;         9;         9;         4;         4;         9;         8;         7;         6;       5;     4;         3;             2;         [1 6];         [1 6];     [4 9];         [3 8];   [2 7]; ... % ConfigurationIndex 40:59
                 [8 9]; [4 8 9]; [3 4 9];   [7 8 9]; [3 4 8 9]; [1 4 6 9];   [1:2:9];         9;         9;         9;         9;   [4 9]; [7 9];     [7 9];         [8 9];         [4 9]; [2 3 4 7 8 9];         9;             9;       9; ... % ConfigurationIndex 60:79
                 [8 9];   [4 9];   [7 9]; [3 4 8 9]; [3 4 8 9];   [1:2:9];     [0:9];         9;         9;         9;     [7 9];   [8 9]; [7 9];     [4 9];         [4 9]; [2 3 4 7 8 9];             2;         7;             9;       9; ... % ConfigurationIndex 80:99
                     9;       9;   [2 7];     [8 9];     [4 9];     [7 9]; [3 4 8 9]; [3 4 8 9];   [1:2:9];     [0:9];         9;       9;     9;     [4 9];         [7 9];         [7 9];         [4 9];     [8 9]; [2 3 4 7 8 9];       2; ... % ConfigurationIndex 100:119
                     7;       9;       9;         9;         9;     [2 7];     [8 9];     [4 9];     [7 9]; [3 4 8 9]; [3 4 8 9]; [1:2:9]; [0:9];         9;             9;         [7 9];         [4 9];     [4 9];             9;       9; ... % ConfigurationIndex 120:139
                     9;   [8 9];   [4 9];     [7 9];   [1:2:9];         9;         9;         9;         9;         9;     [7 9];   [4 9]; [4 9];     [8 9]; [2 3 4 7 8 9];             1;             2;         4;             7;       9; ... % ConfigurationIndex 140:159
                     9;       9;   [4 9];     [7 9];     [8 9]; [3 4 8 9];   [1:2:9];     [0:9];     [0:9];         9;         9;       9;     9;     [8 9];         [7 9];         [7 9];         [4 9];     [4 9]; [2 3 4 7 8 9];       9; ... % ConfigurationIndex 160:179
                     9;       9;   [8 9];     [4 9];     [7 9]; [3 4 8 9]; [3 4 8 9];   [1:2:9];     [0:9];         9;         9;       9;     9;     [8 9];         [7 9];         [7 9];         [4 9];     [4 9]; [2 3 4 7 8 9];       9; ... % ConfigurationIndex 180:199
                     9;       9;       9;         9;     [8 9];     [4 9];     [7 9]; [3 4 8 9]; [3 4 8 9];   [1:2:9];     [0:9];       9; [4 9];     [7 9];         [7 9];         [4 9];         [8 9];         9;             9;       9; ... % ConfigurationIndex 200:219
                 [8 9];   [4 9];   [7 9]; [3 4 8 9];   [1:2:9];     [0:9];         9;     [4 9];     [7 9];     [4 9];     [8 9];       9;     9;         9;         [8 9];         [4 9];         [7 9]; [3 4 8 9];     [3 4 8 9]; [1:2:9]; ... % ConfigurationIndex 220:239
                 [0:9];       9;   [4 9];     [7 9];     [7 9];     [4 9];     [8 9];         9;         9;         9;     [8 9];   [4 9]; [7 9]; [3 4 8 9];       [1:2:9];         [0:9];             7;         7;             7;       7; ... % ConfigurationIndex 240:259
                     7;       2;       2};                                                                                                                                                                                                       % ConfigurationIndex 260:262

    t63323.StartingSymbol = [  0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   7;   0;   0;   0; ... % ConfigurationIndex 0:19
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   7;   7;   7; ... % ConfigurationIndex 20:39
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   7;   0;   0;   0; ... % ConfigurationIndex 40:59
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   7;   7;   0;   0;   0;   0;   0;   7;   0; ... % ConfigurationIndex 60:79
                               0;   0;   7;   0;   0;   0;   7;   0;   0;   0;   0;   0;   9;   9;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 80:99
                               9;   0;   0;   0;   0;   9;   0;   0;   0;   9;   0;   0;   0;   7;   7;   0;   0;   0;   0;   0; ... % ConfigurationIndex 100:119
                               0;   0;   0;   7;   0;   0;   0;   0;   7;   0;   0;   0;   7;   2;   2;   2;   8;   2;   2;   8; ... % ConfigurationIndex 120:139
                               2;   2;   2;   8;   2;   0;   0;   2;   0;   2;   2;   2;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 140:159
                               2;   0;   2;   2;   0;   2;   2;   0;   2;   2;   2;   2;   2;   2;   2;   8;   8;   2;   2;   2; ... % ConfigurationIndex 160:179
                               8;   2;   2;   2;   8;   2;   2;   2;   8;   2;   2;   2;   2;   2;   2;   8;   8;   2;   2;   8; ... % ConfigurationIndex 180:199
                               8;   2;   8;   2;   2;   2;   8;   2;   2;   2;   8;   2;   8;   8;   2;   2;   2;   2;   8;   2; ... % ConfigurationIndex 200:219
                               2;   2;   8;   2;   2;   8;   0;   6;   6;   0;   0;   0;   6;   0;   0;   0;   6;   0;   0;   0; ... % ConfigurationIndex 220:239
                               6;   0;   2;   0;   2;   0;   0;   0;   2;   0;   0;   0;   2;   0;   0;   2;   0;   0;   0;   0; ... % ConfigurationIndex 240:259
                               0;   0;   0];                                                                                         % ConfigurationIndex 260:262

    t63323.SlotsPerSF = [NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 0:19
                         NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 20:39
                         NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 40:59
                         NaN; NaN; NaN; NaN; NaN; NaN; NaN;   2;   2;   1;   1;   1;   1;   1;   2;   2;   1;   2;   1;   1; ... % ConfigurationIndex 60:79
                           2;   1;   1;   1;   2;   1;   1;   2;   2;   1;   1;   2;   1;   1;   2;   1;   1;   1;   1;   2; ... % ConfigurationIndex 80:99
                           1;   1;   1;   2;   1;   1;   1;   2;   1;   1;   2;   2;   1;   1;   1;   1;   2;   2;   1;   1; ... % ConfigurationIndex 100:119
                           1;   1;   2;   1;   1;   1;   2;   1;   1;   1;   2;   1;   1;   1;   1;   1;   1;   2;   2;   1; ... % ConfigurationIndex 120:139
                           1;   2;   1;   1;   1;   2;   2;   1;   1;   1;   1;   1;   2;   2;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 140:159
                           1;   2;   1;   1;   2;   1;   1;   2;   1;   2;   2;   1;   1;   2;   1;   1;   1;   2;   1;   2; ... % ConfigurationIndex 160:179
                           1;   1;   2;   1;   1;   1;   2;   1;   1;   2;   2;   1;   1;   2;   1;   1;   1;   2;   1;   2; ... % ConfigurationIndex 180:199
                           1;   2;   1;   1;   2;   1;   1;   1;   2;   1;   1;   1;   1;   1;   1;   2;   2;   2;   1;   1; ... % ConfigurationIndex 200:219
                           2;   1;   1;   2;   1;   1;   1;   1;   1;   2;   2;   2;   1;   1;   2;   1;   1;   1;   2;   1; ... % ConfigurationIndex 220:239
                           1;   1;   1;   1;   1;   2;   2;   2;   1;   1;   2;   1;   1;   2;   1;   1; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                           NaN; NaN; NaN];                                                                                       % ConfigurationIndex 260:262

    t63323.NumTimeOccasions = [NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 0:19
                               NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 20:39
                               NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 40:59
                               NaN; NaN; NaN; NaN; NaN; NaN; NaN;   6;   6;   6;   6;   3;   3;   6;   6;   6;   6;   6;   3;   6; ... % ConfigurationIndex 60:79
                                 6;   6;   3;   6;   6;   6;   3;   3;   3;   3;   3;   3;   1;   1;   3;   3;   3;   3;   3;   3; ... % ConfigurationIndex 80:99
                                 1;   3;   3;   3;   3;   1;   3;   3;   3;   1;   2;   2;   2;   1;   1;   2;   2;   2;   2;   2; ... % ConfigurationIndex 100:119
                                 2;   2;   2;   1;   2;   2;   2;   2;   1;   2;   2;   2;   1;   6;   6;   6;   3;   6;   6;   3; ... % ConfigurationIndex 120:139
                                 6;   6;   6;   3;   6;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 140:159
                                 1;   1;   1;   1;   1;   1;   1;   1;   1;   6;   6;   6;   6;   6;   6;   3;   3;   6;   6;   6; ... % ConfigurationIndex 160:179
                                 3;   6;   6;   6;   3;   6;   6;   6;   3;   2;   2;   2;   2;   2;   2;   1;   1;   2;   2;   1; ... % ConfigurationIndex 180:199
                                 1;   2;   1;   2;   2;   2;   1;   2;   2;   2;   1;   6;   3;   3;   6;   6;   6;   6;   3;   6; ... % ConfigurationIndex 200:219
                                 6;   6;   3;   6;   6;   3;   3;   2;   2;   3;   3;   3;   2;   3;   3;   3;   2;   3;   3;   3; ... % ConfigurationIndex 220:239
                                 2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                               NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63323.PRACHDuration = [  0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 0:19
                              0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 20:39
                              0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 40:59
                              0;   0;   0;   0;   0;   0;   0;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 60:79
                              2;   2;   2;   2;   2;   2;   2;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4; ... % ConfigurationIndex 80:99
                              4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; ... % ConfigurationIndex 100:119
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 120:139
                              2;   2;   2;   2;   2;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12; ... % ConfigurationIndex 140:159
                             12;  12;  12;  12;  12;  12;  12;  12;  12;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 160:179
                              2;   2;   2;   2;   2;   2;   2;   2;   2;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; ... % ConfigurationIndex 180:199
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 200:219
                              2;   2;   2;   2;   2;   2;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4; ... % ConfigurationIndex 220:239
                              4;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   0;   0;   0;   0; ... % ConfigurationIndex 240:259
                              0;   0;   0];                                                                                         % ConfigurationIndex 260:262

    %% Define TS 38.211 Table 6.3.3.2-4
    % Each column represents the following:
    %   * Configuration index
    %   * Preamble format
    %   * x
    %   * y
    %   * Slot number
    %   * Starting symbol
    %   * Number of PRACH slots within a 60 kHz slot
    %   * Number of time-domain PRACH occasions within a PRACH slot (Nslot_t)
    %   * PRACH duration (N_dur)
    
    t63324.ConfigurationIndex = [  0;   1;   2;   3;   4;   5;   6;   7;   8;   9;  10;  11;  12;  13;  14;  15;  16;  17;  18;  19; ... % ConfigurationIndex 0:19
                                  20;  21;  22;  23;  24;  25;  26;  27;  28;  29;  30;  31;  32;  33;  34;  35;  36;  37;  38;  39; ... % ConfigurationIndex 20:39
                                  40;  41;  42;  43;  44;  45;  46;  47;  48;  49;  50;  51;  52;  53;  54;  55;  56;  57;  58;  59; ... % ConfigurationIndex 40:59
                                  60;  61;  62;  63;  64;  65;  66;  67;  68;  69;  70;  71;  72;  73;  74;  75;  76;  77;  78;  79; ... % ConfigurationIndex 60:79
                                  80;  81;  82;  83;  84;  85;  86;  87;  88;  89;  90;  91;  92;  93;  94;  95;  96;  97;  98;  99; ... % ConfigurationIndex 80:99
                                 100; 101; 102; 103; 104; 105; 106; 107; 108; 109; 110; 111; 112; 113; 114; 115; 116; 117; 118; 119; ... % ConfigurationIndex 100:119
                                 120; 121; 122; 123; 124; 125; 126; 127; 128; 129; 130; 131; 132; 133; 134; 135; 136; 137; 138; 139; ... % ConfigurationIndex 120:139
                                 140; 141; 142; 143; 144; 145; 146; 147; 148; 149; 150; 151; 152; 153; 154; 155; 156; 157; 158; 159; ... % ConfigurationIndex 140:159
                                 160; 161; 162; 163; 164; 165; 166; 167; 168; 169; 170; 171; 172; 173; 174; 175; 176; 177; 178; 179; ... % ConfigurationIndex 160:179
                                 180; 181; 182; 183; 184; 185; 186; 187; 188; 189; 190; 191; 192; 193; 194; 195; 196; 197; 198; 199; ... % ConfigurationIndex 180:199
                                 200; 201; 202; 203; 204; 205; 206; 207; 208; 209; 210; 211; 212; 213; 214; 215; 216; 217; 218; 219; ... % ConfigurationIndex 200:219
                                 220; 221; 222; 223; 224; 225; 226; 227; 228; 229; 230; 231; 232; 233; 234; 235; 236; 237; 238; 239; ... % ConfigurationIndex 220:239
                                 240; 241; 242; 243; 244; 245; 246; 247; 248; 249; 250; 251; 252; 253; 254; 255; 256; 257; 258; 259; ... % ConfigurationIndex 240:259
                                 260; 261; 262];                                                                                         % ConfigurationIndex 260:262

    t63324.Format = {   'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1'; ... % ConfigurationIndex 0:19
                        'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A1';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2'; ... % ConfigurationIndex 20:39
                        'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A2';    'A3'; ... % ConfigurationIndex 40:59
                        'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3'; ... % ConfigurationIndex 60:79
                        'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'A3';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1'; ... % ConfigurationIndex 80:99
                        'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B1';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4'; ... % ConfigurationIndex 100:119
                        'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4';    'B4'; ... % ConfigurationIndex 120:139
                        'B4';    'B4';    'B4';    'B4';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0'; ... % ConfigurationIndex 140:159
                        'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C0';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2'; ... % ConfigurationIndex 160:179
                        'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2';    'C2'; ... % ConfigurationIndex 180:199
                        'C2';    'C2'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; 'A1/B1'; ... % ConfigurationIndex 200:219
                     'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A2/B2'; 'A3/B3'; 'A3/B3'; ... % ConfigurationIndex 220:239
                     'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3'; 'A3/B3';     '-';     '-';     '-';     '-'; ... % ConfigurationIndex 240:259
                         '-';     '-';    '-'};                                                                                                                                                              % ConfigurationIndex 260:262

    t63324.x = [ 16;  16;   8;   8;   8;   4;   4;   4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 0:19
                  1;   1;   1;   1;   1;   1;   1;   1;   1;  16;  16;   8;   8;   8;   4;   4;   4;   2;   2;   2; ... % ConfigurationIndex 20:39
                  2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16; ... % ConfigurationIndex 40:59
                 16;   8;   8;   8;   4;   4;   4;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 60:79
                  1;   1;   1;   1;   1;   1;   1;   1;   1;  16;   8;   8;   4;   2;   2;   1;   1;   1;   1;   1; ... % ConfigurationIndex 80:99
                  1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;  16;   8;   8;   8;   4;   4;   4; ... % ConfigurationIndex 100:119
                  2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 120:139
                  1;   1;   1;   1;  16;  16;   8;   8;   8;   4;   4;   4;   2;   2;   2;   2;   1;   1;   1;   1; ... % ConfigurationIndex 140:159
                  1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;  16;   8;   8;   8;   4;   4; ... % ConfigurationIndex 160:179
                  4;   2;   2;   2;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 180:199
                  1;   1;  16;  16;   8;   8;   4;   4;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 200:219
                 16;  16;   8;   8;   4;   4;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;  16;  16; ... % ConfigurationIndex 220:239
                  8;   8;   4;   4;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63324.y = {    1;     1; [1 2];     1;     1;     1;     1;     1;     1;     1;     1;     1;     0;     0;     0;     0;     0;     0;     0;     0; ... % ConfigurationIndex 0:19
                    0;     0;     0;     0;     0;     0;     0;     0;     0;     1;     1;     1;     1; [1 2];     1;     1;     1;     1;     1;     1; ... % ConfigurationIndex 20:39
                    1;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     1; ... % ConfigurationIndex 40:59
                    1;     1;     1; [1 2];     1;     1;     1;     1;     1;     1;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0; ... % ConfigurationIndex 60:79
                    0;     0;     0;     0;     0;     0;     0;     0;     0;     1;     1; [1 2];     1;     1;     1;     0;     0;     0;     0;     0; ... % ConfigurationIndex 80:99
                    0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0; [1 2]; [1 2]; [1 2]; [1 2]; [1 2];     1;     1; [1 2]; ... % ConfigurationIndex 100:119
                    1;     1;     1;     1;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0; ... % ConfigurationIndex 120:139
                    0;     0;     0;     0;     1;     1;     1;     1; [1 2];     1;     1;     1;     1;     1;     1;     1;     0;     0;     0;     0; ... % ConfigurationIndex 140:159
                    0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     1;     1;     1;     1; [1 2];     1;     1; ... % ConfigurationIndex 160:179
                    1;     1;     1;     1;     1;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0; ... % ConfigurationIndex 180:199
                    0;     0;     1;     1;     1;     1;     1;     1;     1;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0; ... % ConfigurationIndex 200:219
                    1;     1;     1;     1;     1;     1;     1;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     1;     1; ... % ConfigurationIndex 220:239
                    1;     1;     1;     1;     1;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;     0;   NaN;   NaN;   NaN;   NaN; ... % ConfigurationIndex 240:259
                  NaN;   NaN;   NaN};                                                                                                                           % ConfigurationIndex 260:262
 
    t63324.sfn = {       [4:5:39];         [3:4:39];     [9 19 29 39];         [4:5:39];                     [3:4:39];      [4:5:39];      [4:5:39];        [3:4:39];              [7 15 23 31 39];         [4:5:39];         [4:5:39];         [3:4:39];          [19 39];            [3 5 7];                [24 29 34 39];     [9 19 29 39]; [17 19 37 39];                 [9 19 29 39];         [4:5:39];         [4:5:39]; ... % ConfigurationIndex 0:19
                         [3:2:13]; [23 27 31 35 39];  [7 15 23 31 39]; [23 27 31 35 39]; [13 14 15 29 30 31 37 38 39];      [3:4:39];      [3:4:39];        [1:2:39];                       [0:39];         [4:5:39];         [3:4:39];         [4:5:39];         [3:4:39];       [9 19 29 39];                     [4:5:39];         [4:5:39];      [3:4:39];              [7 15 23 31 39];         [4:5:39];         [4:5:39]; ... % ConfigurationIndex 20:39
                         [3:4:39];          [19 39];          [3 5 7];    [24 29 34 39];                 [9 19 29 39]; [17 19 37 39];  [9 19 29 39]; [7 15 23 31 39];             [23 27 31 35 39]; [23 27 31 35 39];         [3:2:13];         [3:2:13];         [4:5:39];           [4:5:39]; [13 14 15 29 30 31 37 38 39];         [3:4:39];      [3:4:39];                     [1:2:39];           [0:39];         [4:5:39]; ... % ConfigurationIndex 40:59
                         [3:4:39];         [4:5:39];         [3:4:39];     [9 19 29 39];                     [4:5:39];      [4:5:39];      [3:4:39];        [4:5:39];                     [4:5:39];         [3:4:39];          [19 39];          [3 5 7];        [9 11 13];      [24 29 34 39];                 [9 19 29 39];    [17 19 37 39];  [9 19 29 39];              [7 15 23 31 39]; [23 27 31 35 39]; [23 27 31 35 39]; ... % ConfigurationIndex 60:79
                         [3:2:13];         [3:2:13];         [4:5:39];         [4:5:39]; [13 14 15 29 30 31 37 38 39];      [3:4:39];      [3:4:39];        [1:2:39];                       [0:39];         [4:5:39];         [4:5:39];     [9 19 29 39];         [4:5:39];           [4:5:39];                     [3:4:39];          [19 39];       [3 5 7];                [24 29 34 39];     [9 19 29 39];    [17 19 37 39]; ... % ConfigurationIndex 80:99
                     [9 19 29 39];  [7 15 23 31 39]; [23 27 31 35 39]; [23 27 31 35 39];                     [3:2:13];      [4:5:39];      [4:5:39];        [3:4:39]; [13 14 15 29 30 31 37 38 39];         [3:4:39];         [1:2:39];           [0:39];         [4:5:39];           [3:4:39];                     [4:5:39];         [3:4:39];  [9 19 29 39];                     [4:5:39];         [4:5:39];         [3:4:39]; ... % ConfigurationIndex 100:119
                  [7 15 23 31 39];         [4:5:39];         [4:5:39];         [3:4:39];                      [19 39]; [17 19 37 39]; [24 29 34 39];    [9 19 29 39];                 [9 19 29 39];  [7 15 23 31 39];  [7 15 23 31 39]; [23 27 31 35 39]; [23 27 31 35 39]; [9 11 13 15 17 19];                     [3:2:13];         [4:5:39];      [4:5:39]; [13 14 15 29 30 31 37 38 39];         [3:4:39];         [3:4:39]; ... % ConfigurationIndex 120:139
                         [3:2:25];         [3:2:25];         [1:2:39];           [0:39];                     [4:5:39];      [3:4:39];      [4:5:39];        [3:4:39];                 [9 19 29 39];         [4:5:39];         [4:5:39];         [3:4:39];  [7 15 23 31 39];           [4:5:39];                     [4:5:39];         [3:4:39];       [19 39];                      [3 5 7];    [24 29 34 39];     [9 19 29 39]; ... % ConfigurationIndex 140:159
                    [17 19 37 39];     [9 19 29 39]; [23 27 31 35 39];  [7 15 23 31 39];             [23 27 31 35 39];      [3:2:13];      [4:5:39];        [4:5:39]; [13 14 15 29 30 31 37 38 39];         [3:4:39];         [3:4:39];         [1:2:39];           [0:39];           [4:5:39];                     [3:4:39];         [4:5:39];      [3:4:39];                 [9 19 29 39];         [4:5:39];         [4:5:39]; ... % ConfigurationIndex 160:179
                         [3:4:39];  [7 15 23 31 39];         [4:5:39];         [4:5:39];                     [3:4:39];       [19 39];       [3 5 7];   [24 29 34 39];                 [9 19 29 39];    [17 19 37 39];     [9 19 29 39];  [7 15 23 31 39];         [3:2:13];   [23 27 31 35 39];             [23 27 31 35 39];         [4:5:39];      [4:5:39]; [13 14 15 29 30 31 37 38 39];         [3:4:39];         [3:4:39]; ... % ConfigurationIndex 180:199
                         [1:2:39];           [0:39];         [4:5:39];         [3:4:39];                     [4:5:39];      [3:4:39];      [4:5:39];        [3:4:39];                     [4:5:39];          [19 39];     [9 19 29 39];    [17 19 37 39];     [9 19 29 39];   [23 27 31 35 39];              [7 15 23 31 39]; [23 27 31 35 39];      [4:5:39];                     [4:5:39];         [3:4:39];         [1:2:39]; ... % ConfigurationIndex 200:219
                         [4:5:39];         [3:4:39];         [4:5:39];         [3:4:39];                     [4:5:39];      [3:4:39];      [4:5:39];         [19 39];                 [9 19 29 39];    [17 19 37 39];     [9 19 29 39]; [23 27 31 35 39];  [7 15 23 31 39];   [23 27 31 35 39];                     [4:5:39];         [4:5:39];      [3:4:39];                     [1:2:39];         [4:5:39];         [3:4:39]; ... % ConfigurationIndex 220:239
                         [4:5:39];         [3:4:39];         [4:5:39];         [3:4:39];                     [4:5:39];       [19 39];  [9 19 29 39];   [17 19 37 39];                 [9 19 29 39];  [7 15 23 31 39]; [23 27 31 35 39]; [23 27 31 35 39];         [4:5:39];           [4:5:39];                     [3:4:39];         [1:2:39];           NaN;                          NaN;              NaN;              NaN; ... % ConfigurationIndex 240:259
                              NaN;              NaN;              NaN};                                                                                                                                                                                                                                                                                                                                                              % ConfigurationIndex 260:262

    t63324.StartingSymbol = [  0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   7;   0;   7;   7;   0;   0;   0;   7; ... % ConfigurationIndex 0:19
                               7;   7;   0;   0;   7;   7;   0;   0;   7;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 20:39
                               0;   5;   0;   5;   5;   0;   0;   0;   5;   0;   5;   0;   5;   0;   5;   5;   0;   0;   5;   0; ... % ConfigurationIndex 40:59
                               0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   7;   0;   2;   7;   7;   0;   0;   0;   7;   0; ... % ConfigurationIndex 60:79
                               0;   7;   0;   7;   7;   7;   0;   0;   7;   2;   2;   2;   2;   2;   2;   8;   2;   8;   8;   2; ... % ConfigurationIndex 80:99
                               2;   2;   8;   2;   8;   8;   2;   8;   8;   2;   2;   8;   0;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 100:119
                               2;   0;   0;   0;   2;   0;   2;   2;   0;   0;   0;   0;   2;   0;   2;   0;   2;   2;   0;   2; ... % ConfigurationIndex 120:139
                               2;   0;   0;   2;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   0;   8;   0;   8;   8; ... % ConfigurationIndex 140:159
                               0;   0;   8;   0;   0;   8;   8;   0;   8;   8;   0;   0;   8;   0;   0;   0;   0;   0;   0;   0; ... % ConfigurationIndex 160:179
                               0;   2;   0;   0;   0;   2;   0;   7;   7;   0;   2;   2;   7;   7;   0;   7;   2;   7;   7;   0; ... % ConfigurationIndex 180:199
                               0;   7;   2;   2;   2;   2;   2;   2;   2;   8;   8;   2;   2;   8;   2;   2;   8;   2;   2;   2; ... % ConfigurationIndex 200:219
                               2;   2;   2;   2;   2;   2;   2;   6;   6;   2;   2;   6;   2;   2;   6;   2;   2;   2;   2;   2; ... % ConfigurationIndex 220:239
                               2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                             NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63324.SlotsPerSF = [  2;   1;   2;   2;   1;   1;   2;   1;   2;   1;   2;   1;   1;   1;   1;   2;   1;   2;   1;   1; ... % ConfigurationIndex 0:19
                           1;   1;   1;   1;   2;   1;   1;   1;   1;   2;   1;   2;   1;   2;   1;   2;   1;   2;   1;   2; ... % ConfigurationIndex 20:39
                           1;   1;   1;   1;   2;   1;   2;   1;   1;   1;   1;   1;   1;   1;   2;   1;   1;   1;   1;   2; ... % ConfigurationIndex 40:59
                           1;   2;   1;   2;   1;   2;   1;   1;   2;   1;   1;   1;   1;   1;   2;   1;   2;   1;   1;   1; ... % ConfigurationIndex 60:79
                           1;   1;   1;   1;   2;   1;   1;   1;   1;   2;   2;   2;   2;   2;   1;   1;   1;   1;   2;   1; ... % ConfigurationIndex 80:99
                           2;   1;   1;   1;   1;   1;   1;   1;   2;   1;   1;   1;   2;   1;   2;   1;   2;   1;   2;   1; ... % ConfigurationIndex 100:119
                           2;   1;   2;   1;   2;   1;   1;   2;   2;   1;   2;   1;   2;   1;   1;   1;   2;   2;   1;   1; ... % ConfigurationIndex 120:139
                           1;   2;   1;   1;   2;   1;   1;   1;   2;   1;   2;   1;   2;   1;   2;   1;   1;   1;   1;   2; ... % ConfigurationIndex 140:159
                           1;   2;   1;   1;   1;   1;   1;   1;   2;   1;   1;   1;   1;   2;   1;   2;   1;   2;   1;   2; ... % ConfigurationIndex 160:179
                           1;   2;   1;   2;   1;   1;   1;   1;   2;   1;   2;   1;   1;   2;   1;   2;   1;   2;   1;   1; ... % ConfigurationIndex 180:199
                           1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   2;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 200:219
                           1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   2;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 220:239
                           1;   1;   1;   1;   1;   1;   1;   1;   2;   1;   1;   2;   1;   2;   1;   1; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                         NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63324.NumTimeOccasions = [  6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   3;   6;   3;   3;   6;   6;   6;   3; ... % ConfigurationIndex 0:19
                                 3;   3;   6;   6;   3;   3;   6;   6;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3;   3; ... % ConfigurationIndex 20:39
                                 3;   2;   3;   2;   2;   3;   3;   3;   2;   3;   2;   3;   2;   3;   2;   2;   3;   3;   2;   2; ... % ConfigurationIndex 40:59
                                 2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   1;   2;   2;   1;   1;   2;   2;   2;   1;   2; ... % ConfigurationIndex 60:79
                                 2;   1;   2;   1;   1;   1;   2;   2;   1;   6;   6;   6;   6;   6;   6;   3;   6;   3;   3;   6; ... % ConfigurationIndex 80:99
                                 6;   6;   3;   6;   3;   3;   6;   3;   3;   6;   6;   3;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 100:119
                                 1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1;   1; ... % ConfigurationIndex 120:139
                                 1;   1;   1;   1;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   7;   3;   7;   3;   3; ... % ConfigurationIndex 140:159
                                 7;   7;   3;   7;   7;   3;   3;   7;   3;   3;   7;   7;   3;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 160:179
                                 2;   2;   2;   2;   2;   2;   2;   1;   1;   2;   2;   2;   1;   1;   2;   1;   2;   1;   1;   2; ... % ConfigurationIndex 180:199
                                 2;   1;   6;   6;   6;   6;   6;   6;   6;   3;   3;   6;   6;   3;   6;   6;   3;   6;   6;   6; ... % ConfigurationIndex 200:219
                                 3;   3;   3;   3;   3;   3;   3;   2;   2;   3;   3;   2;   3;   3;   2;   3;   3;   3;   2;   2; ... % ConfigurationIndex 220:239
                                 2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                               NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

    t63324.PRACHDuration = [  2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 0:19
                              2;   2;   2;   2;   2;   2;   2;   2;   2;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4; ... % ConfigurationIndex 20:39
                              4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   6; ... % ConfigurationIndex 40:59
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; ... % ConfigurationIndex 60:79
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 80:99
                              2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;  12;  12;  12;  12;  12;  12;  12;  12; ... % ConfigurationIndex 100:119
                             12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12;  12; ... % ConfigurationIndex 120:139
                             12;  12;  12;  12;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 140:159
                              2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   6;   6;   6;   6;   6;   6;   6; ... % ConfigurationIndex 160:179
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; ... % ConfigurationIndex 180:199
                              6;   6;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2;   2; ... % ConfigurationIndex 200:219
                              4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   4;   6;   6; ... % ConfigurationIndex 220:239
                              6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6;   6; NaN; NaN; NaN; NaN; ... % ConfigurationIndex 240:259
                            NaN; NaN; NaN];                                                                                         % ConfigurationIndex 260:262

end

%% Output
switch tableIdx
    
    case 2 % TS 38.211 Table 6.3.3.2-2
        table = t63322;
    case 3 % TS 38.211 Table 6.3.3.2-3
        table = t63323;
    case 4 % TS 38.211 Table 6.3.3.2-4
        table = t63324;
    otherwise
        coder.internal.error('nr5g:nrPRACH:InvalidConfigurationTables');
end

ConfigurationIndex = table.ConfigurationIndex;
Format = table.Format;
x = table.x;
y = table.y;
sfn = table.sfn;
StartingSymbol = table.StartingSymbol;
SlotsPerSF = table.SlotsPerSF;
NumTimeOccasions = table.NumTimeOccasions;
PRACHDuration = table.PRACHDuration;

end