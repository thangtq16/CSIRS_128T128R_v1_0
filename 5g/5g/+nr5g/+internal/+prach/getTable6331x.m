function out = getTable6331x(tableIdx)
%getTable6331x Get TS 38.211 Tables 6.3.3.1-1, 6.3.3.1-2, 6.3.3.1-5,
% 6.3.3.1-6, and 6.3.3.1-7.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    OUT = nr5g.internal.prach.getTable6331x(TABLEIDX) returns TS 38.211
%    Tables 6.3.3.1-x, in which 'x' is defined by the input parameter
%    'TABLEIDX'.

%    Copyright 2019-2022 The MathWorks, Inc.

%#codegen

persistent table63311;
persistent table63312;
persistent table63315;
persistent table63316;
persistent table63317;

if isempty(table63311)
    
    % Define TS 38.211 Table 6.3.3.1-1
    % Each column represents the following:
    %   Column 1: Format
    %   Column 2: LRA
    %   Column 3: SubcarrierSpacing (in kHz)
    %   Column 4: Nu (in units of k)
    %   Column 5: NCP (in units of k)
    %   Column 6: Support for restricted set
    t63311 = {
              '0'  839  1.25    24576   3168  'Type A, Type B';
              '1'  839  1.25  2*24576  21024  'Type A, Type B';
              '2'  839  1.25  4*24576   4688  'Type A, Type B';
              '3'  839     5   4*6144   3168  'Type A, Type B';
             };
    table63311 = cell2table(t63311,'VariableNames',{'Format','LRA','SubcarrierSpacing','N_u','N_CP','RestrictedSets'});
    table63311.Properties.Description = 'TS 38.211 Table 6.3.3.1-1: PRACH Preamble formats for LRA = 839';
    
    % Define TS 38.211 Table 6.3.3.1-2
    % Each column represents the following:
    %   Column 1: Format
    %   Column 2: LRA (for all values of mu)
    %   Column 3: LRA (for mu = {0,3})
    %   Column 4: LRA (for mu = {1,3,5})
    %   Column 5: SubcarrierSpacing (in units of 2^mu kHz)
    %   Column 6: Nu (in units of k/2^mu)
    %   Column 7: NCP (in units of k/2^mu)
    %   Column 8: Support for restricted set
    t63312 = {
              'A1'  139  1151 571 15   2*2048   288  '-';
              'A2'  139  1151 571 15   4*2048   576  '-';
              'A3'  139  1151 571 15   6*2048   864  '-';
              'B1'  139  1151 571 15   2*2048   216  '-';
              'B2'  139  1151 571 15   4*2048   360  '-';
              'B3'  139  1151 571 15   6*2048   504  '-';
              'B4'  139  1151 571 15  12*2048   936  '-';
              'C0'  139  1151 571 15     2048  1240  '-';
              'C2'  139  1151 571 15   4*2048  2048  '-';
             };
    table63312 = cell2table(t63312,'VariableNames',{'Format','LRA','LRA_03','LRA_135','SubcarrierSpacing','N_u','N_CP','RestrictedSets'});
    table63312.Properties.Description = 'TS 38.211 Table 6.3.3.1-2: PRACH Preamble formats for LRA = {139,571,1151}';
    
    % Define TS 38.211 Table 6.3.3.1-5
    % Each column represents the following:
    %   Column 1: zeroCorrelationZoneConfig
    %   Column 2: NCS value for unrestricted set
    %   Column 3: NCS value for restricted set type A
    %   Column 4: NCS value for restricted set type B
    t63315 = {
              0     0    15    15
              1    13    18    18
              2    15    22    22
              3    18    26    26
              4    22    32    32
              5    26    38    38
              6    32    46    46
              7    38    55    55
              8    46    68    68
              9    59    82    82
             10    76   100   100
             11    93   128   118
             12   119   158   137
             13   167   202   NaN
             14   279   237   NaN
             15   419   NaN   NaN
            };
    table63315 = cell2table(t63315,'VariableNames',{'ZeroCorrelationZone','UnrestrictedSet','RestrictedSetTypeA','RestrictedSetTypeB'});
    table63315.Properties.Description = 'TS 38.211 Table 6.3.3.1-5: NCS for preamble formats 0,1,2';
    
    % Define TS 38.211 Table 6.3.3.1-6
    % Each column represents the following:
    %   Column 1: zeroCorrelationZoneConfig
    %   Column 2: NCS value for unrestricted set
    %   Column 3: NCS value for restricted set type A
    %   Column 4: NCS value for restricted set type B
    t63316 = {
              0     0    36    36
              1    13    57    57
              2    26    72    60
              3    33    81    63
              4    38    89    65
              5    41    94    68
              6    49   103    71
              7    55   112    77
              8    64   121    81
              9    76   132    85
             10    93   137    97
             11   119   152   109
             12   139   173   122
             13   209   195   137
             14   279   216   NaN
             15   419   237   NaN
            };
    table63316 = cell2table(t63316,'VariableNames',{'ZeroCorrelationZone','UnrestrictedSet','RestrictedSetTypeA','RestrictedSetTypeB'});
    table63316.Properties.Description = 'TS 38.211 Table 6.3.3.1-6: NCS for preamble format 3';
    
    % Define TS 38.211 Table 6.3.3.1-7
    % Each column represents the following:
    %   Column 1: zeroCorrelationZoneConfig
    %   Column 2: NCS value for unrestricted set and LRA = 139
    %   Column 3: NCS value for unrestricted set and LRA = 571
    %   Column 4: NCS value for unrestricted set and LRA = 1151
    t63317 = {
              0     0      0      0
              1     2      8     17
              2     4     10     21
              3     6     12     25
              4     8     15     30
              5    10     17     35
              6    12     21     44
              7    13     25     52
              8    15     31     63
              9    17     40     82
             10    19     51    104
             11    23     63    127
             12    27     81    164
             13    34    114    230
             14    46    190    383
             15    69    285    575
            };
    table63317 = cell2table(t63317,'VariableNames',{'ZeroCorrelationZone','LRA_139','LRA_571','LRA_1151'});
    table63317.Properties.Description = 'TS 38.211 Table 6.3.3.1-7: NCS for short preambles';
end

switch tableIdx
    
    case 1 % TS 38.211 Table 6.3.3.1-1
        out = table63311;
    case 2 % TS 38.211 Table 6.3.3.1-2
        out = table63312;
    case 5 % TS 38.211 Table 6.3.3.1-5
        out = table63315;
    case 6 % TS 38.211 Table 6.3.3.1-6
        out = table63316;
    case 7 % TS 38.211 Table 6.3.3.1-7
        out = table63317;
    otherwise
        coder.internal.error('nr5g:nrPRACH:InvalidTable6331x');
end

end