function symOrder = generateSymbolOrderVector(bps)
%generateSymbolOrderVector generate symbol order vector for symbol
%modulation and demodulation
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   SYMORDER = nr5g.internal.generateSymbolOrderVector(bps) generates
%   the symbol order vector based on the bits per symbol specified.

%   Copyright 2018-2023 The MathWorks, Inc.

%References:
%   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical channels and
%   modulation.

%#codegen

    switch (bps)
    
        case 2 % QPSK
            symOrder = [2 3 0 1];
        case 4 % 16QAM
            symOrder = [11 10 14 15 9 8 12 13 1 0 4 5 3 2 6 7];
        case 6 % 64QAM
            symOrder = [47 46 42 43 59 58 62 63 45 44 40 41 57 56 60 ...
                61 37 36 32 33 49 48 52 53 39 38 34 35 51 50 ...
                54 55 7 6 2 3 19 18 22 23 5 4 0 1 17 16 20 ...
                21 13 12 8 9 25 24 28 29 15 14 10 11 27 26 30 31];
        case 8 % 256QAM
            symOrder = [191 190 186 187 171 170 174 175 239 238 234 ...
                235 251 250 254 255 189 188 184 185 169 168 ...
                172 173 237 236 232 233 249 248 252 253 181 ...
                180 176 177 161 160 164 165 229 228 224 225 ...
                241 240 244 245 183 182 178 179 163 162 166 ...
                167 231 230 226 227 243 242 246 247 151 150 ...
                146 147 131 130 134 135 199 198 194 195 211 ...
                210 214 215 149 148 144 145 129 128 132 133 ...
                197 196 192 193 209 208 212 213 157 156 152 ...
                153 137 136 140 141 205 204 200 201 217 216 ...
                220 221 159 158 154 155 139 138 142 143 207 ...
                206 202 203 219 218 222 223 31 30 26 27 11 ...
                10 14 15 79 78 74 75 91 90 94 95 29 28 24 25 ...
                9 8 12 13 77 76 72 73 89 88 92 93 21 20 16 17 ...
                1 0 4 5 69 68 64 65 81 80 84 85 23 22 18 19 3 ...
                2 6 7 71 70 66 67 83 82 86 87 55 54 50 51 35 ...
                34 38 39 103 102 98 99 115 114 118 119 53 52 ...
                48 49 33 32 36 37 101 100 96 97 113 112 116 ...
                117 61 60 56 57 41 40 44 45 109 108 104 105 ...
                121 120 124 125 63 62 58 59 43 42 46 47 111 ...
                110 106 107 123 122 126 127];
        case 10 % 1024QAM
            symOrder = symbolMapping1024QAM;
        otherwise % BPSK or pi/2-BPSK
            symOrder = [0 1];
    end

end

% Compute the symbol mapping indices of a 1024QAM constellation as defined
% in TS 38.211 Section 5.1.7 and required by comm.internal.qam.modulate
function symbolMap = symbolMapping1024QAM
    
    % 10-tuplet binary sequence
    b = dec2bin(0:2^10-1) == '1';

    % 10-tuplet mapping to complex-valued modulation symbols as defined in 
    % TS 38.211 Section 5.1.7 (Release 17)
    a = 1 - 2*b;
    symI = a(:,1).*( 16 - a(:,3).*( 8 - a(:,5).*(4 - a(:,7).*( 2 -  a(:,9) ) ) ) );
    symQ = a(:,2).*( 16 - a(:,4).*( 8 - a(:,6).*(4 - a(:,8).*( 2 - a(:,10) ) ) ) );

    % Normalize and shift constellation to match integer numbers and use them as indices 
    symINorm = symI * 31/62 + 16.5;
    symQNorm = symQ * 31/62 - 16.5;

    % Sort constellation vectorizing from top left corner
    [~, symbolMap] = sort( (symINorm - 1) * 32 - symQNorm );
    symbolMap = symbolMap - 1; % Range 0 to 1023
end
