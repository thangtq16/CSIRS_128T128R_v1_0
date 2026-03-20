function d = gridSize(NRB,SCS,prach,P)
%gridSize Size of PRACH slot resource grid
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   D = gridSize(NRB,SCS,PRACH,P) returns a 3-element row vector of
%   dimension lengths for the PRACH slot resource grid, given number of
%   carrier resource blocks NRB (1...275), carrier subcarrier spacing SCS
%   (15,30,60,120,480,960), PRACH configuration object PRACH and number of
%   antennas P.

%   Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    % TS 38.211 Section 5.3.2
    % Number of subcarriers
    K = double(SCS) / prach.SubcarrierSpacing;

    % Number of OFDM symbols in a PRACH slot
    L = nr5g.internal.prach.gridSymbolSize(prach);

    % PRACH grid size
    d = [floor(K*double(NRB)*12) L double(P)];

end
