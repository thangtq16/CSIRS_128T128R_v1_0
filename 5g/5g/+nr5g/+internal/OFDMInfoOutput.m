function info = OFDMInfoOutput(internalinfo)
%OFDMInfoOutput OFDM information output
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   INFO = OFDMInfoOutput(INTERNALINFO) produces OFDM dimensional
%   information output structure INFO from internal structure INTERNALINFO.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    info.Nfft = internalinfo.Nfft;
    info.SampleRate = internalinfo.SampleRate;
    info.CyclicPrefixLengths = internalinfo.CyclicPrefixLengths;
    info.SymbolLengths = internalinfo.CyclicPrefixLengths + internalinfo.Nfft;
    info.Windowing = internalinfo.Windowing;
    info.SymbolPhases = internalinfo.SymbolPhases;
    info.SymbolsPerSlot = internalinfo.SymbolsPerSlot;
    info.SlotsPerSubframe = internalinfo.SlotsPerSubframe;
    info.SlotsPerFrame = internalinfo.SlotsPerSubframe * 10;
    
end
