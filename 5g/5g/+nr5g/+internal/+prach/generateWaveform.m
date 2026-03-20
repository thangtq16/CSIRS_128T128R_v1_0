function [waveform,info] = generateWaveform(carrier,prach,varargin)
%generateWaveform Generate a single-slot PRACH waveform.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   [WAVEFORM,INFO] = generateWaveform(CARRIER,PRACH) generates a PRACH
%   waveform for the current slot specified by PRACH.NPRACHSlot. The
%   waveform is characterized by no windowing. The second output INFO is
%   the information structure returned in nrPRACHOFDMInfo and
%   nrPRACHOFDMModulate.
%   
%   [WAVEFORM,INFO] = generateWaveform(CARRIER,PRACH,INFO) uses the
%   information in the INFO structure to generate the PRACH waveform. INFO
%   is a structure with these (optional) fields:
%
%   symInfo  - The information output of nr5g.internal.prach.getSymbolsInfo,
%              or the second output of nrPRACH.
%   ofdmInfo - The PRACH OFDM information given by nr5g.internal.prach.OFDMInfo.
%              Note that this is not the same INFO structure returned by
%              nrPRACHOFDMInfo.

%  Copyright 2022 The MathWorks, Inc.

%#codegen
    
    % Get optional values
    if nargin>2 && isfield(varargin{1},'symInfo')
        symInfo = varargin{1}.symInfo;
    else
        symInfo = nr5g.internal.prach.getSymbolsInfo(prach);
    end
    if nargin>2 && isfield(varargin{1},'ofdmInfo')
        ofdmInfo = varargin{1}.ofdmInfo;
    else
        ofdmOpts = struct('Windowing',0);
        ofdmInfo = nr5g.internal.prach.OFDMInfo(carrier,prach,ofdmOpts);
    end

    % Generate an empty PRACH resource grid
    gridSize = nr5g.internal.prach.gridSize(carrier.NSizeGrid,carrier.SubcarrierSpacing,prach,1);
    prachGrid = complex(zeros(gridSize));
    
    % Create the PRACH symbols
    symOpts = struct('OutputDataType','double'); % Set the default options
    prachSymbols = nr5g.internal.prach.getSymbols(prach,symInfo,symOpts);

    % If this slot has an active PRACH preamble, generate the waveform
    if ~isempty(prachSymbols)
        % Create the PRACH indices
        indOpts = struct('IndexStyle','index','IndexBase','1based'); % Set the default options
        prachIndices = nr5g.internal.prach.getIndices(carrier,prach,indOpts);

        % Map the PRACH symbols into the grid
        prachGrid(prachIndices) = prachSymbols;

        % Generate the PRACH waveform for this slot
        waveform = nr5g.internal.prach.OFDMModulate(carrier,prach,prachGrid,ofdmInfo);
    else
        % PRACH is not active in this slot. Generate a waveform with no
        % information
        N = sum(ofdmInfo.CyclicPrefixLengths + ofdmInfo.Nfft + ofdmInfo.GuardLengths) + ofdmInfo.OffsetLength;
        waveform = complex(zeros(N,gridSize(3)));
    end

    % Create OFDM information output structure
    info = nr5g.internal.prach.OFDMInfoOutput(ofdmInfo);

end
