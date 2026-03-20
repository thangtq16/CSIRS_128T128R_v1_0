classdef TBSDetermination
    %TBSDetermination Static functions for calculating transport block sizes
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    
    %   Copyright 2024 The MathWorks, Inc.

    %#codegen

    methods (Static)

        % Usage context in toolbox:
        % 
        % nrTBS(wavephy)  - tcr,xOh,tbScaling (for PDSCH case)
        % 
        % nrTBS(phy,  tcr)
        % nrTBS(phy,  tcr,xOh)
        % nrTBS(phy,  tcr,xOh,tbScaling)
        % 
        % nrTBS(modulation,nlayers,nPRB,NREPerPRB,  tcr)
        % nrTBS(modulation,nlayers,nPRB,NREPerPRB,  tcr,xOh)
        % nrTBS(modulation,nlayers,nPRB,NREPerPRB,  tcr,xOh,tbScaling)
        
        % Parsing and validation of tcr and {xOh,tbScaling} in varargin (with tbScaling being optionally read)
        function [tcr,xOh,tbScaling] = transportChParamsLinear(tcr, readtbscaling, varargin)

            fcnName = 'nrTBS';
            % tcr
            validateattributes(tcr,{'double','single'},...
                {'nonempty','real','>',0,'<',1},fcnName,'TCR');
            validateattributes(numel(tcr),{'double'},{'scalar','<=',2},fcnName,'length of TCR');
            
            if nargin < 3 
                % nrTBS(modulation,nlayers,nPRB,NREPerPRB,  tcr)
                xOh = 0;       % Default value
                tbScaling = 1; % Default value
            else
                % nrTBS(modulation,nlayers,nPRB,NREPerPRB,  tcr,xOh)
                xOh = varargin{1};
                validateattributes(xOh,{'numeric'},...
                    {'scalar','integer','nonnegative'},fcnName,'XOH');        
                tbScaling = 1; % Default value
                if readtbscaling && nargin > 3
                    tbsclin = varargin{2};
                    validateattributes(tbsclin,...
                        {'double','single'},{'nonempty','real','>',0,'<=',1},fcnName,'TBSCALING');
                    validateattributes(numel(tbsclin),{'double'},{'scalar','<=',2},fcnName,'length of TBSCALING');
                    tbScaling = double(tbsclin);
                end
            end
        end

        % Full linear physical channel parameters
        function Qm = physicalChParamsLinear(modulation,nlayers,nPRB,NREPerPRB)

            % --------------------------
            % Physical channel related parameters
            % --------------------------
        
            % Validate inputs
            fcnName = 'nrTBS';
            % modulation
            modlist = {'pi/2-BPSK', 'QPSK', '16QAM', '64QAM', '256QAM', '1024QAM'};
            validateattributes(modulation,{'cell','char','string'},...
                {'nonempty'},fcnName,'MODULATION');
            numModSchemes = numel(modulation);
            if ischar(modulation) || (isstring(modulation) && numModSchemes == 1)
                % Single character array or string scalar
                modscheme = validatestring(modulation,modlist,fcnName,'MODULATION');
                Qm = nr5g.internal.getQm(modscheme);
            else
                % Cell array or string array case
    
                % First convert any string array to a cell array for uniformity
                tempVal = convertStringsToChars(modulation);
                validateattributes(numModSchemes,{'double'},{'scalar','<=',2},fcnName,'length of MODULATION');
        
                % Validate each modulation name and get the associated Qm
                % value in the process
                Qm = zeros(numModSchemes,1);
                for idx = 1:numModSchemes
                    modscheme = validatestring(tempVal{idx},modlist,fcnName,'MODULATION');
                    Qm(idx) = nr5g.internal.getQm(modscheme);
                end
            end
        
            % nlayers
            validateattributes(nlayers,{'numeric'},...
                {'scalar','integer','positive','<=',8},fcnName,'NLAYERS');
        
            % nPRB
            validateattributes(nPRB,{'numeric'},...
                {'scalar','integer','nonnegative'},fcnName,'NPRB');
        
            % NREPerPRB
            validateattributes(NREPerPRB,{'numeric'},...
                {'scalar','integer','nonnegative'},fcnName,'NREPERPRB');
        
        end

        % Object based physical channel parameters
        function tbs = getTBSEntry(channel,tcr,xOh,tbScaling,nprboverride)
            % We already know that channel will be good for the PHY parameters
            
            % Extract PhyCH parts out of channel object
            nlayers = channel.NumLayers;
            [nPRB,NREPerPRB] = getNPRBAndNRE(channel);
            Qm = getQm(channel.Modulation);
            
            if nargin > 4 && nprboverride >= 0
                nPRB = nprboverride;
            end

           % The full linear 'internal' function below
           tbs = nr5g.internal.TBSDetermination.calculateTBS(nlayers,nPRB,NREPerPRB,Qm,tcr,xOh,tbScaling);
        end

        function tbs = calculateTBS(nlayers,nPRB,NREPerPRB,Qm,tcr,xOh,tbScaling)
    
            % --------------------------
            % Expand any 'scalars' against the number of codewords/layers
            % --------------------------
        
            % Get the number of codewords
            nlayersD = double(nlayers);
            ncw = ceil(nlayersD/4);
        
            % Number of layers for each codeword
            nLayers = floor((nlayersD + (0:ncw-1)')/ncw);
        
            % Apply scalar expansion for tempQm, tcr, and tbScaling
            Qm = applyScalarExpansion(Qm,ncw);
        
            R = applyScalarExpansion(double(tcr(:)),ncw);
            S = applyScalarExpansion(double(tbScaling(:)),ncw);
        
            % -------------------------------------------
        
            % Calculate the number of REs available for the data transmission in
            % the physical shared channel, within one PRB for one slot
            NREPrime = double(NREPerPRB) - double(xOh);
        
            % Output zero(s) for TBS, when no REs are available for the data
            % transmission
            if ~NREPerPRB || ~nPRB || NREPrime <= 0
                tbs = zeros(1,ncw);
                return;
            end
        
            % Calculate the total number of REs allocated for the physical shared channel
            NRE = min(156,NREPrime)*double(nPRB);
        
            % Calculate the intermediate number of information bits (N_info)
            Ninfo = S.*NRE.*R.*Qm.*nLayers;
        
            % Calculate the transport block size(s) associated with a data
            % transmission in the physical shared channel with one or two codewords
            tbs = getTBS(Ninfo,R);
    
        end

    end

end

% File local functions

% Calculate the Qm values associated with a 1 or 2 modulation names, 
% specifically for a char array or cell array of char arrays
function Qm = getQm(modulation) 
    if ischar(modulation)
        Qm = nr5g.internal.getQm(modulation);
    else
        % Cell array of 'strings' case
        numModSchemes = numel(modulation);
        Qm = zeros(numModSchemes,1);
        for idx = 1:numModSchemes
            Qm(idx) = nr5g.internal.getQm(modulation{idx});
        end
    end
end

function out = applyScalarExpansion(in,ncw)
% applyScalarExpansion Applies scalar expansion
%   OUT = applyScalarExpansion(IN,NCW) returns the output OUT of length
%   equals to the number of codewords NCW by repeating the input IN for NCW
%   times, if IN is a scalar.

    if isscalar(in)
        out = repmat(in,ncw,1);
    else
        out = in(1:ncw);
    end
end

function tbs = getTBS(Ninfo,R)
% getTBS returns the calculated transport block size(s) for the shared channel
%   TBS = getTBS(NINFO,R) returns the TBS, as defined in TS 38.214
%   Section 5.1.3.2 and Section 6.1.4.2, with the intermediate number of
%   information bits, NINFO, and target code rate, R.

    % Get the number of codewords
    ncw = numel(Ninfo);

    % Initialize the output
    tbs = zeros(1,ncw);

    % Calculate transport block size, as per TS 38.214 Section 5.1.3.2
    for cwIdx = 1:ncw
        Ninfo_cw = Ninfo(cwIdx);
        R_cw = R(cwIdx);
        if Ninfo_cw <= 3824
            n = max(3,floor(log2(Ninfo_cw))-6);
            % Calculate quantized intermediate number of information bits
            % (Nd_info)
            NdInfo = max(24,(2^n)*floor(Ninfo_cw/(2^n)));
            % Get the TBS value using TS 38.214 Table 5.1.3.2-1
            tbs(cwIdx) = getTBSFromTable(NdInfo);
        else
            n = floor(log2(Ninfo_cw - 24)) - 5;
            % Calculate quantized intermediate number of information bits
            % (Nd_info)
            NdInfo = max(3840,(2^n)*round((Ninfo_cw - 24)/(2^n)));
            if R_cw <= 1/4
                C = ceil((NdInfo + 24)/3816);
            else
                if NdInfo > 8424
                    C = ceil((NdInfo + 24)/8424);
                else
                    C = 1;
                end
            end
            % Calculate TBS
            tbs(cwIdx) = 8*C*ceil((NdInfo + 24)/(8*C)) - 24;
        end
    end
end

function tbs = getTBSFromTable(NdInfo)
% getTBSFromTable returns the TBS value using the standard table
%   TBS = getTBSFromTable(NDINFO) returns the TBS value using TS 38.214
%   Table 5.1.3.2-1, with the quantized intermediate number of information
%   bits, NDINFO.

    persistent tbsTable;

    if isempty(tbsTable)
        % Capture TS 38.214 Table 5.1.3.2-1
        tbsTable = [  24;  32;  40;  48;  56;  64;  72;  80;  88;  96; 104; 112; 120; 128; 136; 144; 152; 160; 168; 176; 184; 192; 208; 224; 240; 256; 272; 288; 304; 320;...
                     336; 352; 368; 384; 408; 432; 456; 480; 504; 528; 552; 576; 608; 640; 672; 704; 736; 768; 808; 848; 888; 928; 984;1032;1064;1128;1160;1192;1224;1256;...
                    1288;1320;1352;1416;1480;1544;1608;1672;1736;1800;1864;1928;2024;2088;2152;2216;2280;2408;2472;2536;2600;2664;2728;2792;2856;2976;3104;3240;3368;3496;...
                    3624;3752;3824];
    end

    % Find the closest TBS value that is not less than NdInfo
    tbsIndex = find(tbsTable >= NdInfo, 1);
    tbs = tbsTable(tbsIndex(1));
end

% Calculate the number of RE available for data (for the purposes of TBS determination)
% across an allocated PRB in a slot
function [nPRB,NREPerPRB] = getNPRBAndNRE(channel)

    % Identify the CDM groups already that are already used for something else
    cdmgroupsinuse = zeros(1,3);
    cdmgroupsinuse(1:channel.DMRS.NumCDMGroupsWithoutData) = 1;
    cdmgroupsinuse(channel.DMRS.CDMGroups+1) = 1;

    % Number of data RE in a PRB containing DM-RS
    ndatare = max(0,12-sum(cdmgroupsinuse)*(4+2*(channel.DMRS.DMRSConfigurationType==1)));  % Or length(phych.DMRS.DMRSSubcarrierLocations)

    % Number of DM-RS containing symbols
    numDMRSSymbols = (1+channel.DMRS.DMRSAdditionalPosition)*channel.DMRS.DMRSLength;
    if ~isempty(channel.DMRS.CustomSymbolSet)
        smap = zeros(1,14);
        smap(channel.SymbolAllocation(1)+1:(channel.SymbolAllocation(1)+channel.SymbolAllocation(2))) = 1;
        smap(channel.DMRS.CustomSymbolSet) = smap(channel.DMRS.CustomSymbolSet+1)+1;
        numDMRSSymbols = sum(smap==2);
    end

    % Number of RE per PRB across slot allocation, available for data for
    % the purposes of the TBS formulae
    NREPerPRB = 12*(channel.SymbolAllocation(2)-numDMRSSymbols) + ndatare*numDMRSSymbols;

    % Number of PRB allocated
    nPRB = numel(channel.PRBSet);
end