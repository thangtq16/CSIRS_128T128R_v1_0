function [nVar,estConfig,info] = estimateNoise(H,Hn,R,ECP,fdCDM,tdCDM,refRBs,estConfig,prgKranges)
%ESTIMATENOISE performs noise estimation and averaging window selection.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    nPRG = size(prgKranges,1);

    nVar = zeros(nPRG,1,class(H));
    avgWs = repmat(estConfig.AveragingWindow,nPRG,1);

    gridSize = size(Hn);
    indn = find(~isnan(Hn));
    [kn,nn,rn,pn] = ind2sub(gridSize,indn);

    % Loop over all PRGs
    for g = 1:nPRG

        ind = find(kn>=prgKranges(g,1) & kn<=prgKranges(g,2));
        knThisPRG = kn(ind);
        indnThisPRG = sub2ind(gridSize,kn(ind),nn(ind),rn(ind),pn(ind));
        HnThisPRG = Hn(indnThisPRG);
        subnThisPRG = [knThisPRG,nn(ind),rn(ind),pn(ind)];
        avgWThisPRG = avgWs(g,:);
        refRBsThisPRG = extractRefRBsPRG(refRBs,prgKranges(g,1),prgKranges(g,2));
        [nVarThisPRG,avgW] = estimateNoiseCore(H,HnThisPRG,indnThisPRG, ...
            subnThisPRG,R,ECP,fdCDM,tdCDM,refRBsThisPRG,avgWThisPRG);
        nVar(g) = nVarThisPRG;
        avgWs(g,:) = avgW;

    end

    % Output results
    estConfig.AveragingWindow = avgWs;
    info.AveragingWindow = avgWs;

end

%% Local functions

% Core function for noise estimation
function [nVar,avgW] = estimateNoiseCore(H,Hn,indn,subn,R,ECP,fdCDM,tdCDM,refRBs,avgW)

    if (numel(Hn)>R)
        Hdn = H(indn);
        subnR1 = subn(subn(:,3)==1,[1 2 4]); % 1st receive antenna
        [s,dk] = determineNoiseScaling(ECP,subnR1,fdCDM,tdCDM);
        nVar = determineNoiseVariance(Hn,Hdn,s,dk);
        % Determine how the channel CSI will affect the receiver SNR
        % (assuming MMSE equalization)
        c = determineCSIScaling(refRBs,H,nVar);
    else
        nVar = zeros(1,1,class(H));
        c = ones(1,1,class(H));
    end

    % If either averaging parameter is not set, automatically choose a
    % value based on the SNR (calculated from the noise variance and the
    % effect of CSI)
    avgW = setAveragingParameters(avgW(:),nVar*c);

end

% Determine noise scaling based on CP length, reference symbol layout, and
% CDM parameters
function [s,dk] = determineNoiseScaling(ECP,subn,fdCDM,tdCDM)
    
    % Determine reference symbol frequency spacings 'deltas' and their
    % average 'dk'. Note that spacings greater than 12 are initially
    % preserved, which allows for the case of widely-spaced reference
    % signals with less than one RE per RB. If the resulting average
    % spacing 'dk' is then less than or equal to 12, the average spacing is
    % recalculated with spacings greater than 12 being limited to 12, for
    % backwards compatibility with earlier versions of this function
    deltas = calculateDeltas(subn);
    dk = mean(deltas);
    if (dk<=12)
        deltas = min(deltas,12);
        dk = mean(deltas);
    end

    % The effect of FD-CDM despreading, interpolation and CIR denoising on
    % the noise variance is influenced by the FD-CDM despreading length,
    % the reference symbol frequency spacing, and the cyclic prefix length.
    % For sufficiently large 'dk' and/or 'fdCDM', the frequency span of the
    % averaging exceeds the bandwidth of the interpolation and CIR
    % denoising, and the scaling factor is a constant multiple of 'fdCDM'.
    % 'log10s' contains log10 of the noise scaling factors for smaller
    % values of 'dk' and 'fdCDM'. 'p1' and 'p2' are first order polynomials
    % for creating the scaling factors for larger values of 'dk' with
    % fdCDM=1 and fdCDM=2 respectively
    if (ECP)
        %  dk:         1      2      3      4
        log10s = [0.3599 1.3548 2.9660 4.1610;  % fdCDM=1
                  1.0524 1.2727      0      0]; % fdCDM=2
        p1 = [0.3157 2.9911]; % fdCDM=1
    else
        %  dk:         1      2      3      4      5      6
        log10s = [0.0713 0.1654 0.2845 0.4164 0.6409 0.9259;  % fdCDM=1
                  0.4633 0.6965 0.9379 1.1196 1.2059      0;  % fdCDM=2
                  0.7403 1.0876 1.2757      0      0      0;  % fdCDM=3
                  0.9720 1.3307 1.4028      0      0      0;  % fdCDM=4
                  1.2079 1.4707 1.4990      0      0      0;  % fdCDM=5
                  1.3616 1.5517 1.5961      0      0      0;  % fdCDM=6
                  1.5208 1.6357 1.6470      0      0      0]; % fdCDM=7
        p1 = [0.4342 -1.7355]; % fdCDM=1
    end
    p2 = [0.0000 1.2727]; % fdCDM=2

    % Establish if reference symbols are arranged in groups of adjacent
    % resource elements. This arrangement influences the noise variance for
    % fdCDM=1 and fdCDM=2, so specific scaling factors are used
    if (any(fdCDM==[1 2]))
        % Get unique frequency spacings
        udk = unique(deltas);
        % If there are two unique spacings and the minimum spacing is 1
        % (adjacent resource elements)
        if (numel(udk)==2 && min(udk)==1)
            % Update 'dk' to be the other spacing and update the scaling
            % factors. Note that the first column of 'log10s' corresponds
            % to dk=1 which is not used as 'dk' is now greater than 1
            dk = max(deltas);
            if (ECP)
                log10s(1,:) = [NaN 0.6880 1.2332 1.6372];
                log10s(2,:) = [NaN 1.6140 1.9154 0.0000];
                p1 = [0.1140 1.3631];
                p2 = [0.1031 1.8078];
            else
                log10s(1,:) = [NaN 0.1087 0.1203 0.1070 0.0477 0.0090];
                log10s(2,:) = [NaN 0.0000 0.0000 0.0000 0.0000 0.0000];
                p1 = [0.1343 -0.9540];
                p2 = [0.2745 -0.0861];
            end
        end
    end
    
    if (dk<=12)
    
        % General case, constant multiple of fdCDM
        s = fdCDM * 6.41;
        
        % For tabulated fdCDM values 
        if (fdCDM<=size(log10s,1))
            
            % Select the tabulated entries for the configured fdCDM value
            log10s = log10s(fdCDM,:);
            log10s(log10s==0) = [];
            ndk = length(log10s);
            
            if (dk>=1 && dk<=ndk)
                % For 'dk' in the tabulated range, interpolate between the 
                % tabulated entries in 'log10s'
                y = interp1(1:ndk,log10s,dk);
                s = 10^y;
            elseif (dk>ndk && any(fdCDM==[1 2]))
                % For larger 'dk' with fdCDM=1 or fdCDM=2, interpolate
                % using first order polynomial 'p'
                if (fdCDM==1)
                    p = p1;
                else % fdCDM==2
                    p = p2;
                end
                y = (p(1)*dk + p(2));
                s = 10^y;
            end
            
        end

    else

        % Scaling appropriate for high-pass filtering of 'Hn' using diff()
        s = 1/2 * fdCDM^2;

    end

    % The effect of TD-CDM despreading on the noise variance is simply a
    % processing gain factor given by the CDM despreading length 'tdCDM'
    if (tdCDM>1)
        s = s * tdCDM;
    end
    
end

% Determine the noise variance, using the most appropriate technique given
% the reference symbol frequency spacing 'dk'
function nVar = determineNoiseVariance(Hn,Hdn,s,dk)

    if (dk<=12)
        % Perform noise estimation by measuring the variance of the
        % difference between the original LS estimates 'Hn' and the same
        % locations in the denoised estimate 'Hdn'
        nVar = var(Hn - Hdn) * s;
    else
        % For large reference symbol frequency spacing 'dk', estimating the
        % noise via the CIR denoising is not effective - the noise on the
        % widely-spaced reference symbols is mostly preserved by the CIR
        % denoising. Therefore the part of the expected variance
        % E[(Hn-Hdn)^2] due to noise is very small, and its estimation is
        % susceptible to any self-noise in the CIR denoising. In this case
        % an alternative noise estimation approach is used - applying a
        % high-pass filter to original LS estimates 'Hn' and measuring the
        % variance
        nVar = var(diff(Hn)) * s;
    end

end

% Determine how the channel CSI will affect the receiver SNR 
% (assuming MMSE equalization)
function c = determineCSIScaling(refRBs,H,nVar)

    % Create a vector used to store CSI values
    csi = [];

    % Get the RBs (0-based) with reference symbols in any OFDM symbol (2nd
    % dimension of 'refRBs') and any port (3rd dimension of 'refRBs')
    RBs = find(any(refRBs,[2 3])) - 1;
    
    % For each RB
    for i = 1:numel(RBs)

        % Prepare logical indices 'x' giving the OFDM symbols for which the
        % current RB has reference symbols on any port
        rb = RBs(i);
        x = any(refRBs(rb + 1,:,:),3);

        % Check that the set of ports 'p' which have reference symbols is
        % the same for every OFDM symbol; if not, skip this RB
        px = permute(refRBs(rb+1,x,:),[2 3 1]);
        p = find(px(1,:));
        if (~all(px(1,:)==px,'all'))
            continue;
        end
        
        % Extract the channel matrix 'Hrb' from one RE of the current RB,
        % across the OFDM symbols 'x', all receive antennas, and set of
        % ports 'p'
        Hrb = H((rb*12) + 6,x,:,p);
        
        % Permute 'Hrb' to give an R-by-P-by-N array where R is the number
        % of receive antennas, P is the number of ports, and N is the
        % number of active OFDM symbols
        P = numel(p);
        Hrb = permute(Hrb,[3 4 2 1]);
        
        % Compute the CSI for each R-by-P channel matrix 'Hn' in 'Hrb'
        for n = 1:size(Hrb,3)

            Hn = Hrb(:,:,n);
            csi = [csi; svd(Hn'*Hn + nVar*eye(P)/sqrt(P))]; %#ok<AGROW>

        end

    end

    % Compute factor 'c' used to scale the noise estimate 'nVar', such that
    % 1/(nVar*c) gives an estimate of the average receiver SNR assuming
    % MMSE equalization
    if (~isempty(csi) && ~any(csi==0))
        c = mean(1 ./ csi);
    else
        c = ones(1,1,class(H));
    end

end

% If either averaging parameter is not set, automatically choose a value
% based on the SNR (calculated from the noise variance)
function avgW = setAveragingParameters(avgW,nVar)

    if (any(avgW==0))
        
        SNR = -10 * log10(nVar);
        
        % Frequency averaging
        if (avgW(1)==0)
            if (SNR>20)
                avgW(1) = 1;
            elseif (SNR>7)
                avgW(1) = 3;
            elseif (SNR>5)
                avgW(1) = 5;
            else % SNR<=5
                avgW(1) = 7;
            end
        end
        
        % Time averaging
        if (avgW(2)==0)
            if (SNR>8)
                avgW(2) = 1;
            else % SNR<=8
                avgW(2) = 3;
            end
        end
        
    end
    
end

% Determine reference symbol frequency spacings. Each column of 'idx' will
% have two rows, giving the index of the first and last element of the
% subscripts for each OFDM symbol and antenna plane in 'subn'. Then for
% each OFDM symbol and antenna plane the reference symbol frequency spacing
% is calculated
function deltas = calculateDeltas(subn)

    idx = [[0 0]; subn(:,2:3); [0 0]];
    idx = find(sum(diff(idx)~=0,2));
    idx = [idx(1:end-1) idx(2:end)-1].';
    deltas = [];
    for i = 1:size(idx,2)
        nk = (idx(2,i)-idx(1,i)+1);
        delta = diff(subn(idx(1,i):idx(2,i),1));
        if (nk>1)
            deltas = [deltas; delta]; %#ok<AGROW>
        end
    end

end

% Blank out any reference symbols outside the current PRG, whose subcarrier
% subscripts are within the range [low,high]
function Aout = extractRefRBsPRG(Ain,low,high)

    Aout = Ain;
    rbLow = floor((low-1)/12)+1;
    rbHigh = floor((high-1)/12)+1;
    Aout(1:rbLow-1,:,:,:) = 0;
    Aout(rbHigh+1:end,:,:,:) = 0;

end