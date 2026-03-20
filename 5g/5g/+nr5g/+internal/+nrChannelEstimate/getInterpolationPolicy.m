function [policy,kpattern,nRefSymPRG,fracPRGInd,uprg,prgKranges] = getInterpolationPolicy(K,N,P,refInd,prgSize,prgInfo)
%GETINTERPOLATIONPOLICY gets the processing policy for PRGs in each
%transmit port to speed up channel estimation in the PDSCH PRG bundling
%case when executing on MATLAB.
%
% The processing policy is determined by examining the reference symbol
% allocation inside each PRG. When multiple PRGs have the same relative
% reference symbol allocation, they can be processed in parallel to speed
% up channel estimation.
%
% Outputs:
%
% POLICY     - Processing policy indicated by 0, 1, or 2. 0 indicates that
%              all PRGs on the current OFDM symbol of the current port need
%              to be processed individually. 1 indicates that all PRGs on
%              the current OFDM symbol of the current port except a few
%              fractional PRGs can be processed in parallel in frequency
%              domain processing (CIR denoising, frequency averaging and
%              interpolation). 2 indicates the same frequency domain
%              processing policy as indicated by 1, with the extension that
%              time domain processing (time domain averaging and
%              interpolation) can be performed in parallel as well. NaN
%              indicates no reference symbols are present on the current
%              OFDM symbol of the current port.
% PATTERN    - Reference symbol subcarrier subscripts relative to single
%              PRG shared by all PRGs on the current OFDM symbol of the
%              current port that can be processed in parallel, or [] if all
%              PRGs need to be processed individually.
% NREFSYMPRG - Number of reference symbols in each PRG on the current OFDM
%              symbol of the current port, or [] if no reference symbols
%              are present on the current OFDM symbol of the current port.
% FRACPRGIND - 1-based PRG indices of the fractional PRGs (which need to be
%              processed individually) on the current OFDM symbol of the
%              current port, or [] if no reference symbols are present on
%              the current OFDM symbol of the current port.
% UPRG       - 1-based PRG indices of all transmitted PRGs.
% PRGKRANGES - NPRG-by-2 matrix where NPRG is the number of transmitted
%              PRGs storing the range of subcarrier subscripts for each
%              PRG. The first column is the subcarrier subscript start of
%              each PRG, and the second column is the subcarrier subscript
%              end of each PRG.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    % Initialization
    initEmpty = zeros(0,1);
    coder.varsize('initEmpty',[Inf,1],[1,0]);
    policy = nan(P,N);
    kpattern = repmat({initEmpty},sum(P),sum(N));
    nRefSymPRG = repmat({initEmpty},sum(P),sum(N));
    fracPRGInd = repmat({initEmpty},sum(P),sum(N));

    % Convert reference indices to subscripts
    [ksubs,nsubs,psubs] = ind2sub([K N P], refInd);

    % Find the most common number of RBs in PRG and use this as the
    % effective PRG size. As most PRG are of this size, try to determined
    % if they can be processed in parallel
    if K*N ~= 0 && ~isempty(prgSize)
        % Nonempty input grid with PRG bundling
        rb = unique(floor((ksubs(:)-1)/12));
        [prg,uprg,prgKranges] = getPRGKRanges(rb,prgInfo);
        cnRB = getCommonNRBPerPRG(prg,uprg,prgSize);
    elseif isempty(prgSize)
        % No PRG bundling
        uprg = 1;
        prgKranges = [1 K];
        cnRB = ceil(K/12);
    else
        % Empty input grid
        uprg = zeros(0,1);
        prgKranges = [1 Inf];
        return;
    end

    % Loop over all port
    for p = 1:P

        % Get subcarriers and OFDM symbols of this port
        thisPort = (psubs==p);
        nThisPort = nsubs(thisPort);
        kThisPort = ksubs(thisPort);

        % Find all symbols in this port
        unThisPort = unique(nThisPort);

        % Loop over all symbols
        for sym = 1:numel(unThisPort)

            % Find all subcarriers with reference symbols on this symbol
            thisSym = (nThisPort==unThisPort(sym));
            kThisSym = kThisPort(thisSym);

            if ~isempty(prgSize) && isempty(coder.target)
                % PRG bundling on MATLAB path

                % Loop over all PRGs
                kPattern = [];
                tempPolicy = 1;
                for g = 1:numel(uprg)

                    % If this PRG is fractional, there is no need for
                    % further check in this PRG. Store the PRG index in
                    % fracPRGInd and continue to the next PRG
                    nRBThisPRG = numel(prg(prg==uprg(g)));
                    if nRBThisPRG~=cnRB
                        fracPRGInd{p,unThisPort(sym)} = [fracPRGInd{p,unThisPort(sym)};find(uprg==uprg(g))];
                        continue;
                    end

                    % Get subcarrier subscripts of this PRG
                    kThisPRG = kThisSym(kThisSym>=prgKranges(g,1) & kThisSym<=prgKranges(g,2));

                    % Shift subcarrier subscripts into the range
                    % (1,prgSize*12) for comparison
                    kThisPRG = kThisPRG-prgKranges(g,1)+1;

                    % Establish reference symbol allocation pattern if this
                    % is the first non-fractional PRG, or compare to the
                    % established pattern if a pattern has been established
                    if isempty(kPattern)
                        % This is the first non-fractional PRG, so use this
                        % PRG to establish the pattern
                        kPattern = kThisPRG;
                    else
                        % This is not the first non-fractional PRG, compare
                        % to the established pattern
                        if ~isequal(kPattern,kThisPRG)
                            % If this PRG is out of pattern, then assign
                            % policy '0' and continue to the next OFDM
                            % symbol
                            tempPolicy = 0;
                            break;
                        end
                    end

                end % end of PRG loop

            else
                % No PRG bundling or codegen path

                tempPolicy = 0;
                fracPRGInd{p,unThisPort(sym)} = (1:numel(uprg))';

            end

            % Find the number of reference symbols inside each PRG for this
            % OFDM symbol in this port
            nRefSymPRGThisSym = zeros(numel(uprg),1);
            for g = 1:numel(uprg)
                nRefSymPRGThisSym(g) = sum(kThisSym>=prgKranges(g,1) & kThisSym<=prgKranges(g,2));
            end
            nRefSymPRG{p,unThisPort(sym)} = nRefSymPRGThisSym;

            % Store processing policy for this port and symbol
            policy(p,unThisPort(sym)) = tempPolicy;
            if tempPolicy
                % Store subcarrier subscript pattern if policy is 1
                kpattern{p,unThisPort(sym)} = kPattern;
            else
                % Store PRG indices of fractional PRGs - treat all
                % transmitted PRGs as fractional PRG
                fracPRGInd{p,unThisPort(sym)} = find(nRefSymPRGThisSym>0);
            end

        end % end of symbol loop

        % Check if time domain processing can be performed in parallel if
        % all symbols are assigned policy 1 by checking if they have the
        % same pattern and fractional PRGs
        if ~isempty(unThisPort) && all(policy(p,unThisPort))
            pattern0 = kpattern{p,unThisPort(1)};
            fracInd0 = fracPRGInd{p,unThisPort(1)};
            tempFlag = 1;
            for sym = 1:numel(unThisPort)
                if ~isequal(pattern0,kpattern{p,unThisPort(sym)}) || ...
                        ~isequal(fracInd0,fracPRGInd{p,unThisPort(sym)})
                    tempFlag = 0;
                    break;
                end
            end
            if tempFlag
                policy(p,unThisPort) = 2;
            end
        end

    end % end of port loop

end

%% Local function

% Get the most common number of RBs per PRG. This helps the main function
% make smarter decision on processing policy than simply relying on prgSize
% in some corner cases (e.g. a grid with 2 PRGs where both PRGs are
% fractional and of the same size)
function cnRB = getCommonNRBPerPRG(prg,uprg,prgSize)

    % Get the number of RBs in each PRG. countMat is a NRB-by-NPRG logical
    % matrix where NRB is the total number of RBs and NPRG is the total
    % number of PRGs. countMat(i,j) is true if the i-th RB is in the j-th
    % PRG, and false otherwise. Summing countMat on the first dimension
    % would then give a NPRG-element row vector, whose elements are the
    % number of RBs in the corresponding PRG.
    prgSet_t = repmat(prg,1,numel(uprg));
    uprg_t = repmat(uprg',numel(prg),1);
    countMat = (prgSet_t==uprg_t);
    nRBPerPRG = (sum(countMat,1)).';

    coder.varsize('cnRB',[1 1],[1 1]);
    % Find the most common number of RBs per PRG.
    cnRB = mode(nRBPerPRG);

    % If only one PRG has cnRB RBs, then that means all PRGs have different
    % number of RBs. In this case, use the bundle size prgSize as cnRB and
    % consider all PRGs as 'fractional'
    if isscalar(find(nRBPerPRG==cnRB)) && ~isscalar(nRBPerPRG)
        cnRB = prgSize;
    end

end

% Get the k (subcarrier) subscript ranges for each PRG. kRanges is a
% NPRG-by-2 matrix, whose i-th row corresponds to the i-th transmitted PRG.
% The first column of kRanges is the lower edge of the PRGs, or
% equivalently the upper bound of the k subscript in the grid for this PRG.
% The second column of kRanges is the upper edge of the PRGs, or
% equivalently the lower bound of the k subscript in the grid for this PRG.
% For example, for a grid consisting of 3 PRGs, kRanges is a 3-by-2 matrix:
%                -------                                  k subscript
% kRanges(1,1) -> XXXXX                                        1
%                 XXXXX                  PRG 1                 .
%                 XXXXX <- kRanges(1,2)                        .
%                -------                                       .
% kRanges(2,1) -> XXXXX                                        .
%                 XXXXX                  PRG 2                 .
%                 XXXXX <- kRanges(2,2)                        .
%                -------                                       .
% kRanges(3,1) -> XXXXX                                        .
%                 XXXXX                  PRG 3                 .
%                 XXXXX <- kRanges(3,2)                        K
%                -------                                       
% The k subscript (in the grid) of the reference symbols inside the i-th
% PRG is then k(k>=kRanges(i,1) & k<=kRanges(i,2))
function [prg,uprg,kRanges] = getPRGKRanges(rb,prgInfo)

    prg = prgInfo.PRGSet(rb+1);
    uprg = unique(prg);

    kRanges = zeros(numel(uprg),2);

    for i = 1:numel(uprg)

        prgInd = uprg(i);
        rbLow = min(find(prgInfo.PRGSet==prgInd)-1);
        rbHigh = max(find(prgInfo.PRGSet==prgInd)-1);
        rbIndices = rb(rb>=rbLow & rb<=rbHigh);
        kRanges(i,1) = (min(rbIndices+1)-1)*12+1;
        kRanges(i,2) = (max(rbIndices+1))*12;

    end

end