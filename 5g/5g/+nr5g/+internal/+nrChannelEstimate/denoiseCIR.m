function [H,Hn] = denoiseCIR(p,K,N,P,R,eK,refInd,kmin,kmax,kpatternmin,kpatternmax,refRBs,H,Hn,fdCDM,w,policy,pattern,prgKranges,fracPRGInd)
%DENOISECIR performs the frequency interpolation and CIR denoising
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    % Get frequency (subcarrier k) and time (OFDM symbol n) subscripts
    % of the reference signal for the current port
    [ksubs,nsubs] = getPortSubscripts(K,N,P,refInd,p);
    NRB = K/12;
    eRB = eK/12;
    totalNRB = NRB+eRB;
    siz = [K+eK N R P];
    nPRG = size(prgKranges,1);

    % Create interpolators ('vpisLow' and 'vpisHigh') that will be used
    % to create virtual pilots (VPs) for the low and high edge of the
    % grid for this port, and the corresponding sets of unique minima
    % ('ukLow') and maxima ('ukHigh') frequency subscripts (one value
    % for each interpolator). The interpolators use reference symbols
    % from any OFDM symbol across the whole reference resource grid,
    % and whose frequency subscripts are near the minimum or maximum
    % frequency subscript for that interpolator
    rangeLow = [-12 eK/2+12];
    rangeHigh = [-eK/2-12 12];
    [vpisLow,ukLow,ndsLow] = ...
        createVPInterpolatorsFractionalPRGs(kmin(:,p,:),K,eK,ksubs,nsubs,rangeLow);
    [vpisHigh,ukHigh,ndsHigh] = ...
        createVPInterpolatorsFractionalPRGs(kmax(:,p,:),K,eK,ksubs,nsubs,rangeHigh);
    [vpisLowPar,ndsLowPar,vpisHighPar,ndsHighPar] = ...
        createVPInterpolatorsParallelPRGs(p,policy,pattern,K,eK,kpatternmin,kpatternmax,rangeLow,rangeHigh);

    % For each OFDM symbol
    un = unique(nsubs).';
    for uni = 1:numel(un)

        % Get frequency subscripts for the current OFDM symbol
        n = un(uni);
        k = ksubs(nsubs==n);

        % Get processing policy for this symbol and fractional PRG indices
        policyThisSym = policy(p,n);
        fracPRGIndThisSym = fracPRGInd{p,n};

        for g = 1:numel(fracPRGIndThisSym)
            % Process the PRGs that need to be processed individually

            prgKrange = prgKranges(fracPRGIndThisSym(g),:);
            kThisPRG = k(k>=prgKrange(1) & k<=prgKrange(2));
            prgStart = prgKrange(1);
            kminThisPRG = kmin(n,p,fracPRGIndThisSym(g));
            kmaxThisPRG = kmax(n,p,fracPRGIndThisSym(g));

            % Find the VP interpolators for this PRG
            vpiIdxLow = find(ukLow==kminThisPRG);
            vpiLow = vpisLow{vpiIdxLow(1)};
            ndLow = ndsLow(vpiIdxLow(1));
            vpiIdxHigh = find(ukHigh==kmaxThisPRG);
            vpiHigh = vpisHigh{vpiIdxHigh(1)};
            ndHigh = ndsHigh(vpiIdxHigh(1));

            % Process VP interpolators to blank out any reference symbols
            % outside this PRG, as in PRG bundling case, VP region may
            % overlap with the adjacent PRG and the VP region is then
            % erroneously non-empty
            vpiLow = processVPInterpolator(vpiLow,kminThisPRG+eK/2,kmaxThisPRG-eK/2,ndLow);
            vpiHigh = processVPInterpolator(vpiHigh,kminThisPRG+eK/2,kmaxThisPRG-eK/2,ndHigh);

            % CIR denoising
            [H,Hn] = ...
                denoiseCIRBlockCore(p,n,kThisPRG,ksubs,nsubs,siz,K,R,eK, ...
                totalNRB,kminThisPRG,kmaxThisPRG,refRBs,H,Hn,fdCDM,w,0, ...
                prgStart,vpiLow,ndLow,vpiHigh,ndHigh);

        end

        if policyThisSym && isempty(coder.target)
            % Process the PRGs that can be processed in parallel, avoiding
            % unnecessary code generation

            kPattern = pattern{p,n};
            prgIndices = true(nPRG,1);
            prgIndices(fracPRGIndThisSym) = false;
            allPRGs = (1:nPRG)';
            allPRGs = allPRGs(prgIndices);

            if ~isempty(allPRGs)
                % In corner cases, non-fractional PRGs may have no
                % reference symbols

                prgStarts = prgKranges(prgIndices,1);
                kminPattern = kpatternmin(n,p);
                kmaxPattern = kpatternmax(n,p);

                % Find VP interpolators for this group of PRG
                if policyThisSym==2
                    vpiIdx = 1;
                else
                    vpiIdx = uni;
                end
                vpiLow = vpisLowPar{vpiIdx};
                ndLow = ndsLowPar(vpiIdx);
                vpiHigh = vpisHighPar{vpiIdx};
                ndHigh = ndsHighPar(vpiIdx);


                % CIR denoising
                [H,Hn] = ...
                    denoiseCIRBlockCore(p,n,kPattern,ksubs,nsubs,siz,K,R,eK, ...
                    totalNRB,kminPattern,kmaxPattern,refRBs,H,Hn,fdCDM,w,1, ...
                    prgStarts,vpiLow,ndLow,vpiHigh,ndHigh);

            end

        end
    end

end

%% Local functions

% Core function for frequency interpolation and CIR denoising
function [H,Hn] = denoiseCIRBlockCore(p,n,k,ksubs,nsubs,siz,K,R,eK,totalNRB,kmin,kmax,refRBs,H,Hn,fdCDM,w,policy,prgStarts,vpiLow,ndLow,vpiHigh,ndHigh)

    % Initialize frequency subscripts 'ke' to be used for interpolation,
    % taking extra subcarriers into account. Also store these as 'ke0'
    % which is used to reset these subscripts for each receive antenna
    ke = k + eK/2;
    ke0 = ke;
    nPRG = size(prgStarts,1);

    % Skip the PRGs without any reference symbols
    if kmin==0 && kmax==0
        return;
    end

    % Calculate range of frequency subscripts for this OFDM symbol and
    % port, and expand it for reinserting into channel estimate grid
    krange = (kmin:kmax).';
    kRel = ((eK/2+1):(size(krange,1)-eK/2))';

    % Record subcarrier subscripts used for extracting from and recording
    % into 'H' and 'Hn'. The subcarrier subscripts need to be expanded when
    % multiple PRGs are being processed in parallel
    if policy
        keFull = nr5g.internal.nrChannelEstimate.expandSubcarrierSubscripts(ke0,prgStarts);
        krangeFull = nr5g.internal.nrChannelEstimate.expandSubcarrierSubscripts(krange,prgStarts);
        kRelFull = nr5g.internal.nrChannelEstimate.expandSubcarrierSubscripts(kRel,prgStarts);
    else
        keFull = ke0;
        krangeFull = krange;
        kRelFull = krangeFull(kRel);
    end

    % Calculate 0-based RB values 'edgeRBs' corresponding to the edges of
    % gaps in the RB allocation for this PRG, symbol and port. The first
    % element of 'edgeRBs' is the last RB before a gap, the second element
    % is the first RB after the gap, and so on for each gap
    edgeRBs = find(refRBs(:,n,p)) - 1;
    rbidx = reshape(contiguousRBs(edgeRBs,totalNRB),[],1);
    edgeRBs = edgeRBs(rbidx(2:end-1));

    % Only handles the gap inside this PRG
    rbRange = unique(floor(([kmin kmax]-1)/12));
    gapStart = edgeRBs(1:2:end);
    gapEnd = edgeRBs(2:2:end);
    idx = find(gapStart>=rbRange(1) & gapEnd<=rbRange(2));
    if ~isempty(idx)
        idx = reshape(((idx-1)*2+(1:2))',[],1);
    end
    edgeRBs = edgeRBs(idx);

    % Create interpolators ('vpisGap') that will be used to create VPs for
    % gaps in the RB allocation for this OFDM symbol and port. The
    % interpolators use reference symbols from any OFDM symbol across the
    % whole reference resource grid, and whose frequency subscripts are
    % near the minimum or maximum frequency subscript for that interpolator
    if policy
        [vpisGap,ndsGap] = ...
            createGapVPInterpolatorsParallelPRG(edgeRBs,K,eK,k,n);
    else
        [vpisGap,~,ndsGap] = ...
            createGapVPInterpolatorsFractionalPRG(edgeRBs,K,eK,ksubs,nsubs,kmin,kmax);
    end

    % Extract channel estimate regions that will be used to provide values
    % for the VP interpolation for each receive antenna
    HvpLow = extractVPs(vpiLow,ndLow,siz,n,p,H,policy,prgStarts);
    HvpHigh = extractVPs(vpiHigh,ndHigh,siz,n,p,H,policy,prgStarts);
    HvpsGap = extractGapVPs(ndsGap,vpisGap,siz,n,p,H,policy,prgStarts);

    % Loop over receive antennas
    for r = 1:R

        % Initialize frequency subscripts to be used for interpolation
        ke = ke0;

        % Get LS estimates corresponding to this antenna, of the current
        % PRG (groups), symbol and port
        H_LS = H(keFull,n,r,p);

        % Record LS estimates for use in noise estimation
        Hn(keFull,n,r,p) = H_LS;

        % Reshape H_LS to put multiple PRGs on different pages
        H_LS = reshape(H_LS,[],1,nPRG);

        % If FD-CDM despreading is configured, replace groups of LS
        % estimates with a single estimate in the position of the average
        % frequency index
        if (fdCDM>1)
            H_LS = H_LS(1:fdCDM:end,1,:);
            nkCDM = min(fdCDM,numel(ke));
            m = mod(numel(ke),nkCDM);
            km = ke(end-m+1:end);
            ke = reshape(ke(1:end-m),nkCDM,[]);
            ke = [mean(ke,1).'; repmat(mean(km),double(m~=0),1)];
        end

        % Create virtual pilots at the lower edge: every 6 subcarriers
        % between the minimum frequency subscript 'kmin' and the lowest
        % reference signal subcarrier
        kvpLow = (kmin:6:(ke(1)-1)).';
        if policy
            vpLow = createVPs(ndLow,vpiLow,kvpLow,n,HvpLow(:,r,:),prgStarts);
        else
            vpLow = createVPs(ndLow,vpiLow,kvpLow,n,HvpLow(:,r,:));
        end

        % Create virtual pilots at the upper edge: every 6 subcarrier
        % between the maximum frequency subscript 'kmax' and the highest
        % reference signal subcarrier
        kvpHigh = flipud((kmax:-6:(ke(end)+1)).');
        if policy
            vpHigh = createVPs(ndHigh,vpiHigh,kvpHigh,n,HvpHigh(:,r,:),prgStarts);
        else
            vpHigh = createVPs(ndHigh,vpiHigh,kvpHigh,n,HvpHigh(:,r,:));
        end

        % Create virtual pilots in gaps: every 6 subcarriers between the
        % edge of the gap and either the middle of the gap or eK/2
        % subcarriers away, whichever is closer. Exclude any elements
        % already present in lower edge or upper edge virtual pilots
        if policy
            [kvpGap,vpGap] = ...
                createGapVPs(ndsGap,vpisGap, ...
                edgeRBs,n,HvpsGap,r,ke,eK,[kvpLow; kvpHigh],prgStarts);
        else
            [kvpGap,vpGap] = ...
                createGapVPs(ndsGap,vpisGap, ...
                edgeRBs,n,HvpsGap,r,ke,eK,[kvpLow; kvpHigh]);
        end

        % Perform interpolation in the frequency direction to give a
        % channel estimate for all relevant frequency subscripts, and
        % assign the estimate into the appropriate region of the overall
        % channel estimate array
        [kevp,keidx] = sort([ke; kvpLow; kvpGap; kvpHigh]);
        Hvp = [H_LS; vpLow; vpGap; vpHigh];
        Hvp = Hvp(keidx,:,:);
        Hk = nr5g.internal.nrChannelEstimate.interpolateGrid(1,kevp,n,Hvp,krange,n,'spline',0);

        % Compute average for each individual PRG, remove them, and insert
        % into original positions in the grid
        averageH = mean(Hk,1);
        Hk_shifted = Hk-averageH;

        % Insert back into the absolute position in the grid for CIR
        % denoising
        H_temp = complex(zeros([K+eK,1,nPRG],'like',Hk_shifted));
        krangeFullPG = reshape(krangeFull,[],1,nPRG);
        for pg = 1:nPRG
            kThisPRG = krangeFullPG(:,1,pg);
            H_temp(kThisPRG,1,pg) = Hk_shifted(:,1,pg);
        end

        % Perform an IDFT of the channel estimate to give the CIR and apply
        % the time domain windowing function, then perform a DFT to give
        % the denoised CIR
        h = ifft(H_temp);
        h = h.*w;
        H_temp = fft(h);

        % Extract relevant region, reinsert average, blank out the VP
        % regions, and then assign back into H
        H_rel = zeros(size(Hk_shifted),'like',Hk_shifted);
        for pg = 1:nPRG
            kThisPRG = krangeFullPG(:,:,pg);
            H_rel(:,1,pg) = H_temp(kThisPRG,1,pg)+averageH(:,1,pg);
        end
        H_rel = H_rel(kRel,:,:);
        H_rel = reshape(H_rel,[],1);
        H(kRelFull,n,r,p) = H_rel;

    end

end

% Create interpolators used to create virtual pilots. 'vpis' is the set of
% interpolators with each element corresponding to the unique frequency
% subscript in the corresponding element of 'uk'. 'nd' is the number of
% dimensions over which each interpolator operates (0, 1 or 2)
function [vpis,uk,nds] = createVPInterpolatorsFractionalPRGs(kedge,K,eK,ksubs,lsubs,range)

    % Determine the set of unique frequency subscripts
    kedge = reshape(kedge,[],1);
    uk = unique(kedge(kedge~=0)).';
    
    % Take extra subcarriers into account by adjusting reference symbol
    % frequency subscripts
    ksubs = ksubs + eK/2;
    
    % For each unique frequency subscript
    vpis = coder.nullcopy(cell([1 numel(uk)]));
    nds = zeros([1 numel(uk)]);

    for i = 1:numel(uk)

        % Determine the start and end frequency subscripts of reference
        % signals which will contribute to the interpolation performed by
        % this interpolator
        kstart = max(uk(i) + range(1),1);
        kend = min(uk(i) + range(2),K+eK);

        % Create VP interpolator corresponding to 'uk(i)'
        [vpi,nd] = createSingleVPInterpolator(ksubs,lsubs,kstart,kend);
        vpis{i} = vpi;
        nds(i) = nd;

    end
    
end

% Create interpolators used to create virtual pilots for PRGs that are
% processed in parallel
function [vpisLow,ndsLow,vpisHigh,ndsHigh] = createVPInterpolatorsParallelPRGs(p,policy,pattern,K,eK,kpatternmin,kpatternmax,rangeLow,rangeHigh)
 
    % Find the number of VP interpolators that need to be created. Only
    % need to consider symbols with policy>=1, as policy=0 indicates all
    % PRGs need to be processed individually
    policyThisPort = policy(p,policy(p,:)>0);
    symIndices = find(policy(p,:)>0);

    % If all symbols share the same kpattern, then only one VP interpolator
    % is need for all symbols. Otherwise, create one VP interpolator for
    % each symbol
    commonPattern = double(all(policyThisPort==2));
    numVPs = double(~isempty(policyThisPort))*(commonPattern+(1-commonPattern)*numel(symIndices));

    % Initialize
    vpisLow = coder.nullcopy(cell([1 numVPs]));
    ndsLow = zeros([1 numVPs]);
    vpisHigh = coder.nullcopy(cell([1 numVPs]));
    ndsHigh = zeros([1 numVPs]);

    if ~isempty(coder.target)
        % Avoid unnecessary codegen as parallel processing is not supported
        % on codegen path
        return;
    end

    for i = 1:numVPs

        % Find current symbol(s) and subcarriers
        if commonPattern
            lsubs0 = symIndices;
            n = lsubs0(i);
        else
            lsubs0 = symIndices(i);
            n = lsubs0;
        end
        ksubs0 = pattern{p,n}+eK/2;
        ksubs = repmat(ksubs0,numel(lsubs0),1);
        lsubs = reshape(repmat(lsubs0,numel(ksubs0),1),[],1);

        % Extract kmin and kmax for the current symbol(s)
        kmin = kpatternmin(n,p);
        kmax = kpatternmax(n,p);

        % Find kstart for both lower and higher boundary
        kstartLow = max(kmin+rangeLow(1),1);
        kendLow = min(kmin+rangeLow(2),K+eK);
        kstartHigh = max(kmax+rangeHigh(1),1);
        kendHigh = min(kmax+rangeHigh(2),K+eK);

        % Create VP interpolators
        [vpiLow,ndLow] = createSingleVPInterpolator(ksubs,lsubs,kstartLow,kendLow);
        vpisLow{i} = vpiLow;
        ndsLow(i) = ndLow;
        [vpiHigh,ndHigh] = createSingleVPInterpolator(ksubs,lsubs,kstartHigh,kendHigh);
        vpisHigh{i} = vpiHigh;
        ndsHigh(i) = ndHigh;

    end

end

% Create a single VP interpolator to be stored in 'vpis{idx}' from
% subcarrier subscripts 'ksubs' and symbol subscripts 'lsubs' within the
% subcarrier range indicated by 'kstart' and 'kend'
function [vpi,nd] = createSingleVPInterpolator(ksubs,lsubs,kstart,kend)

    % Determine the actual reference signal subcarrier and OFDM symbol
    % subscripts ('kvp' and 'lvp') within those limits
    vpidx = ((ksubs >= kstart) & (ksubs <= kend));
    kvp = ksubs(vpidx);
    lvp = lsubs(vpidx);

    % If there are multiple subcarriers and/or multiple OFDM symbols,
    % construct the interpolator. Values are set to zero and will be
    % provided per receive antenna when performing interpolation. For
    % cases with a single subcarrier and a single OFDM symbol,
    % interpolation is not performed
    multiSubcarrier = (numel(unique(kvp)) > 1);
    multiSymbol = (numel(unique(lvp)) > 1);
    if (multiSubcarrier && multiSymbol)
        nd = 2;
        % 2-D interpolation, record subcarrier and OFDM symbol
        % subscripts
        vpi = [kvp lvp];
    elseif (multiSubcarrier) % && ~multiSymbol
        nd = 1;
        % 1-D interpolation
        if (isempty(coder.target))
            % Use gridded interpolant
            vpi = griddedInterpolant(kvp,zeros(size(kvp)),'linear');
        else
            % Record subcarrier subscripts
            vpi = kvp;
        end
    else % ~multiSubcarrier && ~multiSymbol
        nd = 0;
        % "0-D interpolation", just record the subscript of the single
        % value from which the VPs will be copied (this subscript will
        % not actually be used)
        vpi = unique(kvp);
    end

end

% Create interpolators used to create virtual pilots in gaps in the RB
% allocation. The outputs are as described for the createVPInterpolators
% functions
function [vpis,uk,nd] = createGapVPInterpolatorsFractionalPRG(RBs,K,eK,ksubs,lsubs,kmin,kmax)

    % For each gap edge
    G = numel(RBs);
    vpis = coder.nullcopy(cell([1 G]));
    uk = zeros([1 G]);
    nd = zeros([1 G]);
    for g = 1:G
        
        % If the edge is a "high edge" i.e. the gap is above RBs(g)
        if (mod(g,2))
            kgap = RBs(g)*12 + (eK/2) + 12;
            range = [-eK/2-12 12];
        else % the edge is a "low edge" i.e. the gap is below RBs(g)
            kgap = RBs(g)*12 - (eK/2) + 1;
            range = [-12 eK/2+12];
        end
        
        % Create the interpolator and blank out any reference symbols from
        % the adjacent PRG
        [v,uk(g),nd(g)] = createVPInterpolatorsFractionalPRGs(kgap,K,eK,ksubs,lsubs,range);
        v{1} = processVPInterpolator(v{1},kmin,kmax,nd(g));
        vpis{g} = v{1};
        
    end
    
end

% Create interpolators used to create virtual pilots in gaps in the RB
% allocation for PRGs processed in parallel
function [vpisGap,ndsGap] = createGapVPInterpolatorsParallelPRG(RBs,K,eK,pattern,n)

    % For each gap edge
    G = numel(RBs);
    vpisGap = coder.nullcopy(cell([1 G]));
    ndsGap = zeros([1 G]);
    for g = 1:G

        % If the edge is a "high edge" i.e. the gap is above RBs(g)
        if (mod(g,2))
            kgap = RBs(g)*12 + (eK/2) + 12;
            range = [-eK/2-12 12];
        else % the edge is a "low edge" i.e. the gap is below RBs(g)
            kgap = RBs(g)*12 - (eK/2) + 1;
            range = [-12 eK/2+12];
        end

        % Find kstart, kend, ksubs, and lsubs
        kstart = max(kgap+range(1),1);
        kend = min(kgap+range(2),K+eK);
        ksubs = pattern;
        lsubs = n;

        % Create VP interpolator
        [vpi,nd] = createSingleVPInterpolator(ksubs,lsubs,kstart,kend);
        vpisGap{g} = vpi;
        ndsGap(g) = nd;

    end

end

% Process VP interpolator to blank out any reference symbols outside the
% current PRG. This is possible due to PRG bundling when PRG size is
% smaller than eK/2
function vpi = processVPInterpolator(vpi,kmin,kmax,nd)

    if isempty(coder.target) && nd==1

        ksubs = vpi.GridVectors{1};
        ksubs = ksubs(ksubs>=kmin & ksubs<=kmax);
        vpi.GridVectors = {ksubs};

    else

        ksubs = vpi((vpi(:,1)>=kmin & vpi(:,1)<=kmax),:);
        vpi = ksubs;

    end

end

% Extract the LS estimates in the current PRGs on the current receive
% antenna, symbol and port which will be used to create VPs. For a group of
% PRGs that will be interpolated together, LS estimates of all these PRGs
% will be extracted at the same time and organized into multiple pages
function Hvp = extractVPs(vpi,nd,siz,n,p,H,policy,prgStarts)

    vpIndices = calculateVPIndices(nd,vpi,siz,n,p,policy,prgStarts);
    Hvp = H(vpIndices);

    if policy
        nPRG = numel(prgStarts);
        Hvp = nr5g.internal.nrChannelEstimate.foldMultiplePRG(Hvp,nPRG);
    end
    
end

% Extract the LS estimates in the current PRGs on the current receive
% antenna, symbol and port which will be used to create VPs in the gap
% region. For a group of PRGs that will be interpolated together, LS
% estimates of all these PRGs will be extracted at the same time and
% organized into multiple pages
function Hvps = extractGapVPs(nds,vpis,siz,n,p,H,policy,prgStarts)

    G = numel(vpis);
    Hvps = coder.nullcopy(cell([1 G]));

    for g = 1:G

        ind = calculateVPIndices(nds(g),vpis{g},siz,n,p,policy,prgStarts);
        Htemp = H(ind);
        if policy
            nPRG = numel(prgStarts);
            Htemp = nr5g.internal.nrChannelEstimate.foldMultiplePRG(Htemp,nPRG);
        end
        Hvps{g} = Htemp;
    end

end

% Calculate indices for VPs in the channel estimate across all receive
% antennas, given number of dimensions 'nd', VP interpolator 'vpi', channel
% estimate size vector 'siz' and port 'p'. For nd=0 or 1, 'n' additionally
% specifies the OFDM symbol index
function ind = calculateVPIndices(nd,vpi,siz,n,p,policy,prgStarts)

    K = siz(1); % number of subcarriers
    N = siz(2); % number of symbols
    R = siz(3); % number of Rx antenna

    if (nd==2)
        % 2-D interpolation: both subcarrier and symbol subscripts were
        % recorded

        % Extract both subscripts
        points = vpi;

    elseif (nd==1)
        % 1-D interpolation: only subcarrier subscripts were recorded

        % Extract subcarrier subscripts
        if (isempty(coder.target))
            points = vpi.GridVectors{1};
        else
            points = vpi;
        end

        % Append symbol subscripts (current symbol)
        points = [points repmat(n,size(points))];

    else % nd==0
        % '0-D interpolation': single subscript was recorded

        % Extract subcarrier subscripts and append symbol subscripts
        % (current symbol)
        points = [vpi n];

    end

    % Expand to multiple PRGs if multiple PRGs with the same points are
    % being processed
    if policy
        points = nr5g.internal.nrChannelEstimate.expandSubcarrierSubscripts(points,prgStarts,1);
    end

    % Convert subscripts to indices for single Rx antenna
    ind = points(:,1) + (points(:,2)-1)*K;

    % Expand across all Rx antennas and shift to current port
    ind = ind + (0:R-1)*K*N + (p-1)*K*N*R;
    
end

% Create virtual pilots 'vps' using interpolant 'vpi'. 'H0' contains the
% values from which the virtual pilots are created. Virtual pilots are
% created for subcarrier subscripts 'k' and OFDM symbol subscript 'n'.
% 'nd' is the number of dimensions over which the interpolation is
% performed
function vps = createVPs(nd,vpi,k,n,H0,varargin)

    if (nd==0)
        % Repeat the single estimate 'H0' for all subscripts 'k'
        vps = repmat(H0,[size(k) 1]);
    else % nd==1 or nd==2
        % Perform interpolation
        vps = nr5g.internal.nrChannelEstimate.polarInterpolate(nd,vpi,k,n,H0,'VP',varargin{:});
    end
    
end

% Create virtual pilots for gaps in the RB allocation
function [kvps,vps] = createGapVPs(nds,vpis,RBs,n,H0s,r,ke,eK,kvpin,varargin)
    
    % For each interpolator
    G = numel(vpis);
    kvps = [];
    vps = [];
    if numel(RBs)<2
        return;
    end
    for g = 1:G
        
        % Select the interpolator
        nd = nds(g);
        vpi = vpis{g};
        H0 = H0s{g};
        
        % If the interpolator is for a "high edge"
        % i.e. the gap is above RBs(g)
        if (mod(g,2))
            
            % Calculate the endpoint indices for the gap and its midpoint
            kgap = [((RBs(g)+1)*12 + 1) (RBs(g+1)*12)];
            mid = mean(kgap);
            
            % VP locations 'k' are every six subcarriers between 'mn' and 
            % 'mx'. 'mn' is either the first subcarrier of the gap or after
            % the last reference signal subcarrier in the gap, whichever
            % is higher. 'mx' is either the midpoint of the gap or eK/2
            % subcarriers into the gap, whichever is lower, and is not
            % allowed to be higher than the last reference signal
            % subcarrier overall as there are dedicated VPs for that
            % region
            mn = kgap(1);
            keingap = ke(ke >= kgap(1) & ke <= kgap(2));
            if (~isempty(keingap))
                mn = max(mn,keingap(end) + 1);
            end
            mx = min([mid (kgap(1) + eK/2) ke(end)]);
            k = (mn:6:mx).';
            
        else % the interpolator is for a "low edge" 
             % i.e. the gap is below RBs(g)
             
            % Calculate the endpoint indices for the gap and its midpoint
            kgap = [((RBs(g-1)+1)*12 + 1) (RBs(g)*12)];
            mid = mean(kgap);
            
            % VP locations 'k' are every six subcarriers between 'mx' and
            % 'mn'. 'mx' is either the last subcarrier of the gap or before
            % the first reference signal subcarrier in the gap, whichever
            % is lower. 'mn' is either the midpoint of the gap or eK/2
            % subcarriers into the gap, whichever is higher, and is not
            % allowed to be lower than the first reference signal
            % subcarrier overall as there are dedicated VPs for that
            % region
            mx = kgap(2);
            keingap = ke(ke >= kgap(1) & ke <= kgap(2));
            if (~isempty(keingap))
                mx = min(mx,keingap(1) - 1);
            end
            mn = max([mid (kgap(2) - eK/2) ke(1)]);
            k = flipud((mx:-6:mn).');
            
        end
        
        % Create and record the virtual pilots for this antenna. Do not
        % create any element already in the lower edge or upper edge
        % virtual pilots
        kx = k(~any(k == kvpin.',2));
        v = createVPs(nd,vpi,kx,n,H0(:,r),varargin{:});
        kvps = [kvps; kx]; %#ok<AGROW>
        vps = [vps; v]; %#ok<AGROW>
        
    end
    
end

% Get indices of contiguous sets of RBs in 'RBs', which have maximum value
% 'NRB'
function rbidx = contiguousRBs(RBs,NRB)

    d = [-2; RBs; NRB+1];
    d = find(diff(d)~=1);
    rbidx = [d(1:end-1) d(2:end)-1].';
    
end

% Gets k,n subscripts for K-by-N-by-P grid given indices 'ind' and port
% 'port'. 'thisport' is a logical indexing vector for the port 'port', used
% to extract the corresponding reference symbols
function [ksubs,nsubs,thisport] = getPortSubscripts(K,N,P,ind,port)

    [ksubs,nsubs,psubs] = ind2sub([K N P],ind(:));

    thisport = (psubs==port);

    ksubs = ksubs(thisport);
    nsubs = nsubs(thisport);

end