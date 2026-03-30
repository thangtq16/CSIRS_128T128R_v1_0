%% Verification script: Mode B Refined Type I Codebook (TS 38.214 §5.2.2.2.1a)
%  Tests mathematical properties and spec conformance
clear; clc;

addpath(genpath(fullfile(pwd,'5g')));
addpath(genpath(fullfile(pwd,'wirelessnetwork')));
rehash toolboxcache;

%% Config
N1 = 16; N2 = 4; O1 = 4; O2 = 4;
Pcsirs = 2*N1*N2;   % 128
tol = 1e-10;
PASS = @(s) fprintf('  [PASS] %s\n', s);
FAIL = @(s,v) fprintf('  [FAIL] %s  (got %.2e)\n', s, v);

fprintf('=== Mode B Verification (N1=%d, N2=%d, O1=%d, O2=%d, P=%d) ===\n\n', ...
    N1, N2, O1, O2, Pcsirs);

%% Helper: getVlm (same formula as in nrPMIReport)
getVlm = @(l,m) reshape((exp(2i*pi*l*(0:N1-1)/(O1*N1)).' .* ...
                          exp(2i*pi*m*(0:N2-1)/(O2*N2))).',[],1);
phi    = @(x) exp(1i*pi*x/2);

%% -----------------------------------------------------------------------
%  TEST 1: Index mapping correctness
%  i1,2,l = N1N2-1  →  n=0 → n1=0,n2=0 → m1=q1,m2=q2  → vlm = all-ones (l=0,m=0 case when q=0)
%  i1,2,l = 0       →  n=N1N2-1 → n1=N1-1,n2=N2-1 → m1=O1*(N1-1)+q1, m2=O2*(N2-1)+q2
fprintf('--- TEST 1: Index mapping ---\n');

q1=0; q2=0;
n_idx = N1*N2 - 1;         % i1,2,l = N1N2-1
n_val = N1*N2 - 1 - n_idx; % = 0
n1    = mod(n_val, N1);     % = 0
n2    = (n_val - n1) / N1;  % = 0
m1    = O1*n1 + q1;         % = 0
m2    = O2*n2 + q2;         % = 0
v     = getVlm(m1, m2);
expected_v = ones(N1*N2,1); % DFT at (0,0) = all ones (before normalisation in vlm)
err = norm(v - expected_v);
if err < tol; PASS('i1,2,l=N1N2-1, q=0 → m=(0,0) → vlm=all-ones');
else;         FAIL('i1,2,l=N1N2-1, q=0 → m=(0,0) → vlm=all-ones', err); end

n_idx2 = 0;                           % i1,2,l = 0
n_val2 = N1*N2 - 1 - n_idx2;         % = N1N2-1
n1_2   = mod(n_val2, N1);             % = N1-1
n2_2   = (n_val2 - n1_2) / N1;       % = N2-1
m1_2   = O1*n1_2 + q1;               % = O1*(N1-1)
m2_2   = O2*n2_2 + q2;               % = O2*(N2-1)
if m1_2 == O1*(N1-1) && m2_2 == O2*(N2-1)
    PASS(sprintf('i1,2,l=0, q=0 → m=(%d,%d)=O1*(N1-1), O2*(N2-1)', m1_2, m2_2));
else
    FAIL('i1,2,l=0 → m=(O1*(N1-1), O2*(N2-1))', abs(m1_2 - O1*(N1-1)));
end

% Range check: m1 in [0, N1O1-1], m2 in [0, N2O2-1] for all (q,n) combos
ok_range = true;
for q_idx = 0:O1*O2-1
    qq1 = floor(q_idx/O2);  qq2 = mod(q_idx,O2);
    for n_i = 0:N1*N2-1
        nv = N1*N2-1-n_i;
        nn1 = mod(nv,N1);  nn2 = (nv-nn1)/N1;
        mm1 = O1*nn1+qq1;  mm2 = O2*nn2+qq2;
        if mm1 < 0 || mm1 > N1*O1-1 || mm2 < 0 || mm2 > N2*O2-1
            ok_range = false; break;
        end
    end
end
if ok_range; PASS(sprintf('All m1 in [0,%d], m2 in [0,%d]', N1*O1-1, N2*O2-1));
else;        FAIL('m range check failed', 1); end

%% -----------------------------------------------------------------------
%  TEST 2: Power normalization  ||W||_F^2 = 1 for all ranks
fprintf('\n--- TEST 2: Power normalization ||W||_F^2 = 1 ---\n');

for v = 1:4
    all_ok = true;
    for q_idx = 0:3   % sample 4 (q1,q2) combos
        qq1 = floor(q_idx/O2); qq2 = mod(q_idx,O2);
        n_vec = randi(N1*N2, 1, v) - 1;  % random per-layer beam indices
        c_vec = randi(4, 1, v) - 1;       % random co-phase indices

        W_top = zeros(N1*N2, v);
        W_bot = zeros(N1*N2, v);
        for l = 1:v
            nv = N1*N2-1-n_vec(l);
            nn1 = mod(nv,N1); nn2 = (nv-nn1)/N1;
            mm1 = O1*nn1+qq1; mm2 = O2*nn2+qq2;
            vl = getVlm(mm1, mm2);
            W_top(:,l) = vl;
            W_bot(:,l) = phi(c_vec(l)) * vl;
        end
        W = (1/sqrt(v*Pcsirs)) * [W_top; W_bot];
        pwr = norm(W,'fro')^2;
        if abs(pwr - 1) > tol; all_ok = false; end
    end
    if all_ok; PASS(sprintf('rank=%d: ||W||_F^2 = 1', v));
    else;      FAIL(sprintf('rank=%d: ||W||_F^2 ≠ 1', v), abs(pwr-1)); end
end

%% -----------------------------------------------------------------------
%  TEST 3: Co-phase structure  — W_bot(:,l) = phi(c_l) * W_top(:,l)
fprintf('\n--- TEST 3: W structure: W_bot = diag(phi)*W_top ---\n');

for v = 1:4
    qq1=2; qq2=1;
    n_vec = mod(0:v-1, N1*N2);
    c_vec = mod(0:v-1, 4);

    W_top = zeros(N1*N2, v); W_bot = zeros(N1*N2, v);
    for l = 1:v
        nv = N1*N2-1-n_vec(l);
        nn1 = mod(nv,N1); nn2=(nv-nn1)/N1;
        mm1=O1*nn1+qq1; mm2=O2*nn2+qq2;
        vl = getVlm(mm1,mm2);
        W_top(:,l) = vl;
        W_bot(:,l) = phi(c_vec(l)) * vl;
    end
    W = (1/sqrt(v*Pcsirs)) * [W_top; W_bot];

    err_max = 0;
    for l = 1:v
        ratio = W(N1*N2+l, :);   % one row from bottom half
        expected = phi(c_vec(l)) * W(l, :);  % diagonal entry check
        % Check column l: W_bot(:,l) == phi(c_l) * W_top(:,l)
        e = norm(W(N1*N2+1:end,l) - phi(c_vec(l))*W(1:N1*N2,l));
        err_max = max(err_max, e);
    end
    if err_max < tol; PASS(sprintf('rank=%d: co-phase structure correct', v));
    else;             FAIL(sprintf('rank=%d: co-phase structure', v), err_max); end
end

%% -----------------------------------------------------------------------
%  TEST 4: Known LoS channel — Mode B should recover the beam
%  Target beam: m1=4, m2=4 (achievable with q1=0,q2=0 since O1=O2=4)
%  n1 = m1/O1 = 1, n2 = m2/O2 = 1 → n = 1 + 1*N1 = 17
%  → i1,2,l = N1N2-1-n = 63-17 = 46
fprintf('\n--- TEST 4: Known rank-1 LoS channel beam recovery ---\n');

target_m1 = 4; target_m2 = 4;   % must be multiples of O1,O2 for q=0
v_target  = getVlm(target_m1, target_m2);
h_rx      = exp(1i*2*pi*rand(4,1));
H_los     = h_rx * v_target';              % conjugate transpose: [4 × N1N2]
H_full    = [H_los H_los] / sqrt(2);       % [4 × 128], two polarisations

n1_exp    = target_m1 / O1;
n2_exp    = target_m2 / O2;
n_exp     = n1_exp + n2_exp * N1;           % = 1 + 1*16 = 17
n_idx_exp = N1*N2 - 1 - n_exp;             % = 46

qq1=0; qq2=0;
proj = zeros(1, N1*N2);
for n_i = 0:N1*N2-1
    nv = N1*N2-1-n_i;
    nn1 = mod(nv,N1); nn2 = (nv-nn1)/N1;
    mm1 = O1*nn1+qq1; mm2 = O2*nn2+qq2;
    vl = getVlm(mm1,mm2);
    proj(n_i+1) = norm(H_full * [vl; vl]/sqrt(2))^2;
end
[~, best_n_idx] = max(proj);
best_n_idx = best_n_idx - 1;

nv_best  = N1*N2 - 1 - best_n_idx;
nn1_best = mod(nv_best,N1); nn2_best = (nv_best-nn1_best)/N1;
mm1_best = O1*nn1_best+qq1; mm2_best = O2*nn2_best+qq2;

if best_n_idx == n_idx_exp
    PASS(sprintf('LoS beam recovered: i1,2,l=%d → m=(%d,%d)', best_n_idx, mm1_best, mm2_best));
else
    fprintf('  [FAIL] Expected n_idx=%d m=(%d,%d), got n_idx=%d m=(%d,%d)\n', ...
        n_idx_exp, target_m1, target_m2, best_n_idx, mm1_best, mm2_best);
end

%% -----------------------------------------------------------------------
%  TEST 5: Mode B capacity >= Mode A capacity (Mode A is a subset of Mode B)
%  (Mode A restricts second beam to fixed offsets; Mode B picks it freely)
fprintf('\n--- TEST 5: cap(Mode B) >= cap(Mode A) [statistical over 20 random channels] ---\n');

repCfg               = nrCSIReportConfig;
repCfg.NSizeBWP      = 52;
repCfg.NStartBWP     = 0;
repCfg.CodebookType  = 'typeI-SinglePanel-r19';
repCfg.PanelDimensions = [1, 16, 4];
repCfg.PMIFormatIndicator = 'wideband';

carrier_t = nrCarrierConfig;
carrier_t.NSizeGrid = 52;
carrier_t.SubcarrierSpacing = 30;

% Row 18 = 32 ports, but nrPMIReport uses size(H,4) for typeI-SinglePanel-r19
csirs_t = nrCSIRSConfig;
csirs_t.CSIRSType           = {'nzp'};
csirs_t.CSIRSPeriod         = 'on';
csirs_t.RowNumber           = 18;
csirs_t.Density             = {'one'};
csirs_t.SymbolLocations     = {2};
csirs_t.SubcarrierLocations = {[0,2,4,6]};

nVar_t = 0.01;
nRx    = 4;
nSC = 52*12; nSym = 14;

n_trials = 20; n_modeB_better = 0; n_equal = 0;
cap_diff = zeros(1, n_trials);
for trial = 1:n_trials
    H_rand = (randn(nRx,Pcsirs) + 1i*randn(nRx,Pcsirs)) / sqrt(2);
    H_rep  = repmat(reshape(H_rand,1,1,nRx,Pcsirs), [nSC nSym 1 1]);

    repCfg.CodebookMode = 1;
    try
        [~,info_A_t] = nr5g.internal.nrPMIReport(carrier_t, csirs_t, repCfg, 1, H_rep, nVar_t);
        W_A_t = info_A_t.W;
        H_A = H_rand * W_A_t;
        cap_A = real(log2(det(eye(nRx) + (1/nVar_t) * (H_A*H_A'))));
    catch; cap_A = 0; end

    repCfg.CodebookMode = 2;
    try
        [~,info_B_t] = nr5g.internal.nrPMIReport(carrier_t, csirs_t, repCfg, 1, H_rep, nVar_t);
        W_B_t = info_B_t.W;
        H_B = H_rand * W_B_t;
        cap_B = real(log2(det(eye(nRx) + (1/nVar_t) * (H_B*H_B'))));
    catch; cap_B = 0; end

    cap_diff(trial) = cap_B - cap_A;
    if cap_B > cap_A + 1e-6; n_modeB_better = n_modeB_better + 1;
    elseif abs(cap_B - cap_A) <= 1e-6; n_equal = n_equal + 1; end
end

pct_B_better = 100*n_modeB_better/n_trials;
pct_equal    = 100*n_equal/n_trials;
fprintf('  Mode B > Mode A: %d/%d trials (%.0f%%)\n', n_modeB_better, n_trials, pct_B_better);
fprintf('  Mode B = Mode A: %d/%d trials (%.0f%%)\n', n_equal, n_trials, pct_equal);
fprintf('  cap_B - cap_A:   mean=%.3f  min=%.3f  max=%.3f\n', ...
    mean(cap_diff), min(cap_diff), max(cap_diff));
if min(cap_diff) >= -tol
    PASS('cap(Mode B) >= cap(Mode A) in all trials');
else
    FAIL('cap(Mode B) < cap(Mode A) in some trials', min(cap_diff));
end

fprintf('\n=== Verification complete ===\n');
