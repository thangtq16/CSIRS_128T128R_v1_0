%% diag_r3.m — Kiểm chứng nguyên nhân R3 NMSE xấu
%
%  Chạy SAU khi file chính CSIRS_128T128R_v1_0.m đã chạy xong.
%  (KHÔNG clear workspace giữa 2 lần chạy)

fprintf('\n========================================\n');
fprintf(' DIAGNOSTIC: Root cause of R3 NMSE gap\n');
fprintf('========================================\n\n');

gridSizePerRes = [nSubcarriers, nSymPerSlot, nPortsPerRes];
AWGN_theory    = N0^2 * ofdmInfo.Nfft;
fprintf('AWGN theory (freq domain): %.2e\n\n', AWGN_theory);

% -----------------------------------------------------------------------
%  TEST 1: Noise tại pilot REs — slot1-ref vs slot0-ref
%
%  residual = rxGrid_slot1(k,l,:) - sum_ports( H_ref(k,l,:,p) * pilot(p) )
%
%  Nếu slot0_ref cho residual NHỎ HƠN slot1_ref → H_actual_slot1 sai
%  Nếu slot0_ref ≈ slot1_ref                    → rxGrid_slot1 thực sự noisier
% -----------------------------------------------------------------------
fprintf('--- Test 1: Noise residual with slot1-ref vs slot0-ref ---\n');
fprintf('  Res    slot1-ref           slot0-ref\n');

for resIdx = 3:4
    portStart = (resIdx-1)*nPortsPerRes + 1;
    csirsInd  = allCsirsInd{resIdx};
    csirsSym  = allCsirsSym{resIdx};
    [subI, symI, portI] = ind2sub(gridSizePerRes, csirsInd);
    uniquePos = unique([subI, symI], 'rows');

    noise_s1 = 0;  noise_s0 = 0;  npts = 0;
    for i = 1:size(uniquePos, 1)
        k  = uniquePos(i,1);  l = uniquePos(i,2);
        rx = squeeze(rxGrid_slot1(k, l, :));        % [nRx×1]

        mask = (subI == k) & (symI == l);
        exp1 = zeros(nRxAntennas, 1);
        exp0 = zeros(nRxAntennas, 1);
        for j = find(mask)'
            pIdx = portStart + portI(j) - 1;
            x    = csirsSym(j);
            exp1 = exp1 + squeeze(H_actual_slot1(k, l, :, pIdx)) * x;
            exp0 = exp0 + squeeze(H_actual_slot0(k, l, :, pIdx)) * x;
        end
        noise_s1 = noise_s1 + mean(abs(rx - exp1).^2);
        noise_s0 = noise_s0 + mean(abs(rx - exp0).^2);
        npts = npts + 1;
    end
    noise_s1 = noise_s1 / npts;
    noise_s0 = noise_s0 / npts;
    fprintf('  R%d     %.2e (%.1fx AWGN)   %.2e (%.1fx AWGN)\n', ...
        resIdx-1, noise_s1, noise_s1/AWGN_theory, ...
                  noise_s0, noise_s0/AWGN_theory);
end

% -----------------------------------------------------------------------
%  TEST 2: Noise theo từng OFDM symbol trong slot1
%  So sánh noise tại sym 2-5 (R2) vs sym 8-11 (R3)
% -----------------------------------------------------------------------
fprintf('\n--- Test 2: Per-symbol noise within slot1 (slot1-ref) ---\n');

for resIdx = 3:4
    portStart = (resIdx-1)*nPortsPerRes + 1;
    csirsInd  = allCsirsInd{resIdx};
    csirsSym  = allCsirsSym{resIdx};
    [subI, symI, portI] = ind2sub(gridSizePerRes, csirsInd);

    syms_used = unique(symI)';
    fprintf('  R%d (l0=%d): ', resIdx-1, l0_values(resIdx));
    for l = syms_used
        uniqueSub = unique(subI(symI == l));
        n_l = 0;  npts_l = 0;
        for k = uniqueSub'
            mask = (subI == k) & (symI == l);
            rx   = squeeze(rxGrid_slot1(k, l, :));
            exp1 = zeros(nRxAntennas, 1);
            for j = find(mask)'
                pIdx = portStart + portI(j) - 1;
                exp1 = exp1 + squeeze(H_actual_slot1(k, l, :, pIdx)) * csirsSym(j);
            end
            n_l    = n_l    + mean(abs(rx - exp1).^2);
            npts_l = npts_l + 1;
        end
        fprintf('sym%02d=%.1e  ', l, n_l/npts_l);
    end
    fprintf('\n');
end

fprintf('\n--- Cách đọc kết quả ---\n');
fprintf('Test1: slot0_ref << slot1_ref → H_actual_slot1 bị sai (reference lỗi)\n');
fprintf('       slot0_ref ≈ slot1_ref  → rxGrid_slot1 thực sự noisy hơn ở R3\n\n');
fprintf('Test2: noise[sym8-11] >> noise[sym2-5] → OFDM symbol 8-11 bị degraded\n');
fprintf('       noise đồng đều ở tất cả syms    → noise không phụ thuộc vị trí\n');
