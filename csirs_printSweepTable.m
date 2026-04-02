function csirs_printSweepTable(results, snrList)
%CSIRS_PRINTSWEEPTABLE  Print SNR sweep comparison table.
%
%  Columns per SNR point: RI avg | MCS avg | TP avg (Mbps) | % vs B
%  Rows: B (reference), C (Mode A), D (Mode B), E (eTypeII-r19)
%
%  Inputs:
%    results  - struct array (1 x nSNR), each element has:
%                 .snr          scalar
%                 .ri  [1x4]    avg RI  (B C D E)
%                 .mcs [1x4]    avg MCS (B C D E)
%                 .tp  [1x4]    avg TP in Mbps (B C D E)
%    snrList  - [1 x nSNR] SNR values in dB

nSNR = length(snrList);
% ThangTQ23_128T128R_eTypeII_Rel19: added Approach E label
labels = {'B: SVD (ref)', 'C: Mode A   ', 'D: Mode B   ', 'E: eTypeII-r19'};

% ── Header ───────────────────────────────────────────────────────────────
fprintf('\n');
fprintf('========================================================================================\n');
fprintf('       128T128R CSI FEEDBACK -- SNR SWEEP RESULTS (avg over realizations)            \n');
fprintf('========================================================================================\n');

% Per-SNR column headers
fprintf('%-14s', 'Approach');
for i = 1:nSNR
    label = sprintf('SNR=%+ddB', snrList(i));
    switch snrList(i)
        case num2cell(-10:4);  dist = '(far)  ';
        otherwise
            if snrList(i) <= 5
                dist = '(far)  ';
            elseif snrList(i) <= 15
                dist = '(med)  ';
            else
                dist = '(near)';
            end
    end
    fprintf('  %-40s', sprintf('%s %s', label, dist));
end
fprintf('\n');

fprintf('%-14s', '');
for i = 1:nSNR
    fprintf('  %-6s  %-6s  %-10s  %-6s', 'RI', 'MCS', 'TP(Mbps)', '% vsB');
end
fprintf('\n');

sep = repmat('-', 1, 14 + nSNR*32);
fprintf('%s\n', sep);

% ── Data rows ─────────────────────────────────────────────────────────────
for appIdx = 1:4   % B=1, C=2, D=3, E=4 (ThangTQ23_128T128R_eTypeII_Rel19)
    fprintf('%-14s', labels{appIdx});
    for i = 1:nSNR
        ri_val  = results(i).ri(appIdx);
        mcs_val = results(i).mcs(appIdx);
        tp_val  = results(i).tp(appIdx);
        tp_ref  = results(i).tp(1);   % B is reference

        if appIdx == 1
            pct_str = '  ref ';
        else
            pct_str = sprintf('%5.1f%%', tp_val/tp_ref*100);
        end
        fprintf('  %-6.2f  %-6.1f  %-10.1f  %s', ri_val, mcs_val, tp_val, pct_str);
    end
    fprintf('\n');

    % Separator after reference row
    if appIdx == 1
        fprintf('%s\n', sep);
    end
end

fprintf('%s\n', sep);
fprintf('  RI: avg rank indicator  |  MCS: avg MCS index (1-15)  |  TP: throughput Mbps\n');
fprintf('  %% vsB: throughput relative to SVD upper bound\n\n');

end
