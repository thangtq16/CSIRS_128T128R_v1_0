% Kiểm tra rank thực tế của H_res từng resource (không qua codebook)
fprintf('Per-resource singular values:\n');
for resIdx = 1:nResources
    portStart = (resIdx-1)*nPortsPerRes + 1;
    portEnd   = resIdx*nPortsPerRes;
    slotNum   = slotAssign(resIdx);
    symStart  = slotNum*nSymPerSlot + 1;
    symEnd    = (slotNum+1)*nSymPerSlot;

    H_wb_res = squeeze(mean(H_est_full(:, symStart:symEnd, :, portStart:portEnd), [1 2]));
    sv = svd(H_wb_res);  % [4×32] → max rank = 4
    fprintf('  R%d: SVs = [%.2f  %.2f  %.2f  %.2f]  (ratio 3rd/1st = %.3f)\n', ...
        resIdx-1, sv(1), sv(2), sv(3), sv(4), sv(3)/sv(1));
end
