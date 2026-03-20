# 128T128R NZP-CSI-RS Channel Estimation & CSI Feedback

Tài liệu này ghi chú chi tiết về luồng mô phỏng hệ thống Massive MIMO 128T128R, tập trung vào cấu hình tín hiệu Non-Zero Power CSI-RS (NZP-CSI-RS), ước lượng kênh truyền (CE) và cơ chế phản hồi CSI (PMI/RI/CQI) sử dụng MATLAB 5G Toolbox.

## 1. Thông số hệ thống cơ bản (System Configuration)
- Băng thông (Carrier): 10 MHz
- Số lượng PRB (NSizeGrid): 52
- Khoảng cách sóng mang con (SCS): 30 kHz
- Cyclic Prefix: Normal
- Cấu hình Anten:
  - Trạm gốc (gNB): 128 Tx antennas
  - Thiết bị người dùng (UE): 4 Rx antennas
- Phân bổ CSI-RS: 128 port được chia thành 4 NZP-CSI-RS resource, mỗi resource phụ trách 32 port.

## 2. Cấu hình NZP-CSI-RS (Row 17, CDM4)
Do giới hạn tiêu chuẩn của 3GPP (tối đa 32 port cho một resource), 128 port được thiết lập thông qua 4 resource riêng biệt hoat động trên cùng một khe thời gian (slot).
- Chuẩn tham chiếu: TS 38.211 (Mục 7.4.1.5)
- Row Number: 17 (Cấu hình cho 32 port)
- Phương pháp ghép kênh (CDM Type): CDM4 (FD-CDM2, TD-CDM2) với Density 0.5.
- Vị trí Subcarrier (k_i): `[0, 2, 4, 6]` (Tương ứng với khoảng cách FD-CDM2)
- Phân bổ Symbol trong 1 slot (14 symbols):
  - Resource 0 (Ports 0-31): Symbols 2, 3
  - Resource 1 (Ports 32-63): Symbols 5, 6
  - Resource 2 (Ports 64-95): Symbols 8, 9
  - Resource 3 (Ports 96-127): Symbols 11, 12
- Sau khi khởi tạo, các symbol CSI-RS được nhân với hệ số công suất và map vào lưới tài nguyên chung (Resource Grid) 128-port dựa trên chỉ số port offset.

## 3. Mô hình kênh truyền (Channel Model)
Mô hình kênh truyền dạng không gian (Spatial Model) được cấu hình để đánh giá Massive MIMO và Beamforming.
- Loại mô hình: CDL-C (Cluster Delay Line - Urban Macro)
- Delay Spread: 100 ns
- Maximum Doppler Shift: 5 Hz (Môi trường di chuyển chậm/người đi bộ)
- Tần số sóng mang: 3.5 GHz (Băng tần n78)
- Cấu hình mảng Anten:
  - gNB Array: Kích thước `[16, 4, 2, 1, 1]` (16 ngang, 4 dọc, 2 phân cực) -> 128 phần tử. Khoảng cách phần tử: `[0.5, 0.5, 1, 1]` bước sóng.
  - UE Array: Kích thước `[2, 1, 2, 1, 1]` -> 4 phần tử.

## 4. Xử lý tín hiệu lớp vật lý (PHY Pipeline)
- Điều chế: OFDM Modulation (`nrOFDMModulate`).
- Truyền qua kênh: Tín hiệu đi qua kênh truyền CDL-C, mô phỏng thêm trễ kênh (padding) và cộng nhiễu AWGN (SNR = 20 dB).
- Đồng bộ: Ước lượng trễ thời gian (Timing Sync) sử dụng mảng tín hiệu tham chiếu từ Resource 0.
- Giải điều chế: OFDM Demodulation (`nrOFDMDemodulate`).
- Ước lượng kênh truyền (CE):
  - Thực hiện CE độc lập cho từng nhóm 32 port tương ứng với 4 resource.
  - Ma trận H ước lượng của từng resource (kích thước `[K, L, nRx, 32]`) được ghép nối lại tạo thành ma trận kênh truyền đầy đủ `H_est_full` kích thước `[K, L, nRx, 128]`.
  - Đánh giá CE: So sánh `H_est_full` với kênh truyền lý tưởng `H_actual` để tính toán sai số trung bình bình phương chuẩn hóa (NMSE) trên từng port.

## 5. Phản hồi trạng thái kênh (CSI Feedback)
Đánh giá CSI bằng hai phương pháp tiếp cận khác nhau để xử lý ma trận 128 Tx.

### Phương pháp A: Per-resource PMI (Sử dụng hàm tiêu chuẩn 32 port)
- Cấu hình: Type I Single Panel Codebook, kích thước panel `[1, 8, 2]` tương ứng 32 port. Subband Size = 4, Codebook Mode = 1.
- Xử lý: Lặp qua 4 resource (mỗi resource 32 port), sử dụng hàm custom `myCSIReport` để tính toán RI, CQI và PMI (gồm i1 và i2).
- Đặc điểm: Tận dụng trực tiếp các thư viện tiêu chuẩn của 3GPP nhưng không nắm bắt trọn vẹn đặc tính không gian của toàn mảng 128 anten.

### Phương pháp B: Full 128-port thông qua SVD (Không dùng codebook)
- Xử lý kênh: Lấy trung bình ma trận kênh truyền ước lượng trên toàn dải băng tần và thời gian để có kênh băng rộng `H_wb` kích thước `[nRx, nTx]`.
- Phân tích SVD: `[U, S, V] = svd(H_wb, 'econ')`
- Ước lượng RI: Đếm số lượng singular values vượt qua ngưỡng (10% của giá trị lớn nhất). Giới hạn tối đa bằng số lượng Rx Antennas.
- Precoding Matrix (W): Lấy các cột đầu tiên của ma trận V dựa trên giá trị RI (`V(:, 1:ri_svd)`).
- Tính CQI: Dựa trên bộ thu Zero-Forcing (ZF). Ước lượng SINR trên từng lớp (layer), sau đó ánh xạ vào bảng tiêu chuẩn CQI 1-15 của 3GPP.
- Phân tích Beamforming: Trực quan hóa phổ Azimuth cho layer đầu tiên bằng cách nhân ma trận precoding với ma trận lái (steering vector) ULA chuẩn.

## 6. Thành phần tùy chỉnh (Custom Functions)
Hệ thống sử dụng một số hàm tùy biến và ghi đè nội bộ:
- `myCSIReport.m`: Hàm bao bọc (wrapper) để thay thế `nrCSIReportCSIRS`. Bỏ qua cơ chế chặn tính năng (feature gate) của matlab, gọi trực tiếp API nội bộ `nr5g.internal.nrCQIReport` để tính toán CQI, PMI, RI bằng cách lặp qua các rank khả dụng và chọn rank tối đa hóa hiệu suất phổ (Spectral Efficiency).
