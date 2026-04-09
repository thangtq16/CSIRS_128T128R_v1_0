# 3.6 MÔ TẢ CÔNG NGHỆ AI XỬ LÝ SCHEDULING MAC (thangtq23 + TUANPA44)

## 3.6.1 Mục 1: Thangtq23 mô tả matlab / Tuan mô tả phần tích hợp

---

### 3.6.1.1 Tổng quan kiến trúc mô phỏng DRL Scheduling

Phần này mô tả môi trường mô phỏng cấp hệ thống phục vụ huấn luyện và đánh giá DRL agent trong lớp MAC scheduler. Hệ thống được xây dựng trên nền tảng MATLAB 5G Toolbox hỗ trợ Release 19 với cấu hình 128 port CSI-RS đã trình bày tại mục 3.4, đảm bảo tính nhất quán giữa môi trường mô phỏng CSI feedback và môi trường huấn luyện DRL scheduler.

Mục tiêu cụ thể của môi trường mô phỏng DRL:

**(1)** Tích hợp DRL agent vào vòng lặp lập lịch MAC thông qua giao tiếp TCP socket real-time, trong đó MATLAB đóng vai trò môi trường (environment) và Python đóng vai trò agent.

**(2)** Cung cấp đầy đủ observation space bao gồm các UE context: Throughput trung bình của các UE, Rank UE, Allocated RBG theo Layer, UE buffer status, MCS UE, Beam group i1, Subband CQI, Max correlation theo PMI — theo đúng định nghĩa mục 3.4.1.7.

**(3)** Hỗ trợ giao thức huấn luyện layer-by-layer (từng spatial layer một TTI) cho phép agent học chính sách MU-MIMO scheduling đa layer một cách tuần tự và có nhân quả.

Kiến trúc mô phỏng gồm ba thành phần chính:

```
┌─────────────────────────────────────────────────────────┐
│                   MATLAB Environment                     │
│  MU_MIMO.m                                              │
│  ├── nrGNB (128T128R, 4.9GHz, 100MHz, TDD, Rel-19)    │
│  ├── nrUE × N  (CDL-D channel, FTP Model 3 traffic)   │
│  └── nrDRLScheduler (extends nrScheduler)              │
│       ├── buildTrainingFeatureMatrix()  → obs space     │
│       ├── computeMaxCorrSubband()       → PMI corr      │
│       └── scheduleWithTrainingProtocol()→ TCP protocol  │
└──────────────────────┬──────────────────────────────────┘
                       │ TCP Socket (JSON, port 6666)
                       │ TTI_START / LAYER_ACT / TTI_DONE
┌──────────────────────▼──────────────────────────────────┐
│                Python DRL Agent                          │
│  MatlabEnvAdapter                                        │
│  ├── begin_tti()   → nhận TTI_START, build obs/mask     │
│  ├── layer_iter()  → yield LayerContext per layer        │
│  ├── apply_layer_actions() → cập nhật allocation         │
│  ├── compute_layer_transitions() → tính reward          │
│  └── finish_tti()  → gửi LAYER_ACT, nhận TTI_DONE      │
└─────────────────────────────────────────────────────────┘
```

---

### 3.6.1.2 Cấu hình hệ thống mô phỏng

#### Cấu hình gNB mô phỏng DRL

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| Tần số sóng mang | 4.9 GHz (băng n79) | |
| Băng thông kênh | 100 MHz | |
| Số Resource Block | 273 | SCS 30 kHz |
| Duplex Mode | TDD | |
| Số anten phát (T) | 128 | Hệ thống hỗ trợ đầy đủ 128T128R |
| Cấu hình CSI-RS | 4 NZP-CSI-RS resource × 32 port | Rel-19, tương thích 128 port (xem mục 3.4.1.4) |
| CSI-RS Period | [10, 0] slots | |
| SRS Periodicity | 5 slots | |

#### Cấu hình UE mô phỏng DRL

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| Số UE | 32 | |
| Anten thu mỗi UE | 1R | |
| Mô hình traffic | FTP Model 3 | Luôn có data để gửi |
| Packet size | 1500 bytes | |
| Data rate | 6 Mbps/UE | |
| Noise Figure | 7–9 dB | Random per UE |
| Bán kính phân bố | 100–2000 m | Random uniform |

#### Cấu hình MU-MIMO và Scheduler

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| MaxNumUsersPaired | 4 | Số UE ghép cặp tối đa trên một RBG |
| MaxNumLayers | 4 | Số spatial layer tối đa / TTI |
| ResourceAllocationType | 0 (RBG-based) | TS 38.214; mỗi RBG = 16 RB |
| Số RBG | 18 | ⌈273 / 16⌉ |
| PFSWindowSize | 20 slots | EMA window cho throughput trung bình |
| SemiOrthogonalityFactor | 0.4 | Ngưỡng ghép cặp MU-MIMO theo PMI |
| EnablePairingConstraints | false | Cho DRL tự học pairing constraint |
| RVSequence | [0] | Tắt HARQ retransmission (RV0 only) |
| CSI source | CSI-RS | SRS bổ sung giai đoạn tiếp theo |

#### Cấu hình kênh truyền

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| Profile | CDL-D | LOS dominant, K-factor ~13 dB |
| Delay Spread | 450 ns | Urban Macro |
| Max Doppler | 136 Hz | 30 km/h @ 4.9 GHz |
| Hướng anten gNB | Azimuth 60° | |

Lý do chọn CDL-D: Triển khai gNodeB 128T128R tầm cao tại vùng đô thị — phần lớn UE có điều kiện LOS hoặc near-LOS. Kịch bản này phù hợp nhất để đánh giá hiệu năng DRL scheduler trong điều kiện kênh thực tế (cùng lý do tại mục 3.4.1.3).

#### Cấu hình Link Adaptation (OLLA)

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| InitialOffset | 10 | Khởi tạo conservative |
| StepUp | 0.2 | Tăng MCS chậm khi ACK |
| StepDown | 0.1 | Giảm MCS nhanh khi NACK |
| Target BLER | ~10% | StepUp/StepDown ratio = 2 |

OLLA (Outer Loop Link Adaptation) điều chỉnh MCS bằng cách cộng/trừ `MCSOffset` vào giá trị `selectMCSIndexDL` tính từ CQI. Effective MCS được tính: `effectiveMCS = clamp(rawMCS − MCSOffset, 0, 28)`. Giá trị MCS sau OLLA được gửi sang Python trong trường `curr_mcs` của `TTI_START` để observation space phản ánh đúng trạng thái link adaptation thực tế.

---

### 3.6.1.3 Thiết kế DRL Scheduler (nrDRLScheduler)

`nrDRLScheduler` là một MATLAB classdef kế thừa (extends) lớp `nrScheduler` của MATLAB 5G Toolbox. Phương thức `scheduleNewTransmissionsDL` được override để inject DRL-based UE selection vào vòng lặp lập lịch MAC, trong khi giữ nguyên toàn bộ logic HARQ, Link Adaptation (OLLA), và precoding matrix selection của MATLAB.

**Hai chế độ hoạt động:**

**Chế độ 1 — Training Mode** (`TrainingMode = true`): Được kích hoạt cho mục đích huấn luyện DRL agent. Trong mỗi TTI, scheduler thực hiện giao thức layer-by-layer với Python agent qua TCP socket (chi tiết tại mục 3.6.1.5).

**Chế độ 2 — Inference Mode** (`TrainingMode = false`, `EnableDRL = true`): DRL action được set ngoài vòng lặp (thông qua `setDRLAction()`), sau đó `runSchedulingStrategyDRL_RAT0` thực hiện mapping action → RNTI → frequency allocation.

**Luồng xử lý trong scheduleNewTransmissionsDL:**

```
scheduleNewTransmissionsDL()
├── Kiểm tra CSI-RS slot → return rỗng nếu là CSI-RS slot
├── [Training Mode] → scheduleWithTrainingProtocol()
│   ├── buildTrainingFeatureMatrix() → obs space
│   ├── computeMaxCorrSubband()      → PMI correlation
│   ├── drlSendJSON(TTI_START)       → gửi cho Python
│   ├── drlRecvJSON(LAYER_ACT)       ← nhận từ Python
│   ├── map Python index → RNTI
│   ├── computeTrainingEvalMetrics() → metrics
│   ├── drlSendJSON(TTI_DONE)        → gửi cho Python
│   └── scheduleNewTransmissionsDL_DRL() → MATLAB grant
└── [Inference Mode] → scheduleNewTransmissionsDL_DRL()
    └── runSchedulingStrategyDRL_RAT0()
        ├── Xử lý DRL action matrix [NumLayers × NumRBGs]
        ├── Action masking: no-duplicate, MaxUsersPerRBG
        ├── PMI i1 matching constraint (nếu EnablePairingConstraints)
        ├── Precoder orthogonality check
        └── MCS backoff cho MU-MIMO (MU_MCSBackoff)
```

---

### 3.6.1.4 Observation Space — UE Context cho DRL Agent

Observation space được xây dựng trong hàm `buildTrainingFeatureMatrix()` (phía MATLAB) và `_build_obs()` (phía Python adapter). Hai phía tính toán độc lập trên cùng tập dữ liệu nhận từ `TTI_START` để đảm bảo nhất quán.

**Tổng chiều của observation vector:**

> obs_dim = (5 + 2 × N_RBG) × U_max

Với N_RBG = 18 RBG và U_max = 32 UE: **obs_dim = (5 + 2 × 18) × 32 = 41 × 32 = 1312**

**Chi tiết từng thành phần:**

| STT | Feature | Ký hiệu | Dim | Nguồn MATLAB | Chuẩn hóa |
|---|---|---|---|---|---|
| 1 | Throughput trung bình UE (EMA) | R̄_u | [U] | `ActualTputEMA` từ `UE.MAC.ReceivedBytes` | / max_rate (1100 Mbps) |
| 2 | Rank UE | r_u | [U] | `CSIMeasurementDL.CSIRS.RI` | / max_ue_rank (= 2) |
| 3 | RBG đã cấp theo layer | d_u | [U] | `_alloc[:layer, :]` per TTI | / N_RBG |
| 4 | Buffer status DL | b_u | [U] | `UEContext.BufferStatusDL` | / max(b) |
| 5 | MCS wideband | m_u | [U] | `selectMCSIndexDL()` − OLLA offset | / 28 |
| 6 | Subband CQI → MCS | m_{u,k} | [U × N_RBG] | `CSIMeasurementDL.CSIRS.CQI` resample → N_RBG | / 28 |
| 7 | Max cross-correlation PMI | κ_{u,k} | [U × N_RBG] | `computeMaxCorrSubband(W_i, W_j)` | ∈ [0, 1] |

#### Trích xuất Subband CQI từ MATLAB CSI measurement

`decodeCSIForTraining()` trích xuất CQI subband từ `csiMeasurement.CSIRS.CQI`. Trong trường hợp số subband CQI của MATLAB không khớp với N_RBG, thực hiện resample nearest-neighbour:

```matlab
xi  = linspace(1, numel(cqi_raw), numSubbandsFeat);
sbCQI_feat = interp1(1:numel(cqi_raw), double(cqi_raw), xi, 'nearest');
```

#### Tính toán Max Cross-Correlation PMI (Beam group i1 + Precoder W)

`computeMaxCorrSubband(W_i, W_j)` tính tương quan cực đại giữa precoder của hai UE i và j trên mỗi RBG m:

> κ(i, j, m) = max_{sb} |w_{i,sb}^H · w_{j,sb}| / (||w_{i,sb}|| · ||w_{j,sb}||)

Trong đó w_{u,sb} là cột precoder tương ứng RBG m (hoặc PRG chứa RBG m) của UE u. Giá trị κ = 0 là hoàn toàn trực giao (tốt cho MU-MIMO pairing); κ = 1 là hoàn toàn tương quan (không thể ghép cặp). Ma trận κ ∈ R^{U × U × N_RBG} được gửi tới Python trong trường `max_cross_corr` của `TTI_START`.

Lưu ý: Precoder W được lấy từ cùng pipeline CSI-RS Rel-19 đã mô tả tại mục 3.4.1.5 (Refined Type I Mode B hoặc eTypeII). Điều này đảm bảo tính nhất quán giữa CSI feedback và DRL observation space.

#### Throughput trung bình UE — ActualTputEMA

`ActualTputEMA` cập nhật mỗi TTI từ delta `UE.MAC.ReceivedBytes` (ground-truth từ MATLAB MAC layer):

> ActualTputEMA(u) ← β · ActualTputEMA(u) + (1−β) · ΔRxBytes(u) × 8 / (T_slot × 10^6)  [Mbps]

Với β = 0.9 (EMA window ~10 TTI) và T_slot là slot duration (ms) theo SCS 30 kHz = 0.5 ms.

---

### 3.6.1.5 Giao thức huấn luyện layer-by-layer (Training Protocol)

Giao thức được thiết kế để Python agent có thể học chính sách lập lịch MU-MIMO theo từng spatial layer trong một TTI. MATLAB đóng vai trò server TCP (listening trên port 6666); Python adapter kết nối và điều khiển vòng lặp huấn luyện.

**Luồng giao tiếp mỗi TTI:**

```
MATLAB                              Python (MatlabEnvAdapter)
  │                                       │
  │── TTI_START ──────────────────────►  │  begin_tti()
  │   {type, tti, n_layers, n_rbg,       │  ├── cập nhật avg_tp, buf, rank
  │    buf[U], avg_tp[U], ue_rank[U],    │  ├── tính MCS subband (CQI+jitter)
  │    wb_cqi[U], curr_mcs[U],           │  └── chọn selected_ues
  │    sub_cqi[U×M], eligible_ues,       │
  │    max_cross_corr[U×U×M],            │  layer_iter() → LayerContext
  │    occupied_rbgs[M]}                 │  ├── _build_obs(layer)   → obs[obs_dim]
  │                                      │  └── _build_masks(layer) → masks[M, A]
  │                                      │
  │                                      │  agent.act(obs, masks) per layer
  │                                      │  apply_layer_actions() → _alloc[L,M]
  │                                      │  compute_layer_transitions() → reward
  │                                      │
  │◄── LAYER_ACT ──────────────────────  │  finish_tti()
  │    {type, actions[L×M]}              │  └── gửi LAYER_ACT
  │    (0-based: 0..U-1=UE, U=noop)     │
  │                                      │
  │── TTI_DONE ──────────────────────►  │  nhận TTI_DONE → log metrics
  │   {type, metrics:                    │
  │    {avg_cell_tput, jain,             │
  │     pf_utility, no_schedule_rate}}   │
```

**Quy tắc indexing action:**

Để đảm bảo tính nhất quán giữa MATLAB (1-based RNTI) và Python (0-based local index):

| Phía | Giá trị | Ý nghĩa |
|---|---|---|
| Python → MATLAB | val ∈ {0, …, U−1} | Local UE index trong selected_ues |
| Python → MATLAB | val = U | No-op (không cấp phát RBG này) |
| MATLAB mapping | rnti = val + 1 | 1-based RNTI |
| MATLAB → drlAction | ue_pos = find(eligibleUEs == rnti) | 1-based index trong eligibleUEs |

**Xử lý CSI-RS slot:**

Khi MATLAB phát hiện TTI hiện tại là CSI-RS pilot slot (theo `csirsCfg.CSIRSPeriod`), `scheduleNewTransmissionsDL` trả về assignment rỗng và không gửi `TTI_START` cho Python. Điều này tránh DRL scheduling trên các slot dành riêng cho pilot transmission.

---

### 3.6.1.6 Action Space và Action Masking

Action space tại mỗi RBG m và layer l là:

> A = {0, 1, …, U_max−1, U_max (noop)},   |A| = U_max + 1 = 33

Action masking loại bỏ các action không hợp lệ trước khi đưa vào softmax của policy network:

| Constraint | Điều kiện mask = False (invalid) |
|---|---|
| Buffer constraint | UE có buf = 0 (không có data) |
| Rank constraint | Số layer đã cấp cho UE tại RBG m ≥ ue_rank[u] |
| Continuity constraint | UE đã từng xuất hiện ở RBG m layer trước nhưng KHÔNG xuất hiện ở layer ngay trước (layer l−1) |
| No-op | Luôn valid (mask = True) |

Rank constraint đảm bảo không vượt quá khả năng spatial multiplexing của UE. Continuity constraint đảm bảo tính liên tục precoder giữa các layer trên cùng RBG — phù hợp với yêu cầu MU-MIMO precoding của TS 38.214.

---

### 3.6.1.7 Hàm Reward

Reward được thiết kế theo tiêu chí Proportional Fair (PF) chuẩn hóa, tính trên từng RBG m sau mỗi quyết định layer l.

**Bước 1 — Tính throughput tập hợp UE trên RBG m:**

Với tập UE đã cấp S_m trên RBG m (gồm các layer trước đó):

> T(S_m) = (1 − max_{i≠j ∈ S_m} κ_{i,j,m}) / |S_m| × Σ_{u ∈ S_m} R_u^inst(m)

Trong đó R_u^inst(m) là throughput tức thời ước tính từ MCS subband theo TBS 38.214, κ_{i,j,m} là PMI cross-correlation. Hệ số penalty (1 − κ_max) / |S_m| phản ánh mức độ giảm throughput do interference MU-MIMO.

**Bước 2 — Raw reward theo PF:**

> r_raw(u, m) = [T(S_m^prev ∪ {u}) − T(S_m^prev)] / R̄_u

**Bước 3 — Chuẩn hóa và clamp:**

> r(u*, m) = clamp(r_raw(u*, m) / max_u r_raw(u, m), −1, +1)

Trong đó u* là action của DRL tại RBG m. Nếu action là noop và tất cả UE đều làm giảm throughput (max_u r_raw < 0), reward = +1 (noop là optimal). Thiết kế này chuẩn hóa reward về [−1, +1] bất kể scale throughput tuyệt đối, giúp ổn định quá trình huấn luyện.

---

### 3.6.1.8 Metrics đánh giá (TTI_DONE)

Sau mỗi TTI, MATLAB tính và gửi các metrics sau cho Python:

| Metric | Ký hiệu | Mô tả |
|---|---|---|
| avg_cell_tput | T̄_cell | Throughput trung bình cell (Mbps) |
| jain | J | Jain's Fairness Index: (Σ_u R̄_u)² / (U × Σ_u R̄_u²) |
| pf_utility | PF | Σ_u log(R̄_u + ε) |
| no_schedule_rate | — | Tỷ lệ RBG không được cấp phát |

Các metrics này được sử dụng để theo dõi hội tụ của DRL agent trong quá trình huấn luyện. Mục tiêu là tối đa hóa đồng thời `avg_cell_tput` và `jain`, phản ánh trade-off giữa throughput tổng cell và fairness giữa các UE.
