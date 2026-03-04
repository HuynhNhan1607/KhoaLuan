# BÁO CÁO PHÂN TÍCH ĐỘI HÌNH ROBOT

**File:** sync_position_log_20260202_150605.csv
**Thời gian phân tích:** 2026-02-03 11:15:13
**Số robot:** 2
**Thời lượng:** 58.77 giây
**Số mẫu:** 2335

## 1. Khoảng cách giữa các robot

### robot1-robot2
- Trung bình: 60.87 cm
- Độ lệch chuẩn: 1.33 cm
- Hệ số biến thiên (CV): 2.19%
- Min/Max: 57.79 - 64.83 cm

## 2. Sai số đội hình

### robot1-robot2
- Khoảng cách mục tiêu: 62.21 cm
- MAE: 1.65 cm
- RMSE: 1.89 cm
- Sai số tối đa: 4.42 cm
- Thời gian trong threshold (5cm): 100.0%

#### Phân tích theo thành phần:
- **Target ΔX**: 4.07 cm
- **Target ΔY**: 62.07 cm
- **X-error**: MAE=0.97cm, RMSE=1.36cm, Max=3.67cm
- **Y-error**: MAE=1.63cm, RMSE=1.86cm, Max=4.33cm
- **X contribution**: 34.7%
- **Y contribution**: 65.3%
- **Nguồn lỗi chính**: Y-axis

## 3. Đồng bộ thời gian

### robot1
- Latency: 17.82 ± 17.85 ms
- Sampling rate: 20.0 Hz
- Jitter: 19.43 ms

### robot2
- Latency: 16.62 ± 15.83 ms
- Sampling rate: 20.0 Hz
- Jitter: 17.14 ms

## 4. Profile vận tốc

### robot1
- Tốc độ trung bình: 4.68 cm/s
- Tốc độ tối đa: 8.49 cm/s
- Quãng đường: 288.47 cm
- Displacement: 273.23 cm

### robot2
- Tốc độ trung bình: 5.11 cm/s
- Tốc độ tối đa: 10.12 cm/s
- Quãng đường: 289.08 cm
- Displacement: 272.23 cm

## 5. Phân tích Heading

### robot1
- Theta trung bình: -170.79°
- Độ lệch chuẩn: 0.22°
- Tổng góc quay: 0.02°

### robot2
- Theta trung bình: -0.32°
- Độ lệch chuẩn: 0.28°
- Tổng góc quay: 0.06°

## 6. Độ ổn định đội hình

### robot1-robot2
- Chỉ số ổn định: 0.883
- Tỷ lệ không ổn định: 10.0%
- Tần số dao động chính: 0.07 Hz

## 7. Chuyển động tương đối

### robot1-robot2
- Tốc độ tương đối: 1.23 cm/s
- Chỉ số phối hợp: 0.890
- Chênh lệch heading: 179.08°

## 8. Tóm tắt

**Điểm đánh giá tổng thể: 92.4/100**
**Đánh giá: ĐỘI HÌNH XUẤT SẮC** ✅