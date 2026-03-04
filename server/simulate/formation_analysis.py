"""
Formation Maintenance Analysis Tool
====================================
Phân tích đầy đủ các yếu tố trong việc duy trì đội hình của hệ thống multi-robot

Các yếu tố phân tích:
1. Khoảng cách giữa các robot (Inter-robot distance)
2. Sai số đội hình (Formation error)
3. Đồng bộ thời gian (Time synchronization)
4. Tốc độ và gia tốc (Velocity and acceleration)
5. Heading/Orientation consistency
6. Latency analysis
7. Position tracking quality
8. Formation stability metrics
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime
import argparse
import sys

# Thêm path để import config nếu cần
sys.path.insert(0, str(Path(__file__).parent))

try:
    from config import DT, TRIGGER_THRESHOLD
except ImportError:
    DT = 0.05
    TRIGGER_THRESHOLD = 0.05


class FormationAnalyzer:
    """Phân tích dữ liệu đội hình từ log file"""
    
    def __init__(self, log_file: str):
        """
        Khởi tạo analyzer với đường dẫn file log
        
        Args:
            log_file: Đường dẫn tới file CSV log
        """
        self.log_file = Path(log_file)
        self.df = None
        self.robot_ids = []
        self.analysis_results = {}
        
    def load_data(self) -> pd.DataFrame:
        """Load và xử lý dữ liệu từ file CSV"""
        print(f"📂 Loading data from: {self.log_file}")
        
        self.df = pd.read_csv(self.log_file)
        
        # Chuyển đổi thời gian
        self.df['server_timestamp'] = pd.to_numeric(self.df['server_timestamp'])
        self.df['robot_timestamp'] = pd.to_numeric(self.df['robot_timestamp'])
        
        # Lấy danh sách robot
        self.robot_ids = self.df['robot_id'].unique().tolist()
        
        # Tính thời gian relative
        start_time = self.df['server_timestamp'].min()
        self.df['relative_time'] = self.df['server_timestamp'] - start_time
        
        # Tính latency (độ trễ giữa robot và server)
        self.df['latency'] = self.df['server_timestamp'] - self.df['robot_timestamp']
        
        print(f"✅ Loaded {len(self.df)} records for {len(self.robot_ids)} robots: {self.robot_ids}")
        print(f"⏱️  Duration: {self.df['relative_time'].max():.2f} seconds")
        
        return self.df
    
    def synchronize_robot_data(self) -> pd.DataFrame:
        """
        Đồng bộ hóa dữ liệu giữa các robot theo thời gian
        Sử dụng interpolation để có dữ liệu cùng timestamp
        """
        if len(self.robot_ids) < 2:
            print("⚠️  Cần ít nhất 2 robot để phân tích đội hình")
            return None
            
        print("\n🔄 Synchronizing robot data...")
        
        # Tạo common time grid
        t_min = self.df['server_timestamp'].min()
        t_max = self.df['server_timestamp'].max()
        
        # Sample rate khoảng 50ms
        common_times = np.arange(t_min, t_max, 0.05)
        
        synced_data = {'time': common_times, 'relative_time': common_times - t_min}
        
        for robot_id in self.robot_ids:
            robot_df = self.df[self.df['robot_id'] == robot_id].copy()
            robot_df = robot_df.sort_values('server_timestamp')
            
            # Interpolate các giá trị
            for col in ['x', 'y', 'vx', 'vy', 'theta']:
                synced_data[f'{robot_id}_{col}'] = np.interp(
                    common_times, 
                    robot_df['server_timestamp'].values, 
                    robot_df[col].values
                )
        
        self.synced_df = pd.DataFrame(synced_data)
        print(f"✅ Synchronized {len(self.synced_df)} time points")
        
        return self.synced_df
    
    def analyze_inter_robot_distance(self) -> dict:
        """
        Phân tích khoảng cách giữa các robot
        - Khoảng cách trung bình
        - Độ lệch chuẩn
        - Min/Max
        - Biến thiên theo thời gian
        """
        print("\n📏 Analyzing inter-robot distances...")
        
        if not hasattr(self, 'synced_df') or self.synced_df is None:
            self.synchronize_robot_data()
        
        if len(self.robot_ids) < 2:
            return {}
        
        distances = {}
        
        # Tính khoảng cách giữa từng cặp robot
        for i, robot1 in enumerate(self.robot_ids):
            for robot2 in self.robot_ids[i+1:]:
                pair_name = f"{robot1}-{robot2}"
                
                dx = self.synced_df[f'{robot1}_x'] - self.synced_df[f'{robot2}_x']
                dy = self.synced_df[f'{robot1}_y'] - self.synced_df[f'{robot2}_y']
                dist = np.sqrt(dx**2 + dy**2)
                
                distances[pair_name] = {
                    'distance_array': dist.values,
                    'mean': dist.mean(),
                    'std': dist.std(),
                    'min': dist.min(),
                    'max': dist.max(),
                    'cv': dist.std() / dist.mean() * 100,  # Coefficient of variation
                    'range': dist.max() - dist.min()
                }
                
                print(f"  {pair_name}: mean={dist.mean():.4f}m, std={dist.std():.4f}m, CV={distances[pair_name]['cv']:.2f}%")
        
        self.analysis_results['inter_robot_distance'] = distances
        return distances
    
    def analyze_formation_error(self, desired_distance: float = None) -> dict:
        """
        Phân tích sai số đội hình so với khoảng cách mong muốn
        
        Args:
            desired_distance: Khoảng cách mong muốn giữa các robot (nếu None thì dùng khoảng cách ban đầu)
        """
        print("\n📐 Analyzing formation error...")
        
        if 'inter_robot_distance' not in self.analysis_results:
            self.analyze_inter_robot_distance()
        
        formation_error = {}
        
        for pair_name, dist_info in self.analysis_results['inter_robot_distance'].items():
            dist_array = dist_info['distance_array']
            
            # Lấy tên 2 robot từ pair_name
            robot1, robot2 = pair_name.split('-')
            
            # Tính delta X và Y giữa 2 robot theo thời gian
            dx_array = self.synced_df[f'{robot1}_x'].values - self.synced_df[f'{robot2}_x'].values
            dy_array = self.synced_df[f'{robot1}_y'].values - self.synced_df[f'{robot2}_y'].values
            
            # Target dx và dy ban đầu (lock position)
            target_dx = dx_array[0]
            target_dy = dy_array[0]
            
            # Tính error theo từng thành phần
            error_x = np.abs(dx_array - target_dx)
            error_y = np.abs(dy_array - target_dy)
            
            # Nếu không có desired distance, dùng khoảng cách ban đầu (initial lock distance)
            if desired_distance is None:
                target_dist = dist_array[0]  # Khoảng cách đầu tiên là lock distance trong leader-follower
            else:
                target_dist = desired_distance
            
            error = np.abs(dist_array - target_dist)
            
            # Tính contribution của X và Y error vào total distance error
            # error^2 ≈ error_x^2 + error_y^2 (approximation)
            total_squared_error = error_x**2 + error_y**2
            x_contribution = np.mean(error_x**2) / (np.mean(error_x**2) + np.mean(error_y**2)) * 100 if (np.mean(error_x**2) + np.mean(error_y**2)) > 0 else 0
            y_contribution = np.mean(error_y**2) / (np.mean(error_x**2) + np.mean(error_y**2)) * 100 if (np.mean(error_x**2) + np.mean(error_y**2)) > 0 else 0
            
            formation_error[pair_name] = {
                'target_distance': target_dist,
                'error_array': error,
                'mae': np.mean(error),  # Mean Absolute Error
                'rmse': np.sqrt(np.mean(error**2)),  # Root Mean Square Error
                'max_error': np.max(error),
                'error_std': np.std(error),
                'within_threshold': np.mean(error < TRIGGER_THRESHOLD) * 100,  # % thời gian trong threshold
                # Thêm phân tích theo thành phần X và Y
                'target_dx': target_dx,
                'target_dy': target_dy,
                'dx_array': dx_array,
                'dy_array': dy_array,
                'error_x_array': error_x,
                'error_y_array': error_y,
                'error_x_mae': np.mean(error_x),
                'error_y_mae': np.mean(error_y),
                'error_x_rmse': np.sqrt(np.mean(error_x**2)),
                'error_y_rmse': np.sqrt(np.mean(error_y**2)),
                'error_x_max': np.max(error_x),
                'error_y_max': np.max(error_y),
                'x_contribution': x_contribution,  # % contribution of X error
                'y_contribution': y_contribution   # % contribution of Y error
            }
            
            print(f"  {pair_name}: MAE={formation_error[pair_name]['mae']*100:.2f}cm, "
                  f"RMSE={formation_error[pair_name]['rmse']*100:.2f}cm, "
                  f"Within threshold: {formation_error[pair_name]['within_threshold']:.1f}%")
            print(f"    X-error: MAE={formation_error[pair_name]['error_x_mae']*100:.2f}cm, "
                  f"contribution={formation_error[pair_name]['x_contribution']:.1f}%")
            print(f"    Y-error: MAE={formation_error[pair_name]['error_y_mae']*100:.2f}cm, "
                  f"contribution={formation_error[pair_name]['y_contribution']:.1f}%")
        
        self.analysis_results['formation_error'] = formation_error
        return formation_error
    
    def analyze_time_synchronization(self) -> dict:
        """Phân tích chất lượng đồng bộ thời gian"""
        print("\n⏰ Analyzing time synchronization...")
        
        sync_analysis = {}
        
        for robot_id in self.robot_ids:
            robot_df = self.df[self.df['robot_id'] == robot_id]
            latency = robot_df['latency'].values * 1000  # Convert to ms
            
            # Tính sampling rate
            timestamps = robot_df['server_timestamp'].values
            dt = np.diff(timestamps)
            
            sync_analysis[robot_id] = {
                'latency_mean_ms': np.mean(latency),
                'latency_std_ms': np.std(latency),
                'latency_min_ms': np.min(latency),
                'latency_max_ms': np.max(latency),
                'sampling_rate_mean_hz': 1 / np.mean(dt) if len(dt) > 0 else 0,
                'sampling_rate_std_hz': 1 / np.std(dt) if len(dt) > 0 and np.std(dt) > 0 else 0,
                'jitter_ms': np.std(dt) * 1000 if len(dt) > 0 else 0
            }
            
            print(f"  {robot_id}: latency={sync_analysis[robot_id]['latency_mean_ms']:.2f}±{sync_analysis[robot_id]['latency_std_ms']:.2f}ms, "
                  f"rate={sync_analysis[robot_id]['sampling_rate_mean_hz']:.1f}Hz")
        
        self.analysis_results['time_sync'] = sync_analysis
        return sync_analysis
    
    def analyze_velocity_profile(self) -> dict:
        """Phân tích profile vận tốc của các robot"""
        print("\n🚀 Analyzing velocity profiles...")
        
        velocity_analysis = {}
        
        for robot_id in self.robot_ids:
            robot_df = self.df[self.df['robot_id'] == robot_id].copy()
            
            # Tính tốc độ tổng
            speed = np.sqrt(robot_df['vx']**2 + robot_df['vy']**2)
            
            # Tính gia tốc từ đạo hàm vận tốc
            dt = np.diff(robot_df['server_timestamp'].values)
            dt[dt == 0] = 0.001  # Tránh chia cho 0
            
            ax = np.diff(robot_df['vx'].values) / dt
            ay = np.diff(robot_df['vy'].values) / dt
            
            # Tính từ vị trí để so sánh
            x = robot_df['x'].values
            y = robot_df['y'].values
            ts = robot_df['server_timestamp'].values
            
            velocity_analysis[robot_id] = {
                'speed_mean': speed.mean(),
                'speed_max': speed.max(),
                'speed_std': speed.std(),
                'accel_mean': np.sqrt(ax**2 + ay**2).mean() if len(ax) > 0 else 0,
                'accel_max': np.sqrt(ax**2 + ay**2).max() if len(ax) > 0 else 0,
                'total_distance': np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2)),
                'start_pos': (x[0], y[0]),
                'end_pos': (x[-1], y[-1]),
                'displacement': np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)
            }
            
            print(f"  {robot_id}: speed={velocity_analysis[robot_id]['speed_mean']:.4f}m/s, "
                  f"distance={velocity_analysis[robot_id]['total_distance']:.2f}m")
        
        self.analysis_results['velocity'] = velocity_analysis
        return velocity_analysis
    
    def analyze_heading_consistency(self) -> dict:
        """Phân tích độ ổn định của heading (theta)"""
        print("\n🧭 Analyzing heading consistency...")
        
        heading_analysis = {}
        
        for robot_id in self.robot_ids:
            robot_df = self.df[self.df['robot_id'] == robot_id]
            theta = robot_df['theta'].values
            
            # Unwrap theta để tránh discontinuity tại ±π
            theta_unwrapped = np.unwrap(theta)
            
            # Tính tốc độ quay
            dt = np.diff(robot_df['server_timestamp'].values)
            dt[dt == 0] = 0.001
            omega = np.diff(theta_unwrapped) / dt
            
            heading_analysis[robot_id] = {
                'theta_mean': np.mean(theta),
                'theta_std': np.std(theta_unwrapped),
                'theta_range': np.max(theta_unwrapped) - np.min(theta_unwrapped),
                'omega_mean': np.mean(np.abs(omega)) if len(omega) > 0 else 0,
                'omega_max': np.max(np.abs(omega)) if len(omega) > 0 else 0,
                'total_rotation': np.abs(theta_unwrapped[-1] - theta_unwrapped[0]) if len(theta_unwrapped) > 0 else 0
            }
            
            print(f"  {robot_id}: theta_mean={np.degrees(heading_analysis[robot_id]['theta_mean']):.1f}°, "
                  f"rotation={np.degrees(heading_analysis[robot_id]['total_rotation']):.1f}°")
        
        self.analysis_results['heading'] = heading_analysis
        return heading_analysis
    
    def analyze_formation_stability(self) -> dict:
        """
        Phân tích độ ổn định của đội hình theo thời gian
        - Phát hiện các giai đoạn không ổn định
        - Thời gian hội tụ
        - Các oscillation
        """
        print("\n📊 Analyzing formation stability...")
        
        if 'inter_robot_distance' not in self.analysis_results:
            self.analyze_inter_robot_distance()
        
        stability_analysis = {}
        
        for pair_name, dist_info in self.analysis_results['inter_robot_distance'].items():
            dist_array = dist_info['distance_array']
            
            # Tính moving average và variance
            window_size = min(20, len(dist_array) // 10)
            if window_size < 2:
                window_size = 2
            
            moving_avg = np.convolve(dist_array, np.ones(window_size)/window_size, mode='valid')
            
            # Tính variance trong từng window
            moving_var = np.array([
                np.var(dist_array[max(0, i-window_size):i+1]) 
                for i in range(len(dist_array))
            ])
            
            # Phát hiện giai đoạn không ổn định (variance cao)
            variance_threshold = np.percentile(moving_var, 90)
            unstable_periods = moving_var > variance_threshold
            
            # Phát hiện oscillation bằng FFT
            if len(dist_array) > 10:
                fft_result = np.abs(np.fft.fft(dist_array - np.mean(dist_array)))
                freqs = np.fft.fftfreq(len(dist_array), d=0.05)
                
                # Lấy tần số chủ đạo (bỏ DC component)
                positive_freqs = freqs[1:len(freqs)//2]
                positive_fft = fft_result[1:len(fft_result)//2]
                
                if len(positive_fft) > 0:
                    dominant_freq_idx = np.argmax(positive_fft)
                    dominant_freq = positive_freqs[dominant_freq_idx]
                    oscillation_magnitude = positive_fft[dominant_freq_idx] / len(dist_array)
                else:
                    dominant_freq = 0
                    oscillation_magnitude = 0
            else:
                dominant_freq = 0
                oscillation_magnitude = 0
            
            stability_analysis[pair_name] = {
                'variance_mean': np.mean(moving_var),
                'variance_max': np.max(moving_var),
                'unstable_percentage': np.mean(unstable_periods) * 100,
                'dominant_oscillation_freq': dominant_freq,
                'oscillation_magnitude': oscillation_magnitude,
                'stability_index': 1 / (1 + np.std(dist_array) * 10)  # Higher is more stable
            }
            
            print(f"  {pair_name}: stability_index={stability_analysis[pair_name]['stability_index']:.3f}, "
                  f"unstable={stability_analysis[pair_name]['unstable_percentage']:.1f}%")
        
        self.analysis_results['stability'] = stability_analysis
        return stability_analysis
    
    def analyze_relative_motion(self) -> dict:
        """Phân tích chuyển động tương đối giữa các robot"""
        print("\n🔀 Analyzing relative motion between robots...")
        
        if not hasattr(self, 'synced_df') or self.synced_df is None:
            self.synchronize_robot_data()
        
        if len(self.robot_ids) < 2:
            return {}
        
        relative_motion = {}
        
        for i, robot1 in enumerate(self.robot_ids):
            for robot2 in self.robot_ids[i+1:]:
                pair_name = f"{robot1}-{robot2}"
                
                # Relative position
                rel_x = self.synced_df[f'{robot1}_x'] - self.synced_df[f'{robot2}_x']
                rel_y = self.synced_df[f'{robot1}_y'] - self.synced_df[f'{robot2}_y']
                
                # Relative velocity
                rel_vx = self.synced_df[f'{robot1}_vx'] - self.synced_df[f'{robot2}_vx']
                rel_vy = self.synced_df[f'{robot1}_vy'] - self.synced_df[f'{robot2}_vy']
                rel_speed = np.sqrt(rel_vx**2 + rel_vy**2)
                
                # Heading difference
                theta1 = self.synced_df[f'{robot1}_theta'].values
                theta2 = self.synced_df[f'{robot2}_theta'].values
                heading_diff = np.abs(np.arctan2(np.sin(theta1 - theta2), np.cos(theta1 - theta2)))
                
                # Bearing (góc từ robot1 nhìn robot2)
                bearing = np.arctan2(rel_y, rel_x)
                
                relative_motion[pair_name] = {
                    'rel_speed_mean': rel_speed.mean(),
                    'rel_speed_max': rel_speed.max(),
                    'rel_speed_std': rel_speed.std(),
                    'heading_diff_mean': np.degrees(np.mean(heading_diff)),
                    'heading_diff_max': np.degrees(np.max(heading_diff)),
                    'bearing_mean': np.degrees(np.mean(bearing)),
                    'bearing_std': np.degrees(np.std(bearing)),
                    'coordination_index': 1 / (1 + rel_speed.mean() * 10)  # Higher = better coordinated
                }
                
                print(f"  {pair_name}: rel_speed={relative_motion[pair_name]['rel_speed_mean']:.4f}m/s, "
                      f"coordination={relative_motion[pair_name]['coordination_index']:.3f}")
        
        self.analysis_results['relative_motion'] = relative_motion
        return relative_motion
    
    def generate_comprehensive_report(self) -> str:
        """Tạo báo cáo tổng hợp"""
        print("\n" + "="*60)
        print("📋 COMPREHENSIVE FORMATION ANALYSIS REPORT")
        print("="*60)
        
        report = []
        report.append("# BÁO CÁO PHÂN TÍCH ĐỘI HÌNH ROBOT")
        report.append(f"\n**File:** {self.log_file.name}")
        report.append(f"**Thời gian phân tích:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"**Số robot:** {len(self.robot_ids)}")
        report.append(f"**Thời lượng:** {self.df['relative_time'].max():.2f} giây")
        report.append(f"**Số mẫu:** {len(self.df)}")
        
        # 1. Khoảng cách giữa các robot
        if 'inter_robot_distance' in self.analysis_results:
            report.append("\n## 1. Khoảng cách giữa các robot")
            for pair, data in self.analysis_results['inter_robot_distance'].items():
                report.append(f"\n### {pair}")
                report.append(f"- Trung bình: {data['mean']*100:.2f} cm")
                report.append(f"- Độ lệch chuẩn: {data['std']*100:.2f} cm")
                report.append(f"- Hệ số biến thiên (CV): {data['cv']:.2f}%")
                report.append(f"- Min/Max: {data['min']*100:.2f} - {data['max']*100:.2f} cm")
        
        # 2. Sai số đội hình
        if 'formation_error' in self.analysis_results:
            report.append("\n## 2. Sai số đội hình")
            for pair, data in self.analysis_results['formation_error'].items():
                report.append(f"\n### {pair}")
                report.append(f"- Khoảng cách mục tiêu: {data['target_distance']*100:.2f} cm")
                report.append(f"- MAE: {data['mae']*100:.2f} cm")
                report.append(f"- RMSE: {data['rmse']*100:.2f} cm")
                report.append(f"- Sai số tối đa: {data['max_error']*100:.2f} cm")
                report.append(f"- Thời gian trong threshold ({TRIGGER_THRESHOLD*100:.0f}cm): {data['within_threshold']:.1f}%")
                
                # Thêm phân tích X/Y error nếu có
                if 'error_x_mae' in data:
                    report.append(f"\n#### Phân tích theo thành phần:")
                    report.append(f"- **Target ΔX**: {data['target_dx']*100:.2f} cm")
                    report.append(f"- **Target ΔY**: {data['target_dy']*100:.2f} cm")
                    report.append(f"- **X-error**: MAE={data['error_x_mae']*100:.2f}cm, RMSE={data['error_x_rmse']*100:.2f}cm, Max={data['error_x_max']*100:.2f}cm")
                    report.append(f"- **Y-error**: MAE={data['error_y_mae']*100:.2f}cm, RMSE={data['error_y_rmse']*100:.2f}cm, Max={data['error_y_max']*100:.2f}cm")
                    report.append(f"- **X contribution**: {data['x_contribution']:.1f}%")
                    report.append(f"- **Y contribution**: {data['y_contribution']:.1f}%")
                    
                    # Xác định nguồn lỗi chính
                    main_source = "X-axis" if data['x_contribution'] > data['y_contribution'] else "Y-axis"
                    report.append(f"- **Nguồn lỗi chính**: {main_source}")
        
        # 3. Đồng bộ thời gian
        if 'time_sync' in self.analysis_results:
            report.append("\n## 3. Đồng bộ thời gian")
            for robot, data in self.analysis_results['time_sync'].items():
                report.append(f"\n### {robot}")
                report.append(f"- Latency: {data['latency_mean_ms']:.2f} ± {data['latency_std_ms']:.2f} ms")
                report.append(f"- Sampling rate: {data['sampling_rate_mean_hz']:.1f} Hz")
                report.append(f"- Jitter: {data['jitter_ms']:.2f} ms")
        
        # 4. Vận tốc
        if 'velocity' in self.analysis_results:
            report.append("\n## 4. Profile vận tốc")
            for robot, data in self.analysis_results['velocity'].items():
                report.append(f"\n### {robot}")
                report.append(f"- Tốc độ trung bình: {data['speed_mean']*100:.2f} cm/s")
                report.append(f"- Tốc độ tối đa: {data['speed_max']*100:.2f} cm/s")
                report.append(f"- Quãng đường: {data['total_distance']*100:.2f} cm")
                report.append(f"- Displacement: {data['displacement']*100:.2f} cm")
        
        # 5. Heading
        if 'heading' in self.analysis_results:
            report.append("\n## 5. Phân tích Heading")
            for robot, data in self.analysis_results['heading'].items():
                report.append(f"\n### {robot}")
                report.append(f"- Theta trung bình: {np.degrees(data['theta_mean']):.2f}°")
                report.append(f"- Độ lệch chuẩn: {np.degrees(data['theta_std']):.2f}°")
                report.append(f"- Tổng góc quay: {np.degrees(data['total_rotation']):.2f}°")
        
        # 6. Ổn định đội hình
        if 'stability' in self.analysis_results:
            report.append("\n## 6. Độ ổn định đội hình")
            for pair, data in self.analysis_results['stability'].items():
                report.append(f"\n### {pair}")
                report.append(f"- Chỉ số ổn định: {data['stability_index']:.3f}")
                report.append(f"- Tỷ lệ không ổn định: {data['unstable_percentage']:.1f}%")
                report.append(f"- Tần số dao động chính: {data['dominant_oscillation_freq']:.2f} Hz")
        
        # 7. Chuyển động tương đối
        if 'relative_motion' in self.analysis_results:
            report.append("\n## 7. Chuyển động tương đối")
            for pair, data in self.analysis_results['relative_motion'].items():
                report.append(f"\n### {pair}")
                report.append(f"- Tốc độ tương đối: {data['rel_speed_mean']*100:.2f} cm/s")
                report.append(f"- Chỉ số phối hợp: {data['coordination_index']:.3f}")
                report.append(f"- Chênh lệch heading: {data['heading_diff_mean']:.2f}°")
        
        # Summary
        report.append("\n## 8. Tóm tắt")
        
        # Tính điểm tổng thể
        overall_score = 0
        factors = 0
        
        if 'formation_error' in self.analysis_results:
            for pair, data in self.analysis_results['formation_error'].items():
                overall_score += data['within_threshold']
                factors += 1
        
        if 'stability' in self.analysis_results:
            for pair, data in self.analysis_results['stability'].items():
                overall_score += data['stability_index'] * 100
                factors += 1
        
        if 'relative_motion' in self.analysis_results:
            for pair, data in self.analysis_results['relative_motion'].items():
                overall_score += data['coordination_index'] * 100
                factors += 1
        
        if factors > 0:
            overall_score /= factors
            report.append(f"\n**Điểm đánh giá tổng thể: {overall_score:.1f}/100**")
            
            if overall_score >= 80:
                report.append("**Đánh giá: ĐỘI HÌNH XUẤT SẮC** ✅")
            elif overall_score >= 60:
                report.append("**Đánh giá: ĐỘI HÌNH TỐT** 👍")
            elif overall_score >= 40:
                report.append("**Đánh giá: ĐỘI HÌNH CẦN CẢI THIỆN** ⚠️")
            else:
                report.append("**Đánh giá: ĐỘI HÌNH CẦN XEM XÉT LẠI** ❌")
        
        return "\n".join(report)
    
    def plot_analysis(self, save_path: str = None):
        """
        Tạo các biểu đồ phân tích - Chia thành 2 figure:
        - Figure 1 (Main): Formation analysis chính
        - Figure 2 (Detailed): Metrics chi tiết
        """
        print("\n📈 Generating visualization plots...")
        
        if not hasattr(self, 'synced_df') or self.synced_df is None:
            self.synchronize_robot_data()
        
        colors = ['blue', 'red', 'green', 'purple', 'orange']
        
        # ============================================================
        # FIGURE 1: MAIN FORMATION ANALYSIS (2x3 = 6 plots)
        # ============================================================
        fig1 = plt.figure(figsize=(18, 12))
        fig1.suptitle('Formation Analysis - Main Metrics', fontsize=16, fontweight='bold')
        
        # 1. Trajectory plot
        ax1 = fig1.add_subplot(2, 3, 1)
        for i, robot_id in enumerate(self.robot_ids):
            robot_df = self.df[self.df['robot_id'] == robot_id]
            x_vals = robot_df['x'].values
            y_vals = robot_df['y'].values
            ax1.plot(x_vals, y_vals, color=colors[i % len(colors)], 
                    label=robot_id, alpha=0.7)
            ax1.scatter(x_vals[0], y_vals[0], 
                       color=colors[i % len(colors)], marker='o', s=100, edgecolors='black')
            ax1.scatter(x_vals[-1], y_vals[-1], 
                       color=colors[i % len(colors)], marker='s', s=100, edgecolors='black')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Robot Trajectories')
        ax1.legend()
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        
        # 2. Inter-robot distance
        ax2 = fig1.add_subplot(2, 3, 2)
        if 'inter_robot_distance' in self.analysis_results:
            for pair_name, data in self.analysis_results['inter_robot_distance'].items():
                time_arr = self.synced_df['relative_time'].values[:len(data['distance_array'])]
                ax2.plot(time_arr, data['distance_array'] * 100, label=pair_name)
                # Vẽ đường target distance (initial lock distance)
                initial_dist = data['distance_array'][0]
                ax2.axhline(y=initial_dist * 100, linestyle='--', alpha=0.5, label=f'{pair_name} target')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Distance (cm)')
        ax2.set_title('Inter-Robot Distance')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. Formation Error Over Time
        ax3 = fig1.add_subplot(2, 3, 3)
        if 'formation_error' in self.analysis_results:
            for pair_name, data in self.analysis_results['formation_error'].items():
                time_arr = self.synced_df['relative_time'].values[:len(data['error_array'])]
                ax3.plot(time_arr, data['error_array'] * 100, label=pair_name, linewidth=2)
            ax3.axhline(y=TRIGGER_THRESHOLD * 100, color='red', linestyle='--', 
                       label=f'Threshold ({TRIGGER_THRESHOLD*100:.0f}cm)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Formation Error (cm)')
        ax3.set_title('Formation Error Over Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. X-axis Formation Error
        ax4 = fig1.add_subplot(2, 3, 4)
        if 'formation_error' in self.analysis_results:
            for pair_name, data in self.analysis_results['formation_error'].items():
                if 'error_x_array' in data:
                    time_arr = self.synced_df['relative_time'].values[:len(data['error_x_array'])]
                    ax4.plot(time_arr, data['error_x_array'] * 100, label=f'{pair_name}', linewidth=2)
            ax4.axhline(y=TRIGGER_THRESHOLD * 100, color='red', linestyle='--', alpha=0.3)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('X-axis Error (cm)')
        ax4.set_title('Formation Error - X Component')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. Y-axis Formation Error
        ax5 = fig1.add_subplot(2, 3, 5)
        if 'formation_error' in self.analysis_results:
            for pair_name, data in self.analysis_results['formation_error'].items():
                if 'error_y_array' in data:
                    time_arr = self.synced_df['relative_time'].values[:len(data['error_y_array'])]
                    ax5.plot(time_arr, data['error_y_array'] * 100, label=f'{pair_name}', linewidth=2)
            ax5.axhline(y=TRIGGER_THRESHOLD * 100, color='red', linestyle='--', alpha=0.3)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Y-axis Error (cm)')
        ax5.set_title('Formation Error - Y Component')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. Error Contribution Pie Chart
        ax6 = fig1.add_subplot(2, 3, 6)
        if 'formation_error' in self.analysis_results:
            for pair_name, data in self.analysis_results['formation_error'].items():
                if 'x_contribution' in data and 'y_contribution' in data:
                    contributions = [data['x_contribution'], data['y_contribution']]
                    labels = [f'X Error\\n({data["x_contribution"]:.1f}%)', 
                             f'Y Error\\n({data["y_contribution"]:.1f}%)']
                    colors_pie = ['#ff9999', '#66b3ff']
                    ax6.pie(contributions, labels=labels, colors=colors_pie, autopct='%1.1f%%', 
                           startangle=90, textprops={'fontsize': 11})
                    ax6.set_title(f'Error Source Contribution\\n{pair_name}', fontsize=12)
        
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        
        # ============================================================
        # FIGURE 2: DETAILED METRICS (1x3 = 3 plots)
        # ============================================================
        fig2 = plt.figure(figsize=(18, 6))
        fig2.suptitle('Formation Analysis - Detailed Metrics', fontsize=16, fontweight='bold')
        
        # 1. Velocity profiles
        ax1_fig2 = fig2.add_subplot(1, 3, 1)
        # 1. Velocity profiles
        ax1_fig2 = fig2.add_subplot(1, 3, 1)
        for i, robot_id in enumerate(self.robot_ids):
            robot_df = self.df[self.df['robot_id'] == robot_id]
            speed = np.sqrt(robot_df['vx'].values**2 + robot_df['vy'].values**2) * 100
            time_rel = robot_df['server_timestamp'].values - self.df['server_timestamp'].min()
            ax1_fig2.plot(time_rel, speed, color=colors[i % len(colors)], label=robot_id, alpha=0.7, linewidth=1.5)
        ax1_fig2.set_xlabel('Time (s)')
        ax1_fig2.set_ylabel('Speed (cm/s)')
        ax1_fig2.set_title('Velocity Profiles')
        ax1_fig2.legend()
        ax1_fig2.grid(True, alpha=0.3)
        
        # 2. Communication Latency
        ax2_fig2 = fig2.add_subplot(1, 3, 2)
        for i, robot_id in enumerate(self.robot_ids):
            robot_df = self.df[self.df['robot_id'] == robot_id]
            time_rel = robot_df['server_timestamp'].values - self.df['server_timestamp'].min()
            ax2_fig2.plot(time_rel, robot_df['latency'].values * 1000, color=colors[i % len(colors)], 
                    label=robot_id, alpha=0.7, linewidth=1.5)
        ax2_fig2.set_xlabel('Time (s)')
        ax2_fig2.set_ylabel('Latency (ms)')
        ax2_fig2.set_title('Communication Latency')
        ax2_fig2.legend()
        ax2_fig2.grid(True, alpha=0.3)
        
        # 3. Robot Heading
        ax3_fig2 = fig2.add_subplot(1, 3, 3)
        for i, robot_id in enumerate(self.robot_ids):
            robot_df = self.df[self.df['robot_id'] == robot_id]
            time_rel = robot_df['server_timestamp'].values - self.df['server_timestamp'].min()
            theta_deg = np.degrees(robot_df['theta'].values)
            # Lọc outlier
            theta_median = np.median(theta_deg)
            theta_std = np.std(theta_deg)
            valid_mask = np.abs(theta_deg - theta_median) < 3 * theta_std
            ax3_fig2.plot(time_rel[valid_mask], theta_deg[valid_mask], color=colors[i % len(colors)], 
                    label=robot_id, alpha=0.7, linewidth=1.5)
        ax3_fig2.set_xlabel('Time (s)')
        ax3_fig2.set_ylabel('Heading (degrees)')
        ax3_fig2.set_title('Robot Heading')
        ax3_fig2.legend()
        ax3_fig2.grid(True, alpha=0.3)
        ax3_fig2.set_ylim(-185, 5)
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        
        # Save figures
        if save_path:
            # Tạo tên file cho 2 figure
            if save_path.endswith('.png'):
                base_path = save_path[:-4]
            else:
                base_path = save_path
            
            main_path = f"{base_path}_main.png"
            detailed_path = f"{base_path}_detailed.png"
            
            fig1.savefig(main_path, dpi=150, bbox_inches='tight')
            print(f"✅ Main plot saved to: {main_path}")
            
            fig2.savefig(detailed_path, dpi=150, bbox_inches='tight')
            print(f"✅ Detailed plot saved to: {detailed_path}")
        
        return fig1, fig2
    
    def run_full_analysis(self, plot_save_path: str = None) -> dict:
        """Chạy phân tích đầy đủ"""
        print("\n" + "="*60)
        print("🚀 STARTING COMPREHENSIVE FORMATION ANALYSIS")
        print("="*60)
        
        # Load data
        self.load_data()
        
        # Synchronize data
        self.synchronize_robot_data()
        
        # Run all analyses
        self.analyze_inter_robot_distance()
        self.analyze_formation_error()
        self.analyze_time_synchronization()
        self.analyze_velocity_profile()
        self.analyze_heading_consistency()
        self.analyze_formation_stability()
        self.analyze_relative_motion()
        
        # Generate report
        report = self.generate_comprehensive_report()
        print(report)
        
        # Generate plots - giờ trả về 2 figures
        if plot_save_path:
            fig1, fig2 = self.plot_analysis(plot_save_path)
        else:
            fig1, fig2 = self.plot_analysis()
        
        # Lưu references để figures không bị garbage collected
        self.fig_main = fig1
        self.fig_detailed = fig2
        
        return self.analysis_results


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Phân tích đội hình robot từ log file')
    parser.add_argument('--log-file', '-l', type=str, 
                       help='Đường dẫn tới file log CSV')
    parser.add_argument('--output-dir', '-o', type=str, default=None,
                       help='Thư mục lưu kết quả')
    parser.add_argument('--no-plot', action='store_true',
                       help='Không hiển thị biểu đồ')
    
    args = parser.parse_args()
    
    # Tìm file log mới nhất nếu không chỉ định
    if args.log_file is None:
        log_dir = Path(__file__).parent.parent / 'logs'
        sync_logs = list(log_dir.glob('sync_position_log_*.csv'))
        if not sync_logs:
            print("❌ Không tìm thấy file log sync_position_log_*.csv trong thư mục logs/")
            return
        
        # Lấy file mới nhất
        args.log_file = str(max(sync_logs, key=lambda p: p.stat().st_mtime))
        print(f"📂 Using latest log file: {args.log_file}")
    
    # Tạo analyzer
    analyzer = FormationAnalyzer(args.log_file)
    
    # Xác định path lưu plot
    if args.output_dir:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        plot_path = str(output_dir / 'formation_analysis_plot.png')
    else:
        log_path = Path(args.log_file)
        plot_path = str(log_path.parent / f'{log_path.stem}_analysis.png')
    
    # Chạy phân tích
    results = analyzer.run_full_analysis(plot_save_path=plot_path if not args.no_plot else None)
    
    # Lưu báo cáo
    report = analyzer.generate_comprehensive_report()
    report_path = plot_path.replace('.png', '_report.md')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report)
    print(f"✅ Report saved to: {report_path}")
    
    if not args.no_plot:
        plt.show()
    
    return results


if __name__ == '__main__':
    main()
