#!/usr/bin/env python3
"""
PID Auto-Tuner - Phân tích CSV từ Xavier
=========================================
Phân tích file /tmp/pid_log.csv được ghi bởi trajectory_executor trên Xavier
và tính toán PID gains tối ưu.

Cách dùng:
  1. Bật ENABLE_PID_LOGGING=1 trong sys_config.h
  2. Build và deploy lên Xavier
  3. Chạy trajectory trên Xavier
  4. Copy file /tmp/pid_log.csv về laptop
  5. Chạy: python3 pid_analyzer.py pid_log.csv

CSV format (từ trajectory_executor.c):
  time,tgt_x,tgt_y,cur_x,cur_y,err_x,err_y,cmd_vx,cmd_vy,P_x,P_y,I_x,I_y,D_x,D_y
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

class PIDAnalyzer:
    def __init__(self, csv_file):
        """Load CSV data từ Xavier."""
        self.df = pd.read_csv(csv_file)
        self.dt = None
        print(f"Loaded {len(self.df)} samples from {csv_file}")
        
    def analyze(self):
        """Phân tích và tính PID gains."""
        print("\n" + "=" * 60)
        print("PID AUTO-TUNER - Phân tích data từ Xavier")
        print("=" * 60)
        
        # 1. Sample time
        self._calculate_sample_time()
        
        # 2. Error analysis
        self._analyze_errors()
        
        # 3. PID contributions
        self._analyze_pid_contributions()
        
        # 4. Detect problems
        self._detect_problems()
        
        # 5. Calculate recommended gains
        self._calculate_recommended_gains()
        
        # 6. Plot
        self._plot()
        
    def _calculate_sample_time(self):
        """Tính sample time."""
        time = self.df['time'].values
        dt_samples = np.diff(time)
        self.dt = np.median(dt_samples)
        print(f"\n[1] Sample Time:")
        print(f"    Median dt: {self.dt*1000:.1f} ms")
        print(f"    Control rate: {1/self.dt:.1f} Hz")
        print(f"    Duration: {time[-1]:.1f} seconds")
        
    def _analyze_errors(self):
        """Phân tích error."""
        err_x = self.df['err_x'].values
        err_y = self.df['err_y'].values
        err_mag = np.sqrt(err_x**2 + err_y**2)
        
        print(f"\n[2] Position Error:")
        print(f"    Mean:  {np.mean(err_mag)*100:.2f} cm")
        print(f"    Max:   {np.max(err_mag)*100:.2f} cm")
        print(f"    Std:   {np.std(err_mag)*100:.2f} cm")
        print(f"    Final: {err_mag[-1]*100:.2f} cm")
        
        self.err_mag = err_mag
        self.typical_error = np.percentile(err_mag, 75)
        
    def _analyze_pid_contributions(self):
        """Phân tích đóng góp của P, I, D."""
        P_mag = np.sqrt(self.df['P_x']**2 + self.df['P_y']**2)
        I_mag = np.sqrt(self.df['I_x']**2 + self.df['I_y']**2)
        D_mag = np.sqrt(self.df['D_x']**2 + self.df['D_y']**2)
        
        total = P_mag + I_mag + np.abs(D_mag) + 1e-6
        
        print(f"\n[3] PID Contributions:")
        print(f"    P-term: mean={np.mean(P_mag):.4f} m/s ({np.mean(P_mag/total)*100:.1f}%)")
        print(f"    I-term: mean={np.mean(I_mag):.4f} m/s ({np.mean(I_mag/total)*100:.1f}%)")
        print(f"    D-term: mean={np.mean(np.abs(D_mag)):.4f} m/s ({np.mean(np.abs(D_mag)/total)*100:.1f}%)")
        
        # Estimate current gains
        mask = self.err_mag > 0.01
        if np.sum(mask) > 0:
            self.est_Kp = np.median(P_mag[mask] / self.err_mag[mask])
            print(f"    Estimated current Kp ≈ {self.est_Kp:.3f}")
        else:
            self.est_Kp = 0.5
            
        self.P_mag = P_mag
        self.I_mag = I_mag
        self.D_mag = D_mag
        
    def _detect_problems(self):
        """Phát hiện vấn đề."""
        print(f"\n[4] Problem Detection:")
        
        self.kp_factor = 1.0
        self.kd_factor = 1.0
        self.ki_factor = 1.0
        
        # Oscillation
        err_x = self.df['err_x'].values
        sign_changes = np.sum(np.abs(np.diff(np.sign(err_x))) > 0)
        oscillation_ratio = sign_changes / len(err_x)
        
        if oscillation_ratio > 0.3:
            print(f"    [!] OSCILLATION detected ({sign_changes} sign changes)")
            print(f"        → Giảm Kp và/hoặc tăng Kd")
            self.kp_factor = 0.7
            self.kd_factor = 1.3
        elif oscillation_ratio < 0.05:
            print(f"    [i] Phản ứng chậm (ít oscillation)")
            print(f"        → Có thể tăng Kp")
            self.kp_factor = 1.2
        else:
            print(f"    [✓] Oscillation: OK ({sign_changes} sign changes)")
            
        # Overshoot at end
        if len(err_x) > 20:
            end_err = err_x[-20:]
            end_sign_changes = np.sum(np.abs(np.diff(np.sign(end_err))) > 0)
            if end_sign_changes > 3:
                print(f"    [!] OVERSHOOT at goal ({end_sign_changes} reversals)")
                print(f"        → Tăng Kd để damping")
                self.kd_factor = max(self.kd_factor, 1.4)
            else:
                print(f"    [✓] Overshoot: OK")
                
        # I-term saturation
        if np.max(self.I_mag) > 0.04:
            print(f"    [!] I-term may be saturating (max: {np.max(self.I_mag):.3f})")
            print(f"        → Giảm Ki hoặc tăng MAX_I_TERM")
            self.ki_factor = 0.7
        else:
            print(f"    [✓] I-term: OK")
            
        # Steady-state error
        final_err = np.mean(self.err_mag[-10:])
        if final_err > 0.05:
            print(f"    [!] High steady-state error ({final_err*100:.1f} cm)")
            print(f"        → Có thể tăng Ki nhẹ")
            self.ki_factor = max(self.ki_factor, 1.2)
        else:
            print(f"    [✓] Steady-state: OK ({final_err*100:.1f} cm)")
            
    def _calculate_recommended_gains(self):
        """Tính PID gains khuyến nghị."""
        print(f"\n[5] RECOMMENDED PID GAINS:")
        print("=" * 60)
        
        # Constants from trajectory_executor.h
        MAX_VELOCITY = 0.20
        
        # Calculate Kp based on velocity constraint and typical error
        # Kp = velocity / error => reasonable response
        rec_Kp = 0.5 * MAX_VELOCITY / self.typical_error if self.typical_error > 0.01 else 0.5
        rec_Kp = rec_Kp * self.kp_factor
        rec_Kp = np.clip(rec_Kp, 0.3, 1.5)
        
        # Kd for damping, proportional to Kp
        rec_Kd = rec_Kp * 0.15 * self.kd_factor
        rec_Kd = np.clip(rec_Kd, 0.05, 0.3)
        
        # Ki very small to avoid windup
        rec_Ki = rec_Kp / 25 * self.ki_factor
        rec_Ki = np.clip(rec_Ki, 0.01, 0.05)
        
        print(f"""
// PID Control Parameters - Auto-tuned
// Analyzed from {len(self.df)} samples at {1/self.dt:.0f}Hz
// Typical error: {self.typical_error*100:.1f}cm
// Adjustment factors: Kp×{self.kp_factor:.1f}, Kd×{self.kd_factor:.1f}, Ki×{self.ki_factor:.1f}

#define TRAJ_KP {rec_Kp:.4f}f
#define TRAJ_KI {rec_Ki:.4f}f
#define TRAJ_KD {rec_Kd:.4f}f
#define MAX_I_TERM 0.05f
""")
        
        # Also suggest alternatives
        print("\nAlternative tunings:")
        print(f"  Conservative: Kp={rec_Kp*0.8:.3f}, Ki={rec_Ki*0.8:.3f}, Kd={rec_Kd*1.2:.3f}")
        print(f"  Aggressive:   Kp={rec_Kp*1.2:.3f}, Ki={rec_Ki*1.2:.3f}, Kd={rec_Kd*0.8:.3f}")
        
    def _plot(self):
        """Vẽ đồ thị phân tích."""
        time = self.df['time'].values
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # 1. Position tracking
        ax = axes[0, 0]
        ax.plot(self.df['tgt_x'].values, self.df['tgt_y'].values, 'g--', label='Target', linewidth=2)
        ax.plot(self.df['cur_x'].values, self.df['cur_y'].values, 'b-', label='Actual', alpha=0.7)
        ax.plot(self.df['tgt_x'].values[-1], self.df['tgt_y'].values[-1], 'r*', markersize=15, label='Goal')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Position Tracking')
        ax.legend()
        ax.axis('equal')
        ax.grid(True)
        
        # 2. Error magnitude
        ax = axes[0, 1]
        ax.plot(time, self.err_mag * 100, 'r-')
        ax.axhline(y=5, color='g', linestyle='--', alpha=0.5, label='5cm threshold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (cm)')
        ax.set_title('Position Error')
        ax.legend()
        ax.grid(True)
        
        # 3. PID contributions
        ax = axes[1, 0]
        ax.plot(time, self.df['P_x'].values, 'b-', label='P_x', alpha=0.7)
        ax.plot(time, self.df['I_x'].values, 'g-', label='I_x', alpha=0.7)
        ax.plot(time, self.df['D_x'].values, 'r-', label='D_x', alpha=0.7)
        ax.plot(time, self.df['cmd_vx'].values, 'k--', label='Cmd_vx', alpha=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('PID Contributions (X)')
        ax.legend()
        ax.grid(True)
        
        # 4. Command velocity magnitude
        ax = axes[1, 1]
        cmd_mag = np.sqrt(self.df['cmd_vx'].values**2 + self.df['cmd_vy'].values**2)
        ax.plot(time, cmd_mag, 'b-', label='Cmd velocity')
        ax.axhline(y=0.20, color='r', linestyle='--', alpha=0.5, label='MAX_VEL')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Command Velocity')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        output_file = Path(sys.argv[1]).stem + '_analysis.png'
        plt.savefig(output_file, dpi=150)
        print(f"\n[Plot saved: {output_file}]")
        plt.show()


def main():
    if len(sys.argv) < 2:
        print("PID Analyzer - Phân tích CSV từ Xavier")
        print("=" * 50)
        print("\nCách dùng:")
        print("  1. Bật ENABLE_PID_LOGGING=1 trong sys_config.h")
        print("  2. Build và deploy lên Xavier")
        print("  3. Chạy trajectory trên Xavier")
        print("  4. Copy file từ Xavier về laptop:")
        print("     scp xavier:/tmp/pid_log.csv .")
        print("  5. Phân tích:")
        print("     python3 pid_analyzer.py pid_log.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    try:
        analyzer = PIDAnalyzer(csv_file)
        analyzer.analyze()
    except FileNotFoundError:
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
