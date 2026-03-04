#!/usr/bin/env python3
"""
PID Tuner từ Console Log
========================
Parse log output từ trajectory_executor và tính PID gains.

Cách sử dụng:
1. Chạy robot với trajectory
2. Copy terminal output vào file (vd: log.txt)
3. Chạy: python3 pid_tuner_from_log.py log.txt

Log format expected:
[TRAJ] Cur(0.500,2.000) -> Tgt(0.550,2.200) Final(0.950,4.430)
[TRAJ] Idx[50/97] DistToTgt:0.210 DistToFinal:2.500 SpeedLimit:0.200 Cmd(0.100,0.150)
[PID] Err(0.050,0.200) P(0.035,0.140) I(0.002,0.008) D(-0.001,-0.002)
"""

import re
import sys
import numpy as np
import matplotlib.pyplot as plt

def parse_log(filename):
    """Parse trajectory log file."""
    data = {
        'cur_x': [], 'cur_y': [],
        'tgt_x': [], 'tgt_y': [],
        'final_x': [], 'final_y': [],
        'dist_to_tgt': [], 'dist_to_final': [],
        'speed_limit': [],
        'cmd_x': [], 'cmd_y': [],
        'err_x': [], 'err_y': [],
        'P_x': [], 'P_y': [],
        'I_x': [], 'I_y': [],
        'D_x': [], 'D_y': [],
    }
    
    # Patterns
    traj_pattern = r'\[TRAJ\] Cur\(([-\d.]+),([-\d.]+)\) -> Tgt\(([-\d.]+),([-\d.]+)\) Final\(([-\d.]+),([-\d.]+)\)'
    idx_pattern = r'\[TRAJ\] Idx\[\d+/\d+\] DistToTgt:([-\d.]+) DistToFinal:([-\d.]+) SpeedLimit:([-\d.]+) Cmd\(([-\d.]+),([-\d.]+)\)'
    pid_pattern = r'\[PID\] Err\(([-\d.]+),([-\d.]+)\) P\(([-\d.]+),([-\d.]+)\) I\(([-\d.]+),([-\d.]+)\) D\(([-\d.]+),([-\d.]+)\)'
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Match TRAJ position line
        m1 = re.search(traj_pattern, line)
        if m1:
            data['cur_x'].append(float(m1.group(1)))
            data['cur_y'].append(float(m1.group(2)))
            data['tgt_x'].append(float(m1.group(3)))
            data['tgt_y'].append(float(m1.group(4)))
            data['final_x'].append(float(m1.group(5)))
            data['final_y'].append(float(m1.group(6)))
            
            # Look for corresponding Idx line
            if i + 1 < len(lines):
                m2 = re.search(idx_pattern, lines[i+1])
                if m2:
                    data['dist_to_tgt'].append(float(m2.group(1)))
                    data['dist_to_final'].append(float(m2.group(2)))
                    data['speed_limit'].append(float(m2.group(3)))
                    data['cmd_x'].append(float(m2.group(4)))
                    data['cmd_y'].append(float(m2.group(5)))
                    
                    # Look for PID line
                    if i + 2 < len(lines):
                        m3 = re.search(pid_pattern, lines[i+2])
                        if m3:
                            data['err_x'].append(float(m3.group(1)))
                            data['err_y'].append(float(m3.group(2)))
                            data['P_x'].append(float(m3.group(3)))
                            data['P_y'].append(float(m3.group(4)))
                            data['I_x'].append(float(m3.group(5)))
                            data['I_y'].append(float(m3.group(6)))
                            data['D_x'].append(float(m3.group(7)))
                            data['D_y'].append(float(m3.group(8)))
                            i += 2
                        else:
                            # No PID line, fill with zeros
                            for key in ['err_x', 'err_y', 'P_x', 'P_y', 'I_x', 'I_y', 'D_x', 'D_y']:
                                data[key].append(0)
                    i += 1
                else:
                    # No Idx line, skip
                    for key in ['dist_to_tgt', 'dist_to_final', 'speed_limit', 'cmd_x', 'cmd_y',
                                'err_x', 'err_y', 'P_x', 'P_y', 'I_x', 'I_y', 'D_x', 'D_y']:
                        data[key].append(0)
        i += 1
    
    # Verify data consistency
    min_len = min(len(v) for v in data.values())
    for key in data:
        data[key] = data[key][:min_len]
    
    return data

def analyze_and_tune(data):
    """Analyze data and compute optimal PID gains."""
    n = len(data['cur_x'])
    if n < 5:
        print("Không đủ data! Cần ít nhất 5 điểm.")
        return None
    
    print("=" * 60)
    print("PID AUTO-TUNER - Phân tích từ Log")
    print("=" * 60)
    print(f"\nĐã parse được {n} điểm data")
    
    # Convert to numpy
    for key in data:
        data[key] = np.array(data[key])
    
    # System parameters
    dt = 1.0  # 1 second between log outputs (20Hz / 20 = 1Hz logging)
    
    # 1. Analyze error statistics
    err_mag = np.sqrt(data['err_x']**2 + data['err_y']**2)
    print(f"\n[1] Phân tích Error:")
    print(f"    Mean Error:  {np.mean(err_mag)*100:.2f} cm")
    print(f"    Max Error:   {np.max(err_mag)*100:.2f} cm")
    print(f"    Final Error: {err_mag[-1]*100:.2f} cm")
    
    # 2. Analyze PID contributions
    P_mag = np.sqrt(data['P_x']**2 + data['P_y']**2)
    I_mag = np.sqrt(data['I_x']**2 + data['I_y']**2)
    D_mag = np.sqrt(data['D_x']**2 + data['D_y']**2)
    
    print(f"\n[2] Phân tích PID Contributions:")
    print(f"    P-term: mean={np.mean(P_mag):.4f}, max={np.max(P_mag):.4f}")
    print(f"    I-term: mean={np.mean(I_mag):.4f}, max={np.max(I_mag):.4f}")
    print(f"    D-term: mean={np.mean(D_mag):.4f}, max={np.max(D_mag):.4f}")
    
    # 3. Estimate current gains from data
    # P = Kp * err => Kp = P / err
    mask = err_mag > 0.01  # Only use significant errors
    if np.sum(mask) > 0:
        est_Kp = np.median(P_mag[mask] / err_mag[mask])
    else:
        est_Kp = 0.5
    print(f"\n[3] Ước lượng gains hiện tại:")
    print(f"    Estimated Kp ≈ {est_Kp:.3f}")
    
    # 4. Detect oscillation
    err_x_diff = np.diff(np.sign(data['err_x']))
    oscillations = np.sum(np.abs(err_x_diff) > 0)
    print(f"\n[4] Phân tích dao động:")
    print(f"    Số lần đổi dấu error X: {oscillations}")
    if oscillations > n / 3:
        print("    [!] HỆ THỐNG ĐANG OSCILLATE - Giảm Kp!")
        kp_factor = 0.7
    elif oscillations < 2:
        print("    [i] Phản ứng chậm - Có thể tăng Kp")
        kp_factor = 1.2
    else:
        print("    [✓] Ổn định")
        kp_factor = 1.0
    
    # 5. Check overshoot
    # Overshoot if error changes sign near the end
    if len(data['err_x']) > 5:
        end_errors = data['err_x'][-5:]
        sign_changes = np.sum(np.abs(np.diff(np.sign(end_errors))) > 0)
        if sign_changes > 1:
            print(f"\n[5] Phát hiện OVERSHOOT gần đích ({sign_changes} lần)")
            print("    [!] Tăng Kd để giảm overshoot")
            kd_factor = 1.3
        else:
            kd_factor = 1.0
    else:
        kd_factor = 1.0
    
    # 6. Calculate recommended gains
    print(f"\n[6] RECOMMENDED PID GAINS:")
    print("=" * 60)
    
    # Method 1: Based on velocity constraint
    max_vel = 0.20
    typical_err = np.percentile(err_mag, 75)
    rec_Kp = 0.5 * max_vel / typical_err if typical_err > 0.01 else 0.5
    rec_Kp = np.clip(rec_Kp * kp_factor, 0.3, 1.5)
    
    # Kd proportional to Kp and dt
    rec_Kd = rec_Kp * 0.2 * kd_factor
    rec_Kd = np.clip(rec_Kd, 0.05, 0.3)
    
    # Ki very small to avoid windup
    rec_Ki = rec_Kp / 30
    rec_Ki = np.clip(rec_Ki, 0.01, 0.05)
    
    print(f"""
// PID Control Parameters - Auto-tuned
// Based on trajectory analysis ({n} data points)
// Typical error: {typical_err*100:.1f}cm, Max velocity: {max_vel}m/s
#define TRAJ_KP {rec_Kp:.4f}f
#define TRAJ_KI {rec_Ki:.4f}f  
#define TRAJ_KD {rec_Kd:.4f}f
#define MAX_I_TERM 0.05f
""")
    
    return {
        'Kp': rec_Kp,
        'Ki': rec_Ki,
        'Kd': rec_Kd,
        'data': data
    }

def plot_analysis(data, result):
    """Generate analysis plots."""
    n = len(data['cur_x'])
    time = np.arange(n)  # 1 second per sample
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # 1. Position tracking
    ax = axes[0, 0]
    ax.plot(data['tgt_x'], data['tgt_y'], 'g--', label='Target', linewidth=2)
    ax.plot(data['cur_x'], data['cur_y'], 'b-', label='Actual', alpha=0.7)
    ax.plot(data['final_x'][0], data['final_y'][0], 'r*', markersize=15, label='Goal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Position Tracking')
    ax.legend()
    ax.axis('equal')
    ax.grid(True)
    
    # 2. Error over time
    ax = axes[0, 1]
    err_mag = np.sqrt(data['err_x']**2 + data['err_y']**2)
    ax.plot(time, err_mag * 100, 'r-')
    ax.axhline(y=5, color='g', linestyle='--', label='Acceptance (5cm)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (cm)')
    ax.set_title('Position Error')
    ax.legend()
    ax.grid(True)
    
    # 3. PID contributions
    ax = axes[1, 0]
    ax.plot(time, data['P_x'], 'b-', label='P', alpha=0.8)
    ax.plot(time, data['I_x'], 'g-', label='I', alpha=0.8)
    ax.plot(time, data['D_x'], 'r-', label='D', alpha=0.8)
    ax.plot(time, data['cmd_x'], 'k--', label='Cmd', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('m/s')
    ax.set_title('PID Contributions (X axis)')
    ax.legend()
    ax.grid(True)
    
    # 4. Command velocity
    ax = axes[1, 1]
    cmd_mag = np.sqrt(data['cmd_x']**2 + data['cmd_y']**2)
    ax.plot(time, cmd_mag, 'b-', label='Command', alpha=0.8)
    ax.plot(time, data['speed_limit'], 'r--', label='Speed Limit', alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Command Velocity vs Speed Limit')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    plt.savefig('pid_analysis.png', dpi=150)
    print(f"\n[Plot saved: pid_analysis.png]")
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Cách sử dụng:")
        print("  1. Chạy robot và save terminal output vào file:")
        print("     ./main 2>&1 | tee trajectory_log.txt")
        print("")
        print("  2. Chạy analyzer:")
        print("     python3 pid_tuner_from_log.py trajectory_log.txt")
        print("")
        print("Log phải có format:")
        print("  [TRAJ] Cur(x,y) -> Tgt(x,y) Final(x,y)")
        print("  [TRAJ] Idx[n/m] DistToTgt:d DistToFinal:d SpeedLimit:s Cmd(vx,vy)")
        print("  [PID] Err(ex,ey) P(px,py) I(ix,iy) D(dx,dy)")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    try:
        print(f"Đang parse log: {filename}")
        data = parse_log(filename)
        
        if len(data['cur_x']) == 0:
            print("Không tìm thấy data trong log!")
            print("Hãy chắc chắn log có [PID] lines.")
            sys.exit(1)
        
        result = analyze_and_tune(data)
        
        if result:
            plot_analysis(data, result)
            
    except FileNotFoundError:
        print(f"Không tìm thấy file: {filename}")
        sys.exit(1)
    except Exception as e:
        print(f"Lỗi: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
