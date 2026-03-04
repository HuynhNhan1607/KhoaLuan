import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Đọc file CSV
csv_file = 'logs/sync_position_log_20260203_142311.csv'
df = pd.read_csv(csv_file)

# Tách dữ liệu cho robot1 và robot2
robot1_data = df[df['robot_id'] == 'robot1'].copy()
robot2_data = df[df['robot_id'] == 'robot2'].copy()

# Chuyển đổi timestamp thành thời gian tương đối (seconds)
robot1_data['time'] = robot1_data['server_timestamp'] - robot1_data['server_timestamp'].iloc[0]
robot2_data['time'] = robot2_data['server_timestamp'] - robot2_data['server_timestamp'].iloc[0]

# Tạo figure với 3 subplots
fig = plt.figure(figsize=(15, 10))

# 1. Plot pos_unc cho robot1
ax1 = plt.subplot(3, 2, 1)
ax1.plot(robot1_data['time'].values, robot1_data['pos_unc'].values, 'b-', linewidth=1.5, alpha=0.7)
ax1.set_xlabel('Time (s)', fontsize=12)
ax1.set_ylabel('Position Uncertainty (m)', fontsize=12)
ax1.set_title('Robot 1 - Position Uncertainty', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.set_ylim([0, max(robot1_data['pos_unc'].max(), robot2_data['pos_unc'].max()) * 1.1])

# 2. Plot pos_unc cho robot2
ax2 = plt.subplot(3, 2, 2)
ax2.plot(robot2_data['time'].values, robot2_data['pos_unc'].values, 'r-', linewidth=1.5, alpha=0.7)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Position Uncertainty (m)', fontsize=12)
ax2.set_title('Robot 2 - Position Uncertainty', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.set_ylim([0, max(robot1_data['pos_unc'].max(), robot2_data['pos_unc'].max()) * 1.1])

# 3. Plot vel_unc cho robot1
ax3 = plt.subplot(3, 2, 3)
ax3.plot(robot1_data['time'].values, robot1_data['vel_unc'].values, 'b-', linewidth=1.5, alpha=0.7)
ax3.set_xlabel('Time (s)', fontsize=12)
ax3.set_ylabel('Velocity Uncertainty (m/s)', fontsize=12)
ax3.set_title('Robot 1 - Velocity Uncertainty', fontsize=14, fontweight='bold')
ax3.grid(True, alpha=0.3)
ax3.set_ylim([0, max(robot1_data['vel_unc'].max(), robot2_data['vel_unc'].max()) * 1.1])

# 4. Plot vel_unc cho robot2
ax4 = plt.subplot(3, 2, 4)
ax4.plot(robot2_data['time'].values, robot2_data['vel_unc'].values, 'r-', linewidth=1.5, alpha=0.7)
ax4.set_xlabel('Time (s)', fontsize=12)
ax4.set_ylabel('Velocity Uncertainty (m/s)', fontsize=12)
ax4.set_title('Robot 2 - Velocity Uncertainty', fontsize=14, fontweight='bold')
ax4.grid(True, alpha=0.3)
ax4.set_ylim([0, max(robot1_data['vel_unc'].max(), robot2_data['vel_unc'].max()) * 1.1])

# 5. So sánh pos_unc giữa robot1 và robot2
ax5 = plt.subplot(3, 2, 5)
ax5.plot(robot1_data['time'].values, robot1_data['pos_unc'].values, 'b-', linewidth=1.5, alpha=0.7, label='Robot 1')
ax5.plot(robot2_data['time'].values, robot2_data['pos_unc'].values, 'r-', linewidth=1.5, alpha=0.7, label='Robot 2')
ax5.set_xlabel('Time (s)', fontsize=12)
ax5.set_ylabel('Position Uncertainty (m)', fontsize=12)
ax5.set_title('Comparison - Position Uncertainty', fontsize=14, fontweight='bold')
ax5.legend(fontsize=11)
ax5.grid(True, alpha=0.3)

# 6. So sánh vel_unc giữa robot1 và robot2
ax6 = plt.subplot(3, 2, 6)
ax6.plot(robot1_data['time'].values, robot1_data['vel_unc'].values, 'b-', linewidth=1.5, alpha=0.7, label='Robot 1')
ax6.plot(robot2_data['time'].values, robot2_data['vel_unc'].values, 'r-', linewidth=1.5, alpha=0.7, label='Robot 2')
ax6.set_xlabel('Time (s)', fontsize=12)
ax6.set_ylabel('Velocity Uncertainty (m/s)', fontsize=12)
ax6.set_title('Comparison - Velocity Uncertainty', fontsize=14, fontweight='bold')
ax6.legend(fontsize=11)
ax6.grid(True, alpha=0.3)

# Thêm tiêu đề chính
plt.suptitle('Uncertainty Analysis - Robot 1 vs Robot 2', fontsize=16, fontweight='bold', y=0.995)
plt.tight_layout()

# Tính toán thống kê
print("\n" + "="*60)
print("THỐNG KÊ UNCERTAINTY")
print("="*60)
print("\nRobot 1:")
print(f"  Position Uncertainty - Mean: {robot1_data['pos_unc'].mean():.6f} m, Std: {robot1_data['pos_unc'].std():.6f} m")
print(f"  Position Uncertainty - Min: {robot1_data['pos_unc'].min():.6f} m, Max: {robot1_data['pos_unc'].max():.6f} m")
print(f"  Velocity Uncertainty - Mean: {robot1_data['vel_unc'].mean():.6f} m/s, Std: {robot1_data['vel_unc'].std():.6f} m/s")
print(f"  Velocity Uncertainty - Min: {robot1_data['vel_unc'].min():.6f} m/s, Max: {robot1_data['vel_unc'].max():.6f} m/s")

print("\nRobot 2:")
print(f"  Position Uncertainty - Mean: {robot2_data['pos_unc'].mean():.6f} m, Std: {robot2_data['pos_unc'].std():.6f} m")
print(f"  Position Uncertainty - Min: {robot2_data['pos_unc'].min():.6f} m, Max: {robot2_data['pos_unc'].max():.6f} m")
print(f"  Velocity Uncertainty - Mean: {robot2_data['vel_unc'].mean():.6f} m/s, Std: {robot2_data['vel_unc'].std():.6f} m/s")
print(f"  Velocity Uncertainty - Min: {robot2_data['vel_unc'].min():.6f} m/s, Max: {robot2_data['vel_unc'].max():.6f} m/s")

print("\nSự khác biệt:")
print(f"  Position Uncertainty Δ Mean: {abs(robot1_data['pos_unc'].mean() - robot2_data['pos_unc'].mean()):.6f} m")
print(f"  Velocity Uncertainty Δ Mean: {abs(robot1_data['vel_unc'].mean() - robot2_data['vel_unc'].mean()):.6f} m/s")
print("="*60 + "\n")

# Hiển thị plot
plt.show()

# Lưu figure
output_file = 'logs/uncertainty_analysis.png'
fig.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"Đã lưu hình vào: {output_file}")
