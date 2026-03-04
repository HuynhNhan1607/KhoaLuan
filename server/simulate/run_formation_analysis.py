#!/usr/bin/env python3
"""
Run Formation Analysis
======================
Script để chạy phân tích đội hình từ file log sync_position_log

Usage:
    python run_formation_analysis.py [--log-file <path>] [--save-report]
"""

import sys
from pathlib import Path

# Add parent path
sys.path.insert(0, str(Path(__file__).parent))

from formation_analysis import FormationAnalyzer
import matplotlib.pyplot as plt


def main():
    # Default log file - sử dụng log mới nhất
    log_dir = Path(__file__).parent.parent / 'logs'
    
    # Tìm file sync_position_log mới nhất
    sync_logs = list(log_dir.glob('sync_position_log_*.csv'))
    
    if not sync_logs:
        print("❌ Không tìm thấy file sync_position_log_*.csv trong thư mục logs/")
        print(f"   Đường dẫn logs: {log_dir}")
        return
    
    # Sắp xếp theo thời gian sửa đổi
    latest_log = max(sync_logs, key=lambda p: p.stat().st_mtime)
    print(f"📂 Sử dụng file log mới nhất: {latest_log.name}")
    
    # Nếu có argument thì dùng argument
    if len(sys.argv) > 1 and sys.argv[1] != '--save-report':
        log_file = sys.argv[1]
    else:
        log_file = str(latest_log)
    
    # Tạo analyzer
    analyzer = FormationAnalyzer(log_file)
    
    # Output paths
    log_path = Path(log_file)
    output_dir = Path(__file__).parent / 'analysis_output'
    output_dir.mkdir(exist_ok=True)
    
    plot_path = output_dir / f'{log_path.stem}_analysis.png'
    report_path = output_dir / f'{log_path.stem}_report.md'
    
    # Chạy phân tích đầy đủ
    print("\n" + "="*70)
    print("🔬 BẮT ĐẦU PHÂN TÍCH ĐỘI HÌNH ROBOT")
    print("="*70)
    
    results = analyzer.run_full_analysis(plot_save_path=str(plot_path))
    
    # Lưu báo cáo
    report = analyzer.generate_comprehensive_report()
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report)
    
    print(f"\n✅ Báo cáo đã lưu tại: {report_path}")
    
    # Đồ thị đã được lưu tự động bởi run_full_analysis
    main_plot = output_dir / f'{log_path.stem}_analysis_main.png'
    detailed_plot = output_dir / f'{log_path.stem}_analysis_detailed.png'
    
    print(f"\n📊 Đồ thị đã lưu:")
    print(f"   - Main Analysis: {main_plot}")
    print(f"   - Detailed Metrics: {detailed_plot}")

    plt.show()
    
    return results


if __name__ == '__main__':
    main()
