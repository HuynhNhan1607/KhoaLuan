"""
Script để sửa lỗi double rotation trong dữ liệu localization.

Vấn đề: Giá trị ban đầu (stable position) bị xoay 2 lần bởi align_position().
Giải pháp: Tính delta_x, delta_y để trừ lại, khôi phục về giá trị xoay 1 lần.

Công thức align_position:
    rad = (THETA - ALPHA) * PI / 180
    x_aligned = SCALE * (cos(rad)*x - sin(rad)*y) + XT
    y_aligned = SCALE * (sin(rad)*x + cos(rad)*y) + YT
"""
import math

# Các tham số từ localize.h
SCALE_FACTOR = 0.9765408457852169
ALPHA = 6.24
THETA =  6.2625838893853265  # độ
XT = 0.27265129
YT = -0.008631


def align_position(x, y):
    """Áp dụng transformation 1 lần (như trong C code)"""
    rad = (THETA - ALPHA) * math.pi / 180.0
    c = math.cos(rad)
    s = math.sin(rad)
    
    x_new = SCALE_FACTOR * (c * x - s * y) + XT
    y_new = SCALE_FACTOR * (s * x + c * y) + YT
    return x_new, y_new


def inverse_align_position(x, y):
    """Đảo ngược transformation (xoay ngược lại)"""
    # Trừ translation trước
    x_temp = x - XT
    y_temp = y - YT
    
    # Chia cho scale
    x_temp /= SCALE_FACTOR
    y_temp /= SCALE_FACTOR
    
    # Xoay ngược (dùng -rad)
    rad = (THETA - ALPHA) * math.pi / 180.0
    c = math.cos(-rad)
    s = math.sin(-rad)
    
    x_original = c * x_temp - s * y_temp
    y_original = s * x_temp + c * y_temp
    return x_original, y_original


def calculate_correction(double_rotated_x, double_rotated_y):
    """
    Tính delta_x, delta_y để sửa lỗi double rotation.
    
    Input: Tọa độ đã bị xoay 2 lần
    Output: delta_x, delta_y để trừ khỏi dữ liệu
    """
    # Bước 1: Đảo ngược 1 lần xoay để có tọa độ xoay 1 lần (đúng)
    single_rotated_x, single_rotated_y = inverse_align_position(
        double_rotated_x, double_rotated_y
    )
    
    # Bước 2: Tính delta
    delta_x = double_rotated_x - single_rotated_x
    delta_y = double_rotated_y - single_rotated_y
    
    return delta_x, delta_y, single_rotated_x, single_rotated_y


def main():
    print("=" * 50)
    print("FIX DOUBLE ROTATION - Localization Data")
    print("=" * 50)
    print(f"\nTham số hiện tại:")
    print(f"  SCALE_FACTOR = {SCALE_FACTOR}")
    print(f"  THETA = {THETA}°")
    print(f"  ALPHA = {ALPHA}°")
    print(f"  XT = {XT}")
    print(f"  YT = {YT}")
    print()
    
    # Nhập dữ liệu bị xoay 2 lần
    try:
        print("Nhập tọa độ đã bị xoay 2 lần (stable position từ ESP32):")
        x_input = float(input("  X = "))
        y_input = float(input("  Y = "))
    except ValueError:
        print("Lỗi: Vui lòng nhập số!")
        return
    
    # Tính toán
    delta_x, delta_y, correct_x, correct_y = calculate_correction(x_input, y_input)
    
    # Kết quả
    print("\n" + "=" * 50)
    print("KẾT QUẢ:")
    print("=" * 50)
    print(f"\n  Giá trị nhập (bị xoay 2 lần):")
    print(f"    X = {x_input:.6f}")
    print(f"    Y = {y_input:.6f}")
    
    print(f"\n  Giá trị đúng (chỉ xoay 1 lần):")
    print(f"    X = {correct_x:.6f}")
    print(f"    Y = {correct_y:.6f}")
    
    print(f"\n  DELTA để trừ lại:")
    print(f"    delta_X = {delta_x:.6f}")
    print(f"    delta_Y = {delta_y:.6f}")
    
    print(f"\n  Công thức sửa dữ liệu:")
    print(f"    X_corrected = X_data - {delta_x:.6f}")
    print(f"    Y_corrected = Y_data - {delta_y:.6f}")
    print()


if __name__ == "__main__":
    main()
