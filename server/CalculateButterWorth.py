import numpy as np
from scipy.signal import butter

def butter_lowpass(cutoff_freq, sampling_freq, order=5):
    """
    Thiết kế bộ lọc thông thấp Butterworth
    :param cutoff_freq: Tần số cắt (Hz)
    :param sampling_freq: Tần số lấy mẫu (Hz)
    :param order: Bậc của bộ lọc
    :return: Hệ số bộ lọc b, a
    """
    nyquist = 0.5 * sampling_freq  # Tần số Nyquist
    normal_cutoff = cutoff_freq / nyquist  # Chuẩn hóa tần số cắt
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# Thông số
sampling_freq = 50     # Tần số lấy mẫu (Hz)
cutoff_freq = 2      # Tần số cắt (Hz)
order = 2              # Bậc của bộ lọc

time = np.linspace(0, 1, sampling_freq, endpoint=False)  # Trục thời gian
# Tạo bộ lọc
b, a = butter_lowpass(cutoff_freq, sampling_freq, order)

# In phương trình bộ lọc dạng sai phân
y_equation = f"y[n] = {-a[1]:.6f} * y[n-1]"
for i in range(2, len(a)):
    y_equation += f" + {-a[i]:.6f} * y[n-{i}]"
for i in range(len(b)):
    y_equation += f" + {b[i]:.6f} * x[n-{i}]"

print("Hệ số tử số b:", b)
print("Hệ số mẫu số a:", a)
print("Phương trình sai phân:", y_equation)
