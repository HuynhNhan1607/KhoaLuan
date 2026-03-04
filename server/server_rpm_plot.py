import matplotlib
matplotlib.use('TkAgg')  # Đảm bảo sử dụng backend TkAgg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import time
import numpy as np
import pandas as pd
from datetime import datetime
import os
import tkinter as tk
from tkinter import messagebox
import threading

TIME_INTERVAL = 100  # ms - Giảm xuống để cập nhật mượt hơn
MAX_DATA_POINTS = 300  # Giới hạn số điểm dữ liệu để tránh lag

class RPMPlotter:
    def __init__(self, robot_id=None):
        self.robot_id = robot_id  # Store robot_id for per-robot plotting
        self.rpm_data = [[] for _ in range(4)]
        self.time_data = []
        self.start_time = time.time()
        self.last_update_time = 0
        self.buffer = []
        self.buffer_lock = threading.Lock()  # Lock để bảo vệ buffer
        self.plot_window = None
        self.user_zoom = False  # Thêm biến để theo dõi người dùng đã zoom/pan chưa
        self.is_closing = False  # Flag để tránh race condition khi đóng
        
        # Figure sẽ được tạo trong initialize_figure
        self.fig = None
        self.ax = None
        self.lines = []
        
        self.ani = None # Animation object
        self.canvas = None
        self.toolbar = None

    def initialize_figure(self):
        """
        Tạo đối tượng Figure và Axes. 
        Hàm này BẮT BUỘC phải được gọi từ luồng chính (main thread) CỦA TKINTER.
        """
        if self.fig is not None:
            return # Đã được khởi tạo

        try:
            # Tạo figure và axes
            self.fig, self.ax = plt.subplots(figsize=(10, 6))
            self.lines = [self.ax.plot([], [], label=f"Motor {i+1}", linewidth=2)[0] for i in range(4)]
            
            # Làm đẹp biểu đồ
            self.ax.set_xlim(0, 30)
            self.ax.set_ylim(-200, 200)
            self.ax.set_xlabel("Time (s)", fontsize=12)
            self.ax.set_ylabel("RPM", fontsize=12)
            title = f"Robot {self.robot_id} - Motor RPM Real-time Monitoring" if self.robot_id else "Motor RPM Real-time Monitoring"
            self.ax.set_title(title, fontsize=14)
            self.ax.grid(True, linestyle='--', alpha=0.7)
            self.ax.legend(fontsize=10)
            
            plt.tight_layout()

            # Thêm handler cho sự kiện zoom/pan
            self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_event)
            self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_event)
            self.fig.canvas.mpl_connect('scroll_event', self.on_mouse_event)
            
        except Exception as e:
            print(f"Lỗi nghiêm trọng khi khởi tạo Matplotlib cho Robot {self.robot_id}: {e}")
            messagebox.showerror("Lỗi Matplotlib", 
                                 f"Không thể tạo đồ thị cho Robot {self.robot_id}.\n"
                                 f"Lỗi: {e}\n"
                                 "Chắc chắn rằng Matplotlib và Tkinter đã được cài đặt đúng cách.")

    def show_plot(self):
        # Đảm bảo figure đã được tạo (từ luồng chính)
        if self.fig is None:
            try:
                self.initialize_figure()
            except Exception as e:
                print(f"Lỗi khởi tạo figure cho Robot {self.robot_id}: {e}")
                messagebox.showerror("Lỗi", f"Không thể khởi tạo đồ thị: {e}")
                return
            
        # If window exists but is hidden, show it again
        if self.plot_window is not None and self.plot_window.winfo_exists():
            self.plot_window.deiconify()
            self.plot_window.focus_force()
            
            # Khởi động lại animation nếu nó đã bị dừng
            if self.ani is None and not self.is_closing:
                self.ani = FuncAnimation(self.fig, self.update_plot, 
                                         interval=TIME_INTERVAL, 
                                         cache_frame_data=False,
                                         blit=False)  # TẮT blitting để tránh đứng
            return
            
        # Otherwise create a new window
        self.is_closing = False
        self.plot_window = tk.Toplevel()
        self.plot_window.title(f"Robot {self.robot_id} RPM Monitoring")
        self.plot_window.geometry("800x700")
        
        # Tạo canvas cho matplotlib
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_window)
        
        # Thêm toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.plot_window)
        self.toolbar.update()
        
        # Đặt canvas vào cửa sổ
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Vẽ lần đầu
        try:
            self.canvas.draw()
        except Exception as e:
            print(f"Lỗi vẽ canvas cho Robot {self.robot_id}: {e}")
        
        # Bắt đầu animation
        self.ani = FuncAnimation(self.fig, self.update_plot, 
                                 interval=TIME_INTERVAL, 
                                 cache_frame_data=False,
                                 blit=False)
        
        # Đăng ký sự kiện đóng cửa sổ
        self.plot_window.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def on_close(self):
        """Đóng cửa sổ plot một cách an toàn"""
        self.is_closing = True
        
        # Dừng animation
        if self.ani is not None:
            try:
                self.ani.event_source.stop()
                self.ani = None
            except Exception as e:
                print(f"Lỗi khi dừng animation Robot {self.robot_id}: {e}")
        
        # Xóa canvas và đóng cửa sổ
        if self.canvas is not None:
            self.canvas.get_tk_widget().destroy()
            self.canvas = None
        
        if self.plot_window is not None:
            self.plot_window.destroy()
            self.plot_window = None
    
    def update_plot(self, frame):
        """Cập nhật dữ liệu trên biểu đồ."""
        if self.is_closing or self.fig is None:
            return
            
        try:
            # Xử lý buffer với thread safety
            with self.buffer_lock:
                if not self.buffer:
                    return
                buffer_copy = self.buffer[:]
                self.buffer.clear()
                
            # Thêm dữ liệu từ buffer
            for encoders in buffer_copy:
                self._add_rpm_data(encoders)
            
            # Giới hạn số điểm dữ liệu để tránh lag
            if len(self.time_data) > MAX_DATA_POINTS:
                self.time_data = self.time_data[-MAX_DATA_POINTS:]
                for i in range(4):
                    self.rpm_data[i] = self.rpm_data[i][-MAX_DATA_POINTS:]
            
            # Kiểm tra có dữ liệu không
            if not self.time_data:
                return
            
            # Cập nhật dữ liệu cho 4 đường line
            for i in range(4):
                self.lines[i].set_data(self.time_data, self.rpm_data[i])

            # Logic điều chỉnh trục X và Y
            if not self.user_zoom:
                if len(self.time_data) > 1:
                    # Hiển thị 30 giây gần nhất
                    x_max = self.time_data[-1]
                    x_min = max(0, x_max - 30)
                    self.ax.set_xlim(x_min, x_max + 1)

                # Tự động điều chỉnh trục Y dựa trên dữ liệu hiện tại
                if any(self.rpm_data[i] for i in range(4)):
                    all_values = []
                    for rpm_list in self.rpm_data:
                        all_values.extend([val for val in rpm_list if val is not None])
                    
                    if all_values:
                        min_val = min(all_values)
                        max_val = max(all_values)
                        padding = max(20, (max_val - min_val) * 0.15)
                        self.ax.set_ylim(min_val - padding, max_val + padding)
                        
            elif len(self.time_data) > 0:
                # Nếu người dùng đã zoom, vẫn scroll theo thời gian
                x_max = self.time_data[-1]
                current_xlim = self.ax.get_xlim()
                if x_max > current_xlim[1]:
                    current_span = current_xlim[1] - current_xlim[0]
                    self.ax.set_xlim(x_max - current_span, x_max)
            
            # Vẽ lại figure
            if self.canvas is not None:
                self.canvas.draw_idle()
                
        except Exception as e:
            print(f"Lỗi update_plot Robot {self.robot_id}: {e}")
            import traceback
            traceback.print_exc()
    
    def add_rpm_data(self, encoders):
        """Nhận dữ liệu và thêm vào buffer (an toàn thread)."""
        if self.is_closing:
            return
        with self.buffer_lock:
            self.buffer.append(encoders)

    def _add_rpm_data(self, encoders):
        """Thêm dữ liệu từ buffer vào danh sách."""
        try:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)

            for i in range(4):
                try:
                    value = float(encoders[i]) if encoders[i] not in [None, ""] else 0
                    self.rpm_data[i].append(value)
                except (ValueError, IndexError, TypeError): 
                    self.rpm_data[i].append(0)

            # Đồng bộ hóa độ dài dữ liệu
            min_length = min(len(self.time_data), *(len(self.rpm_data[i]) for i in range(4)))
            if min_length < len(self.time_data):
                self.time_data = self.time_data[-min_length:]
                for i in range(4):
                    self.rpm_data[i] = self.rpm_data[i][-min_length:]
        except Exception as e:
            print(f"Lỗi _add_rpm_data Robot {self.robot_id}: {e}")
    
    def on_mouse_event(self, event):
        # Đánh dấu rằng người dùng đã can thiệp vào đồ thị
        self.user_zoom = True

# --- THAY ĐỔI QUAN TRỌNG: Quản lý đa robot ---

# Dictionary toàn cục để lưu trữ các instance plotter
rpm_plotters = {}  
_plotter_lock = threading.Lock() # Lock để bảo vệ dict

def get_rpm_plotter(robot_id):
    """Lấy hoặc tạo RPMPlotter instance cho một robot_id cụ thể."""
    with _plotter_lock:
        if robot_id not in rpm_plotters:
            # Tạo instance mới
            rpm_plotters[robot_id] = RPMPlotter(robot_id=robot_id)
        return rpm_plotters[robot_id]

def update_rpm_plot(robot_id, encoders):
    """Hàm API công khai: Cập nhật RPM plot cho một robot cụ thể."""
    # Lấy đúng plotter (an toàn thread)
    plotter = get_rpm_plotter(robot_id)
    # Thêm dữ liệu vào buffer của plotter đó
    plotter.add_rpm_data(encoders)