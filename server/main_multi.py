import warnings
# Suppress matplotlib legend warnings that spam the terminal
warnings.filterwarnings("ignore", message="No artists with labels found to put in legend")
warnings.filterwarnings("ignore", message="Setting the 'color' property will override")

import ctypes
try:
    # Báo cho Windows biết ứng dụng này có hỗ trợ DPI
    ctypes.windll.shcore.SetProcessDpiAwareness(1) 
except Exception:
    pass  # Silently ignore DPI warning on non-Windows
    
import tkinter as tk
from server_gui_multi import ServerGUI

if __name__ == "__main__":
    root = tk.Tk()
    gui = ServerGUI(root)
    root.mainloop()
