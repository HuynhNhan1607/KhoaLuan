import cv2
import cv2.aruco as aruco
import numpy as np

# --- 1. LOAD DATA CALIB ---
data = np.load("C:/Users/nhatm/Downloads/camera/calib_data.npz")
mtx, dist = data['mtx'], data['dist']

# --- 2. CONFIG ---
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

MARKER_SIZE = 0.05  # 5cm
TARGET_Z = 0.16     # 16cm
X_DEADZONE = 0.01   # Sai số ngang cho phép (1cm)

def get_white_blob_center(frame):
    """Tìm trọng tâm của vùng sáng/trắng nhất (tờ giấy/màn hình)"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Làm mờ để bớt nhiễu pixel
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # Ngưỡng trắng: 200-255 (Nếu phòng tối, hãy hạ xuống 150-180)
    _, mask = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
    
    # Dọn dẹp các đốm trắng nhỏ bằng phép toán hình thái học
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Lấy vùng trắng có diện tích lớn nhất
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 1000: # Chỉ nhận nếu đủ to
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                return cx, c, mask
    return None, None, mask

cap = cv2.VideoCapture(1)
cap.set(3, 640); cap.set(4, 480)

print("Đang chạy chế độ Test Màu Trắng. Nhấn 'Q' để thoát.")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    H, W = frame.shape[:2]
    img_center_x = W // 2
    
    # Bước 1: Quét ArUco (Chính xác cao)
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    
    # Bước 2: Quét vùng trắng (Hỗ trợ tìm kiếm)
    white_cx, white_contour, white_mask = get_white_blob_center(frame)
    
    signal = "SEARCHING: NO_TARGET"
    color = (255, 255, 255)

    if ids is not None and ids[0] == 0:
        # --- CHẾ ĐỘ ARUCO DOCKING ---
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, mtx, dist)
        x_m, z_m = tvecs[0][0][0], tvecs[0][0][2]
        
        aruco.drawDetectedMarkers(frame, corners)
        
        if z_m <= TARGET_Z + 0.005:
            signal = f"FINAL_STOP | {z_m*100:.1f}cm"
            color = (0, 0, 255)
        else:
            if x_m > X_DEADZONE: signal = "PRECISION_STRAFE_RIGHT"
            elif x_m < -X_DEADZONE: signal = "PRECISION_STRAFE_LEFT"
            else: signal = "PRECISION_MOVE_FORWARD"
            color = (0, 255, 0)
            
    elif white_cx is not None:
        # --- CHẾ ĐỘ BÁM VÙNG TRẮNG ---
        cv2.drawContours(frame, [white_contour], -1, (255, 0, 0), 2)
        offset = white_cx - img_center_x
        
        if abs(offset) > 30:
            signal = "WHITE_TARGET: STRAFE_RIGHT" if offset > 0 else "WHITE_TARGET: STRAFE_LEFT"
        else:
            signal = "WHITE_TARGET: CENTERED -> APPROACH"
        color = (255, 255, 0)
    else:
        # --- KHÔNG THẤY GÌ ---
        signal = "SIGNAL_SCAN_START"

    # Hiển thị HUD
    cv2.line(frame, (img_center_x, 0), (img_center_x, H), (100, 100, 100), 1)
    cv2.putText(frame, signal, (10, H-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    cv2.imshow("Mecanum Vision - WHITE TEST", frame)
    # cv2.imshow("White Mask Debug", white_mask) # Bật lên để xem vùng trắng máy đang thấy
    
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()