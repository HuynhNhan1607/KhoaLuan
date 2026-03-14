import cv2
import numpy as np
import os
import glob

# THÔNG SỐ CẦN THIẾT
CHECKERBOARD = (7, 7) # Cho bàn cờ 8x8 ô
SQUARE_SIZE = 0.01   # VÍ DỤ: 2.5cm = 0.025m (Thay bằng số bạn đo được)

# Chuẩn bị tọa độ thực tế
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

objpoints = [] # Điểm 3D thực tế
imgpoints = [] # Điểm 2D trên ảnh

# Đường dẫn ảnh: dùng relative path, chạy từ thư mục mini_server/camera/
path_images = os.path.join(os.path.dirname(__file__), "images", "*.jpg")
images = glob.glob(path_images)

if len(images) == 0:
    print("Không tìm thấy ảnh nào trong thư mục!")
    print(f"  → Đang tìm: {path_images}")
else:
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Tìm góc bàn cờ
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret == True:
            print(f"Đã xử lý thành công ảnh: {fname}")
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            print(f"BỎ QUA: Thuật toán không tìm thấy đủ 7x7 góc trong ảnh {fname}")

    # TÍNH TOÁN MA TRẬN
    if len(objpoints) > 0:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        
        out_dir = os.path.dirname(__file__)

        # Lưu .npz cho Python (pose_estimate.py)
        npz_path = os.path.join(out_dir, "calib_data.npz")
        np.savez(npz_path, mtx=mtx, dist=dist)
        print(f"Đã lưu: {npz_path}")

        # Lưu .yaml cho C++ OpenCV FileStorage (pose_estimate_service)
        # cv::FileStorage KHÔNG đọc được .npz (NumPy binary) → cần YAML
        yaml_path = os.path.join(out_dir, "calib_data.yaml")
        fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
        fs.write("mtx", mtx)
        fs.write("dist", dist)
        fs.release()
        print(f"Đã lưu: {yaml_path}")

        print("\n--- CHÚC MỪNG ---")
        print("Ma trận Camera (mtx):\n", mtx)
        print("Hệ số méo (dist):\n", dist)
    else:
        print("Thất bại! Không có ảnh nào đủ tiêu chuẩn để calibrate.")