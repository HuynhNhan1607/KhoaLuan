import cv2

cap = cv2.VideoCapture(1)
count = 0

print("Nhấn 'S' để lưu ảnh, 'Q' để thoát")

while True:
    ret, frame = cap.read()
    cv2.imshow("Calibration Setup", frame)
    
    key = cv2.waitKey(1)
    if key & 0xFF == ord('s'):
        cv2.imwrite(f"C:/Users/nhatm/Downloads/camera/images/image_{count}.jpg", frame)
        print(f"Đã lưu ảnh thứ {count}")
        count += 1
    elif key & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()