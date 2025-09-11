import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# 1) ตั้ง FOURCC ให้ตรงกับที่กล้องรองรับ (ลอง 'MJPG' ก่อน)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
# 2) แล้วค่อยตั้งขนาด/เฟรมเรต
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

# ตรวจว่าตั้งได้จริงไหม
w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
print("applied:", f"{int(w)}x{int(h)}@{fps:.1f}",
      "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)]))
