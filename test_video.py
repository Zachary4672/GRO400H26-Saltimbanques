from ultralytics import YOLO
import cv2

model = YOLO("jelly_bean1.pt")  # nano = plus léger

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.7)

    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0]
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"{cx},{cy}", (cx+5, cy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    key = cv2.waitKey(2) & 0xFF

    if key == ord('p'):
        print("Allo")

    annotated = results[0].plot()


    cv2.imshow("YOLO", annotated)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()