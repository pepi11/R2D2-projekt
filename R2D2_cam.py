import cv2

CAMERA_URL = "http://192.168.0.164:81/stream"

def main():
    cap = cv2.VideoCapture(CAMERA_URL)

    if not cap.isOpened():
        print("❌ Nie można otworzyć strumienia")
        return

    print("📷 Otwieranie podglądu... [ESC aby zamknąć]")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Brak klatki — rozłączono?")
            break
        #frame = cv2.flip(frame, 1)  # odbicie poziome (mirror)
        frame = cv2.flip(frame, 0)

        cv2.imshow("ESP32-CAM", frame)

        if cv2.waitKey(1) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
