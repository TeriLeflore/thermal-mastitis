def read_frame(capture):
    capture.grab()
    ret, frame = capture.retrieve()
    print(frame)
    if not ret:
        print("empty frame")
        return
    return frame

import cv2 as cv
import datetime

if __name__ == "__main__":
    cap = cv.VideoCapture(0)
    assert cap.isOpened()
    print(cap)
    while True:
        frame = read_frame(cap)
        
        font = cv.FONT_HERSHEY_SCRIPT_COMPLEX
        dt = str(datetime.datetime.now())
        frame = cv.putText(frame, dt,
                           (10, 100),
                           font, 1,
                           (210, 155, 155),
                            4, cv.LINE_8)
        
        cv.imshow("Camera 0", frame)
        if cv.waitKey(1) ==27:
            break