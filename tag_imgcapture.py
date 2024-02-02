import asyncio
import serial_asyncio 
import cv2 as cv
import datetime

#portname = '/dev/cu.usbserial-142110'
portname = '/dev/ttyUSB0'
baudrate = 9600
timeout = 1

def read_frame(capture):
    capture.grab()
    ret, frame = capture.retrieve()
    print(frame)
    if not ret:
        print("empty frame")
        return
    return frame


if __name__ == "__main__":
    num = 1
    cap = cv.VideoCapture(0)
    assert cap.isOpened()
    print(cap)
    while True:
        frame = read_frame(cap)
        
        
        cv.imshow("Camera 0", frame)
        if cv.waitKey(1) ==27:
            cv.imwrite('/home/mastitis/images/'+str(num)+'.jpg', img)
				print('Capture '+str(num)+' Successful!')
				num = num + 1
		if num==3:
				break


async def images():
    reader, writer = await open_serial_connection(url=portname, baudrate=baudrate)
    while True:
        #cam = cv2.VideoCapture(0)
        cam = cv2.VideoCapture('/dev/video2')
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        eid = await reader.readline()
        eid_list = [eid.decode().strip()]
        ret, frame = cam.read()
        datetime_list = str(datetime.datetime.now()).split(" ")
        eid_list.extend(datetime_list)
        img_name = "_".join(eid_list) + ".tiff"
        print(img_name)
        cv2.imwrite(img_name, frame)
        cam.release()
        
loop = asyncio.get_event_loop()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

loop.run_until_complete(images())

# 16 bit not work ing
#see this https://github.com/LJMUAstroecology/flirpy/issues/7