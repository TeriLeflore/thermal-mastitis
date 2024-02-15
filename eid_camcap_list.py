# Test on 2/15

#same as cam1.py just switched some of the words around


from asyncio import get_event_loop
from serial_asyncio import open_serial_connection
import cv2
import datetime
import depthai as dai

portname = '/dev/ttyUSB0'
baudrate = 9600
timeout = 1

#OAK-D pipeline
pipeline = dai.Pipeline()

camRgb = pipeline.createColorCamera()
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setVideoSize(3840, 2160)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.video.link(xoutRgb.input)

oak_camera = dai.Device(pipeline)
qRGB = oak_camera.getOutputQueue(name="rgb", maxSize=10, blocking=False)


#Videocapture devices (thermal)
cap0 = cv2.VideoCapture("/dev/v4l/by-id/usb-FLIR_Boson_196769-video-index0")
cap1 = cv2.VideoCapture("/dev/v4l/by-id/usb-FLIR_Boson_97206-video-index0")
cap2 = cv2.VideoCapture("/dev/v4l/by-id/usb-FLIR_Boson_196764-video-index0")

#source = vid_capdevices
#def sixteenbit_cap(source):
    #cam = cv2.Videocapture(source)
    #cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
    #cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    #cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
    #cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)

async def run():
    reader, writer = await open_serial_connection(url=portname, baudrate=baudrate)
   
    while True:
        
        eid = await reader.readline()
        eid_list = [eid.decode().strip()]
        datetime_list = str(datetime.datetime.now()).split(" ")
        eid_list.extend(datetime_list)
        img_name = "_".join(eid_list) + ".tiff"
        print(img_name)

	#FLIR Boson camera 769 (24mm)
        ret0, frame0 = cap0.read()
        if ret0: 
              cv2.imshow('Cam 0',frame0)
              cv2.imwrite("B769"+ img_name, frame0)
        cap0.release()
        
	#FLIR Boson camera 206 (6mm)
        ret1, frame1 = cap1.read()
        if ret1:
              cv2.imshow('Cam 1',frame1)
              cv2.imwrite("B206" + img_name, frame1)     
        cap1.release()

	#FLIR Boson camera 764
        ret2, frame2 = cap2.read()
        if ret2:
              cv2.imshow('Cam 2',frame2)
              cv2.imwrite("B764" + img_name, frame2)
              
        cap2.release()

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

	#OAK-D 
        inRgb = qRGB.get()
        frame = inRgb.getCvFrame()
        cv2.imwrite(f"{img_name}_Rgb.png", frame)

    cv2.destroyAllWindows()

loop = get_event_loop()
loop.run_until_complete(run())
