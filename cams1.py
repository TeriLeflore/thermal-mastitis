#same as EID_camcap.py just switched some of the code around




from asyncio import get_event_loop
from serial_asyncio import open_serial_connection
import cv2
import datetime
import depthai as dai

portname = '/dev/ttyUSB0'
baudrate = 9600
timeout = 1

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

async def run():
    reader, writer = await open_serial_connection(url=portname, baudrate=baudrate)
    cam = [
      #cv2.VideoCapture(0),
      cv2.VideoCapture("/dev/v4l/by-id/usb-FLIR_Boson_196769-video-index0"),
      cv2.VideoCapture("/dev/v4l/by-id/usb-FLIR_Boson_97206-video-index0"),
      cv2.VideoCapture("/dev/v4l/by-id/usb-FLIR_Boson_196764-video-index0"),
      ]
    while True:
        
        #cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
        #cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        #cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        eid = await reader.readline()
        eid_list = [eid.decode().strip()]
        datetime_list = str(datetime.datetime.now()).split(" ")
        eid_list.extend(datetime_list)
        img_name = "_".join(eid_list) + ".tiff"
        print(img_name)
        results = []
        for vidcapture in cam:
           ret, frame = vidcapture.read()
           results.append( [ret,frame] )
        for number, (ret, frame) in enumerate(results):
           if ret:
              cv2.imwrite(img_name, frame)
              #cv2.imshow(f'Cam {number}', frame)
        #cam.release()

        inRgb = qRGB.get()
        frame = inRgb.getCvFrame()
        cv2.imwrite(f"{img_name}_Rgb.png", frame)

    for vidcapture in cam:
        cam.release()

loop = get_event_loop()
loop.run_until_complete(run())
