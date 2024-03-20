# this added multiple OAK-D devices
#not working: ERROR 
#  File "/home/teri/Mastitis/context.py", line 130, in run
    #if q_rgb.has():
#RuntimeError: Communication exception - possible device error/misconfiguration. Original #message 'Couldn't read data from stream: 'rgb' (X_LINK_ERROR)'

import contextlib
import datetime
from asyncio import get_event_loop
import time
import serial
import cv2
import depthai as dai
from serial_asyncio import open_serial_connection
import numpy as np

# Tag reader variables
portname = '/dev/ttyUSB0'
baudrate = 9600
timeout = 1

# IR camera variables
ir_camera_list = [("B769","/dev/v4l/by-id/usb-FLIR_Boson_196769-video-index0"),
		("B206","/dev/v4l/by-id/usb-FLIR_Boson_97206-video-index0"),
		("B764","/dev/v4l/by-id/usb-FLIR_Boson_196764-video-index0")]	


# Better handling for occulusions:
lr_check = True


def createPipeline():
    # Start defining a pipeline
    pipeline = dai.Pipeline()
    # Define a source - color camera
    camRgb = pipeline.create(dai.node.ColorCamera)

    camRgb.setPreviewSize(300, 300)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)

    # Create output
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.preview.link(xoutRgb.input)

    return pipeline
    
def set_mode(cam, mode):
    if mode == "16bit":
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        return cam
    elif mode == "8bit":
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('h','2','6','4'))
        cam.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        return cam



with contextlib.ExitStack() as stack:
    deviceInfos = dai.Device.getAllAvailableDevices()
    #usbSpeed = dai.UsbSpeed.SUPER
    usb2_mode = True
    openVinoVersion = dai.OpenVINO.Version.VERSION_2021_4

    qRgbMap = []
    devices = []

    for deviceInfo in deviceInfos:
        deviceInfo: dai.DeviceInfo
        device: dai.Device = stack.enter_context(dai.Device(openVinoVersion, deviceInfo, usb2_mode))
        devices.append(device)
        print("===Connected to ", deviceInfo.getMxId())
        mxId = device.getMxId()
        cameras = device.getConnectedCameras()
        usbSpeed = device.getUsbSpeed()
        eepromData = device.readCalibration2().getEepromData()
        print("   >>> MXID:", mxId)
        print("   >>> Num of cameras:", len(cameras))
        print("   >>> USB speed:", usbSpeed)
        if eepromData.boardName != "":
            print("   >>> Board name:", eepromData.boardName)
        if eepromData.productName != "":
            print("   >>> Product name:", eepromData.productName)

        pipeline = createPipeline()
        device.startPipeline(pipeline) 

        # Output queue will be used to get the rgb frames from the output defined above
        q_rgb = device.getOutputQueue(name="rgb", maxSize=8, blocking=False)
        stream_name = "rgb-" + mxId + "-" + eepromData.productName
        qRgbMap.append((q_rgb, stream_name))
        
# define event loop
async def run():
    reader, writer = await open_serial_connection(url=portname, baudrate=baudrate)

    tmpData=serial.Serial('/dev/ttyACM0', 115200)
    time.sleep(1)

    while True:
    
        # when Cattle tag is read create a tag and timestamp string
        eid = await reader.readline()
        eid_list = [eid.decode().strip()]
        datetime_list = str(datetime.datetime.now()).split(" ")
        eid_list.extend(datetime_list)
        # temp, pres, humidity data
        dataPacket=tmpData.readline()
        dataPacket=str(dataPacket,'utf-8')
        dataPacket=dataPacket.strip('\r\n')
        eid_list.extend(dataPacket)
        img_name = "_".join(eid_list)
        print(img_name)
	
	# Loop through the IR cameras collecting 1 unscaled 16-bit and scaled 8-bit images
        for camera in ir_camera_list:
            camobj = cv2.VideoCapture(camera[1])
            for mode in ["16bit", "8bit"]:
            	caminst = set_mode(camobj, mode)
            	ret0, frame0 = caminst.read()
            	if ret0: 
                    #cv2.imshow(camera[0], frame0) # uncomment for image preview screen
                    cv2.imwrite(camera[0] + "_" + mode + "_" + img_name + ".tiff", frame0)
            camobj.release()

    #while True:
        for q_rgb, stream_name in qRgbMap:
            if q_rgb.has():  #X_LINK_ERROR cant read data from stream
                cv2.imshow(stream_name, q_rgb.get().getCvFrame())
                #cv2.imwrite(stream_name + "_disparity_" + ".png", q_rgb.get().getCvFrame())
                

        if cv2.waitKey(1) == ord('q'):
            break
    cv2.destroyAllWindows()

loop = get_event_loop()
loop.run_until_complete(run())
