#Adams corrected eid_camcap_list.py


import datetime
from asyncio import get_event_loop

import cv2
import depthai as dai
from serial_asyncio import open_serial_connection


# Tag reader variables
portname = '/dev/ttyUSB0'
baudrate = 9600
timeout = 1

# IR camera variables
ir_camera_list = [("B769","/dev/v4l/by-id/usb-FLIR_Boson_196769-video-index0"),
		("B206","/dev/v4l/by-id/usb-FLIR_Boson_97206-video-index0"),
		("B764","/dev/v4l/by-id/usb-FLIR_Boson_196764-video-index0")]


def create_oak_pipe() -> dai.Pipeline:
    """creates an Oak Camera pipeline instance
    
    Args: None
    
    Returns: depthai.Pipeline
    
    """
    pipeline = dai.Pipeline()
    camRgb = pipeline.createColorCamera()
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setVideoSize(3840, 2160)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.video.link(xoutRgb.input)
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

# set up oak-D pipeline
pipeline = create_oak_pipe()

# define event loop
async def run():
    reader, writer = await open_serial_connection(url=portname, baudrate=baudrate)
   
    while True:
        # when Cattle tag is read create a tag and timestamp string
        eid = await reader.readline()
        eid_list = [eid.decode().strip()]
        datetime_list = str(datetime.datetime.now()).split(" ")
        eid_list.extend(datetime_list)
        img_name = "_".join(eid_list)
        print(img_name)
	
	# Loop through the IR cameras collecting 1unscaled 16-bit and scaled 8-bit images
        for camera in ir_camera_list:
            camobj = cv2.VideoCapture(camera[1])
            for mode in ["16bit", "8bit"]:
            	caminst = set_mode(camobj, mode)
            	ret0, frame0 = caminst.read()
            	if ret0: 
                    #cv2.imshow(camera[0], frame0) # uncomment for image preview screen
                    cv2.imwrite(camera[0] + "_" + mode + "_" + img_name + ".tiff", frame0)
            camobj.release()
 
	#Aquire Color images from OAK-D cameras
        oak_cam_list = dai.Device.getAllAvailableDevices()
        for oakcam in oak_cam_list:
            device_info = dai.DeviceInfo(oakcam.mxid)
            oak_camera = dai.Device(pipeline, device_info)
            qRGB = oak_camera.getOutputQueue(name="rgb", maxSize=10, blocking=False)
            inRgb = qRGB.get()
            frame = inRgb.getCvFrame()
            cv2.imwrite(oakcam.mxid + "_rgb_" + img_name + ".png", frame)
       
        
        if cv2.waitKey(1) & 0xff == ord('q'):
            break


    cv2.destroyAllWindows()

loop = get_event_loop()
loop.run_until_complete(run())
