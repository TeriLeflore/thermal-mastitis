#kind of works put doesn't show video and shuts down

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
		
#Weights to use when blending depth/rgb image (should equal 1.0)
rgbWeight = 0.4
depthWeight = 0.6

def updateBlendWeights(percent_rgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image
    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percent_rgb)/100.0
    depthWeight = 1.0 - rgbWeight


# Optional. If set (True), the ColorCamera is downscaled from 1080p to 720p.
# Otherwise (False), the aligned depth is automatically upscaled to 1080p
downscaleColor = True
fps = 10
# The disparity is computed at this resolution, then upscaled to RGB resolution
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_800_P


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

# Create pipeline
pipeline = dai.Pipeline()
device= dai.Device()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

rgbOut = pipeline.create(dai.node.XLinkOut)
depthOut = pipeline.create(dai.node.XLinkOut)

rgbOut.setStreamName("rgb")
depthOut.setStreamName("depth")

#Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP) # 4056x3040

try:
    calibData = device.readCalibration2()
    lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.CAM_A)
    if lensPosition:
        camRgb.initialControl.setManualFocus(lensPosition)
except:
    raise
left.setResolution(monoResolution)
left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
left.setFps(fps)
right.setResolution(monoResolution)
right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
right.setFps(fps)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
# # 4056x3040
stereo.setOutputSize(1248, 936)

# Linking
camRgb.isp.link(rgbOut.input)
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.depth.link(depthOut.input)

with device:
    device.startPipeline(pipeline)
    #Print MxID, USB speed, and available cameras on the device
   # print ('MxID:' , device.getDeviceInfo().getMxId())
   # print('USB speed:', device.getUsbSpeed())
   # print('Connected cameras:', device.getConnectedCameras())
    
    frameRgb = None
    depthFrame = None

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    rgbWindowName = "rgb"
    depthWindowName = "depth"
    blendedWindowName = "rgb-depth"
    cv2.namedWindow(rgbWindowName)
    cv2.namedWindow(depthWindowName)
    cv2.namedWindow(blendedWindowName)
    cv2.createTrackbar('RGB Weight %', blendedWindowName, int(rgbWeight*100), 100, updateBlendWeights)

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
 
	#Aquire Color images from OAK-D cameras

        oak_cam_list = dai.Device.getAllAvailableDevices()
        for oakcam in oak_cam_list:
            device_info = dai.DeviceInfo(oakcam.mxid)
            oak_camera = dai.Device(pipeline, device_info)
        
            depthQueue = oak_camera.getOutputQueue(name="depth", maxSize=10, blocking=False)
            rgbQueue = oak_camera.getOutputQueue(name="rgb", maxSize=10, blocking=False)
        
            inRgb= rgbQueue.get()
            frameRgb = inRgb.getCvFrame()
            frameRgb = cv2.resize(frameRgb, (1248, 936), interpolation=cv2.INTER_NEAREST)
            #cv2.imshow(rgbWindowName, frameRgb)

            inDep= depthQueue.get()
            depthFrame = inDep.getFrame()
            depthFrame = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrame = cv2.equalizeHist(depthFrame)
            depthFrame = cv2.applyColorMap(depthFrame, cv2.COLORMAP_HOT)
            #cv2.imshow(depthWindowName, depthFrame)

        # Blend when both received
        if frameRgb is not None and depthFrame is not None:
            # Need to have both frames in BGR format before blending
            if len(depthFrame.shape) < 3:
                depthFrame = cv2.cvtColor(depthFrame, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(frameRgb, rgbWeight, depthFrame, depthWeight, 0)
            cv2.imshow(blendedWindowName, blended)
            frameRgb = None
            depthFrame = None        
            cv2.imwrite(oakcam.mxid + "_rgbBlend_" + img_name + ".png", blended)
            
         
        # Check for keyboard input
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            # Quit when q is pressed
                break
        elif key == ord('c'):
            ctrl = dai.CameraControl
            ctrl.setCaptureStill(True)
            controlQueue.send(ctrl)
        elif key == ord('t'):
            print("Autofocus trigger (and disable continuous)")
            ctrl = dai.CameraControl
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
            ctrl.setAutoFocusTrigger()
            controlQueue.send(ctrl)
            

    cv2.destroyAllWindows()

loop = get_event_loop()
loop.run_until_complete(run())

