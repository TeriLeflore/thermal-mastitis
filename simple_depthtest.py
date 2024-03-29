
#this code takes disparity color and rgb photos everytime the tag read

#mar 19 changed code
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

#Define a pipeline (This is an empty pipeline object)
pipeline = dai.Pipeline()
   
# Configure nodes for mono camera, color camera, and depth
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)
xoutRgb = pipeline.create(dai.node.XLinkOut)

    
xout.setStreamName("disparity")
xoutRgb.setStreamName("rgb")

#Properties    
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setVideoSize(3840,2160)

#left.setBoardSocket(dai.CameraBoardSocket.CAM_B) #left camera
#right.setBoardSocket(dai.CameraBoardSocket.CAM_C) #right camera

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
#depth.setExtendedDisparity(extended_disparity)
#depth.setSubpixel(subpixel)
   
#connect camera output to input
# Configure left and right cameras to work as a stereo pair
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)
camRgb.video.link(xoutRgb.input)


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

#Loop through both OAK-D cameras
#oak_cam_list = dai.Device.getAllAvailableDevices()
#for oakcam in oak_cam_list:
    #device_info = dai.DeviceInfo(oakcam.mxid)
    #device = dai.Device(pipeline, device_info)


# initialize a device with pipeline and start it (device = OAK-D camera)
#device = dai.Device(pipeline)
device = dai.Device(pipeline, usb2Mode=True) #communicate vid USB2
    #formula for depth from disparty
    #calibData= device.readCalibration()
    #intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C)
    #print("Right mono camera focal leangth in pixels:", intrinsics[0][0]
  
#From this point on, the pipeline will be running on the device, producing the results we requested. Lets grab them
# defines a host side output queues to acces the produced results from xlinkout nodes
    # Create a receive queue for each stream
q = device.getOutputQueue(name="disparity", maxSize= 8, blocking=False)
qRGB = device.getOutputQueue(name="rgb", maxSize= 8, blocking=False)

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

            
     # the tryget method retureslatest results or none if the queue is empty   
        inDisparity = q.get() 
        frame = inDisparity.getFrame()
        frame = (frame * (255 /depth.initialConfig.getMaxDisparity())).astype(np.uint8)

        #cv2.imshow("disparity", frame)

        # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        #cv2.imshow("disparity_color", frame)
        cv2.imwrite("_disparity_" + img_name + ".png", frame) #change

        inRgb=qRGB.get()
        Rframe = inRgb.getCvFrame()
        cv2.imwrite(f"{img_name}_Rgb.png", Rframe)
         
        if cv2.waitKey(1) & 0xff == ord('q'):
                break
        

    cv2.destroyAllWindows()

loop = get_event_loop()
loop.run_until_complete(run())
