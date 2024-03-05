# This is to test adding oak depth/disparity photos to code. It has not been tested to see if it works

import datetime
from asyncio import get_event_loop
import time
import serial
import cv2
import depthai as dai
from serial_asyncio import open_serial_connection
import numpy as np
#import hashlib

# Tag reader variables
portname = '/dev/ttyUSB0'
baudrate = 9600
timeout = 1

# IR camera variables
ir_camera_list = [("B769","/dev/v4l/by-id/usb-FLIR_Boson_196769-video-index0"),
		("B206","/dev/v4l/by-id/usb-FLIR_Boson_97206-video-index0"),
		("B764","/dev/v4l/by-id/usb-FLIR_Boson_196764-video-index0")]	

def getFrame(queue):
  # Get frame from queue
  frame = queue.get()
  # Convert frame to OpenCV format and return
  return frame.getCvFrame()


def getMonoCamera(pipeline, isLeft):
  # Configure mono camera
  mono = pipeline.createMonoCamera()

  # Set Camera Resolution
  mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

  if isLeft:
      # Get left camera
      mono.setBoardSocket(dai.CameraBoardSocket.CAM_B)
  else :
      # Get right camera
      mono.setBoardSocket(dai.CameraBoardSocket.CAM_C)
  return mono


def getStereoPair(pipeline, monoLeft, monoRight):
    # Configure stereo pair for depth estimation
    stereo = pipeline.createStereoDepth()
    # Checks occluded pixels and marks them as invalid
    stereo.setLeftRightCheck(True)
    
    # Configure left and right cameras to work as a stereo pair
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    return stereo

def getrgbCamera(pipeline):
 
    #Define sources and outputs
    camRgb = pipeline.createColorCamera()
    #set resolution
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP) #properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setVideoSize(3840, 2160)
    return camRgb


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


if __name__ == '__main__':

    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft = True)
    monoRight = getMonoCamera(pipeline, isLeft = False)
    camRgb = getrgbCamera(pipeline)

    # Combine left and right cameras to form a stereo pair
    stereo = getStereoPair(pipeline, monoLeft, monoRight)
    
    
    # Set XlinkOut for disparity, rectifiedLeft, and rectifiedRight
    xoutDisp = pipeline.createXLinkOut()
    xoutDisp.setStreamName("disparity")
    
    xoutRectifiedLeft = pipeline.createXLinkOut()
    xoutRectifiedLeft.setStreamName("rectifiedLeft")

    xoutRectifiedRight = pipeline.createXLinkOut()
    xoutRectifiedRight.setStreamName("rectifiedRight")
    
    xoutRgb = pipeline.createXLinkOut()
    xoutRgb.setStreamName("rgb")

    stereo.disparity.link(xoutDisp.input)
    
    stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
    stereo.rectifiedRight.link(xoutRectifiedRight.input)
    
    camRgb.video.link(xoutRgb.input) #linking


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
            disparityQueue = oak_camera.getOutputQueue(name="disparity", maxSize=1, blocking=False)
            rectifiedLeftQueue = oak_camera.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
            rectifiedRightQueue = oak_camera.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)
            rgbQueue = oak_camera.getOutputQueue(name="rgb", maxSize=1, blocking=False)
        
            # Calculate a multiplier for colormapping disparity map
            disparityMultiplier = 255 / stereo.initialConfig.getMaxDisparity()

            cv2.namedWindow("Stereo Pair")
        
            # Variable use to toggle between side by side view and one frame view.
            sideBySide = False
        
                # Get disparity map
            disparity = getFrame(disparityQueue)
            
            # Colormap disparity for display
            disparity = (disparity * disparityMultiplier).astype(np.uint8)
            disparity = cv2.applyColorMap(disparity, cv2.COLORMAP_JET)
            
                # Get left and right rectified frame
            leftFrame = getFrame(rectifiedLeftQueue);
            rightFrame = getFrame(rectifiedRightQueue)
            
            if sideBySide:
                # Show side by side view
                imOut = np.hstack((leftFrame, rightFrame))
            else :
                # Show overlapping frames
                imOut = np.uint8(leftFrame/2 + rightFrame/2)
            
            
            imOut = cv2.cvtColor(imOut,cv2.COLOR_GRAY2RGB) 
            
            cv2.imshow("Stereo Pair", imOut)
            cv2.imshow("Disparity", disparity)
            #cv2.imwrite("stereo_pair" + ".png", imOut)
            #cv2.imwrite("disparity" + ".png", disparity) 
            cv2.imwrite(oakcam.mxid + "_stereo_pair_" + img_name + ".png", imOut) 
            cv2.imwrite(oakcam.mxid + "_disparity_" + img_name + ".png", disparity)
            
                      
            #Aquire Color images from OAK-D cameras
            qRGB = getFrame(rgbQueue)
            
            #cv2.imshow("RGB", qRGB)
            cv2.imwrite(oakcam.mxid + "_rgb_" + img_name + ".png", qRGB)
                #cv2.imwrite("color_image" + ".png", qRGB)

                # Check for keyboard input
            key = cv2.waitKey(1)
            if key == ord('q'):
            # Quit when q is pressed
                break
            elif key == ord('t'):
                # Toggle display when t is pressed
                sideBySide = not sideBySide


            #file_name = 'home/Mastitis/oakcam.mxid + _rgb_ + img_name + .png'
            #with open(file_name) as f:
                #data = f.read()
                #read_hash = hashlib.sha256(data).hexdigest()
                #print(read_hash)
   

    cv2.destroyAllWindows()

loop = get_event_loop()
loop.run_until_complete(run())
