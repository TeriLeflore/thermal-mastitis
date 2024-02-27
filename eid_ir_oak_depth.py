# This is to test adding oak depth/disparity photos to code. It has not been tested to see if it works

import datetime
from asyncio import get_event_loop

import cv2
import depthai as dai
from serial_asyncio import open_serial_connection
#import hashlib

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
    # Define sources and outputs
    camRgb = pipeline.createColorCamera()
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camRgb.setVideoSize(3840, 2160)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    depth = pipeline.create(dai.node.StereoDepth)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xout = pipeline.create(dai.node.XLinkOut)

    xout.setStreamName("disparity")
    xoutRgb.setStreamName("rgb")

    # Properties
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")

    # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    depth.setLeftRightCheck(lr_check)
    depth.setExtendedDisparity(extended_disparity)
    depth.setSubpixel(subpixel)

    # Linking
    camRgb.video.link(xoutRgb.input)
    monoLeft.out.link(depth.left)
    monoRight.out.link(depth.right)
    depth.disparity.link(xout.input)
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
            q = oak_camera.getOutputQueue(name="disparity", maxSize=4, blocking=False)
            inDisparity = q.get()  # blocking call, will wait until a new data has arrived
            frame = inDisparity.getFrame()
            # Normalization for better visualization
            frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

            cv2.imshow("disparity", frame)
            cv2.imwrite("disparity_depth.png", frame)

            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            cv2.imshow("disparity_color", frame)
            cv2.imwrite("disparity_color.png", frame)
        
            qRGB = oak_camera.getOutputQueue(name="rgb", maxSize=10, blocking=False)
            inRgb = qRGB.get()
            frame = inRgb.getCvFrame()
            # print(hashlib.sha256(frame).hexdigest())
            cv2.imwrite(oakcam.mxid + "_rgb_" + img_name + ".png", frame)
       
        
        if cv2.waitKey(1) & 0xff == ord('q'):
            break


    cv2.destroyAllWindows()

loop = get_event_loop()
loop.run_until_complete(run())
