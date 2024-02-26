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

# Connect to device and start pipeline
with dai.Device(pipeline) as oak_camera:

 # Output queue will be used to get the disparity frames from the outputs defined above
    q = oak_camera.getOutputQueue(name="disparity", maxSize=4, blocking=False)
    qRGB = oak_camera.getOutputQueue(name="rgb", maxSize=10, blocking=False)
    
    while True:
        inDisparity = q.get()  # blocking call, will wait until a new data has arrived
        #frame = q.getCvFrame()
        frame = inDisparity.getFrame()
        # Normalization for better visualization
        frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

        #cv2.imshow("disparity", frame)
        cv2.imwrite("disparity_depth.png", frame)

        # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        cv2.imshow("disparity_color", frame)
        cv2.imwrite("disparity_color.png", frame)

        inRgb = qRGB.get()
        frame = inRgb.getCvFrame()
        cv2.imwrite("Rgb.png", frame)

        if cv2.waitKey(1) == ord('q'):
            break


