import cv2
import depthai as dai
import time
pipeline = dai.Pipeline()

camRgb = pipeline.createColorCamera()
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setVideoSize(3840, 2160)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.video.link(xoutRgb.input)

oak_camera = dai.Device(pipeline)
qRGB = oak_camera.getOutputQueue(name="rgb", maxSize=10, blocking=False)

for i in range(10):
    inRgb = qRGB.get()
    frame = inRgb.getCvFrame()
    t = int(time.time())
    cv2.imwrite(f"{t}_Rgb.png", frame)
