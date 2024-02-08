'''EID-imager, sortware to take thermal images when triggered by an EID cattle tag
'''


from asyncio import get_event_loop
from serial_asyncio import open_serial_connection
import cv2
import datetime

portname = '/dev/ttyUSB0'
baudrate = 9600
timeout = 1


async def run():
    reader, writer = await open_serial_connection(url=portname, baudrate=baudrate)
    cam = [
      cv2.VideoCapture(0),
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
        results = []
        for vidcapture in cam:
           ret, frame = vidcapture.read()
           results.append( [ret,frame] )
        datetime_list = str(datetime.datetime.now()).split(" ")
        eid_list.extend(datetime_list)
        img_name = "_".join(eid_list) + ".tiff"
        print(img_name)
        for number, (ret, frame) in enumerate(results):
           if ret:
              cv2.imwrite(img_name, frame)
              #cv2.imshow(f'Cam {number}', frame)
        #cam.release()
    for vidcapture in cam:
        cam.release()

loop = get_event_loop()
loop.run_until_complete(run())
