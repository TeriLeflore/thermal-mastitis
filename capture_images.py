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
    while True:
        #cam = cv2.VideoCapture(0)
        cam = cv2.VideoCapture(2)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
        cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        eid = await reader.readline()
        eid_list = [eid.decode().strip()]
        ret, frame = cam.read()
        datetime_list = str(datetime.datetime.now()).split(" ")
        eid_list.extend(datetime_list)
        img_name = "_".join(eid_list) + ".tiff"
        print(img_name)
        cv2.imwrite(img_name, frame)
        cam.release()

loop = get_event_loop()
loop.run_until_complete(run())

# 16 bit not work ing
#see this https://github.com/LJMUAstroecology/flirpy/issues/7
