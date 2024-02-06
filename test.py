#code to test the EID tag reader with theramal images 

import asyncio
import serial_asyncio

class OutProtocol(asyncio.Protocol):
	def camera_snap16 (self, transport):
		self.transport =transport
		
	def camera_snap8 (self, data):
		self.pause_camera()


async def run():
    reader, writer = await serial_asyncio.create_serial_connection (loop, OutProtocol, url = '/dev/ttyUSB0', baudrate=9600)
    
    while True:
        #cam = cv2.VideoCapture(0)
        cam = cv2.VideoCapture('/dev/video2')
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

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
loop.close 
