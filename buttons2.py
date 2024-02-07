#tried to add buttons in pysimplegui to connect to multiple cameras. there is something wrong witht the indenation at line 49 when i try to run it.
#this file is called buttons.py in terminal

import PySimpleGUI as sg
import cv2
import numpy as np
import time

def main():

	sg.theme("LightGreen")

    # Define the window layout
	layout =[
		[sg.Text("Cameras", size=(60, 1), justification="center")],
		[sg.Image(filename="", key="-IMAGE-")],
		[sg.Button("Cam 0", size=(10, 1), font='Helvetic 14')],
		[sg.Button("Cam 1", size=(10, 1), font='Helvetic 14')],
		[sg.Button("Cam 2", size=(10, 1), font='Helvetic 14')],
		[sg.Button("Cam 3", size=(10, 1), font='Helvetic 14')],
		[sg.Button("Cam 4", size=(10, 1), font='Helvetic 14')],
		[sg.Button("Stop", size=(10, 1), font='Helvetic 14')],
		[sg.Button("Exit", size=(10, 1), font='Helvetic 14')],
	]
	window = sg.Window('Mastitis camera 0', layout)
	window1 = sg.Window('Mastitis camera 1', layout)
	window2 = sg.Window('Mastitis camera 2', layout)
	window3 = sg.Window('Mastitis camera 3', layout)
	window4 = sg.Window('Mastitis camera 4', layout)

  # Event Loop read and display frames, operate the GUI
  # mixed with pysimple gui demo_opencv_webcam.py
	video_capture_0 = cv2.VideoCapture(0)
	video_capture_1 = cv2.VideoCapture(1)
	video_capture_2 = cv2.VideoCapture(2)
	video_capture_3 = cv2.VideoCapture(3)
	video_capture_4 = cv2.VideoCapture(4)
	open = False
    
	while True:
    	# Capture frame-by-frame
		ret, frame = window.read(timeout=20)
    		ret0, frame0 = window0.read(timeout=20)
    		ret1, frame1 = window1.read(timeout=20)
    		ret2, frame2 = window2.read(timeout=20)
    		ret3, frame3 = window3.read(timeout=20)
    		ret4, frame4 = window4.read(timeout=20)
		
    		if event == 'Exit' or event == sg.WIN_CLOSED:
    			return
			
    	#Display the resulting frame
		elif ret0 == 'Cam 0':
    			cv2.imshow('Cam 0', frame0)
			
		elif ret1 == 'Cam 1':
        		cv2.imshow('Cam 1', frame1)
			
		elif ret2 == 'Cam 2':
			cv2.imshow('Cam 2', frame2)
			
		elif ret3 == 'Cam 3':
			cv2.imshow('Cam 3', frame3)
			
		elif ret4 == 'Cam 4':
			cv2.imshow('Cam 4', frame4)    
            
	window.close()

main()
