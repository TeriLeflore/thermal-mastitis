import PySimpleGUI as sg
import cv2
import time

def main():

    sg.theme("LightGreen")

    # Define the window layout
    layout =[
            [sg.Text("Cameras", size=(60, 1), justification="center")],
            [sg.Image(filename="", key="-IMAGE-")],
            [sg.Button("Cam 0", size=(10, 1), font='Helvetic 14')],
            [sg.Button("Cam 1", size=(10, 1), font='Helvetic 14')],
            [sg.Button("Stop", size=(10, 1), font='Helvetic 14')],
            [sg.Button("Exit", size=(10, 1), font='Helvetic 14')],
    ]
    window = sg.Window('Mastitis cameras', layout)

  # Event Loop read and display frames, operate the GUI
  # mixed with pysimple gui demo_opencv_webcam.py
    cap = cv2.VideoCapture(0)
    open = False
    
    while True:
        event, values = window.read(timeout=20)
        if event == 'Exit' or event == sg.WIN_CLOSED:
            return

        elif event == 'Cam 0':
            open = True
        
        elif event == 'Cam 1':
            open = True

        elif event == 'Stop':
            recording = False
            img = np.full((480, 640), 255)
            # this is faster, shorter and needs less includes
            imgbytes = cv2.imencode('.png', img)[1].tobytes()
            window['image'].update(data=imgbytes)

        if open:
            ret, frame = cap.read()
            imgbytes = cv2.imencode('.png', frame)[1].tobytes()
            window['image'].update(data=imgbytes)
            
    window.close()

main()
