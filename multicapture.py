import cv2

frame0 = cv2.VideoCapture(0)
frame1 = cv2.VideoCapture(1)
# frame2 = cv2.VideoCapture(2)
# frame3 = cv2.VideoCapture(3)
while True:
    
   ret0, img0 = frame0.read()
   ret1, img1 = frame1.read()
    # ret2, img2 = frame2.read()
   #  ret3, img3 = frame3.read()
   #img0 = cv2.resize(img0, (360, 240))
   #img1 = cv2.resize(img1, (360, 240))
   #  img2 = cv2.resize(img2,(360, 240))
   #  img3 = cv2.resize(img3,(360, 240))
   if (frame0):
   	cv2.imshow('img0' ,img0)
   if (frame1):
   	cv2.imshow('img1' ,img1)
   #  if (frame2):
        #cv2.imshow('img2' ,img2)
   #  if (frame3):
        #cv2.imshow('img3' ,img3)
   if cv2.waitKey(1) & 0xff == 27:
        break
    
frame0.release()
frame1.release()
# frame2.release()
# frame3.release()
cv2.destroyAllWindows()

# doesn't work to open multiple webcams
