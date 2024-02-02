import cv2

frame0 = cv2.VideoCapture(0)
frame1 = cv2.VideoCapture(1)
# frame2 = cv2.VideoCapture(2)
# frame3 = cv2.VideoCapture(3)
while 1:
    
    ret0, img0 = frame0.read()
    ret1, img00 = frame1.read()
    # ret2, img000 = frame2.read()
   #  ret3, img0000 = frame3.read()
    img1 = cv2.resize(img0, (360, 240))
    img2 = cv2.resize(img00,(360, 240))
   #  img3 = cv2.resize(img000,(360, 240))
   #  img4 = cv2.resize(img0000,(360, 240))
    if (frame0):
        cv2.imshow('img1' ,img1)
    if (frame1):
        cv2.imshow('img2' ,img2)
   #  if (frame2):
        cv2.imshow('img3' ,img3)
   #  if (frame3):
        cv2.imshow('img4' ,img4)
    
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    
frame0.release()
frame1.release()
# frame2.release()
# frame3.release()
cv2.destroyAllWindows()