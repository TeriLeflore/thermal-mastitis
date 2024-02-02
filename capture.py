import cv2
import time

num = 1
cap = cv2.VideoCapture(0)

while True:
		ret, img = cap.read()
		cv2.imshow('Camera 0', img)
		if cv2.waitKey(1) & 0xFF == ord('c'):
				cv2.imwrite('/home/mastitis/images/'+str(num)+'.jpg', img)
				print('Capture '+str(num)+' Successful!')
				num = num + 1
		if num==4:
				break
cap.release()
cv2.destroyAllWindows()
		
