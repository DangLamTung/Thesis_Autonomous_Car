import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = np.zeros((300,512,3), np.uint8)
cropped = False
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('a','image',90,180,nothing)
cv2.createTrackbar('b','image',90,180,nothing)
cv2.createTrackbar('y','image',90,180,nothing)
cv2.createTrackbar('f','image',0,2000,nothing)
cv2.createTrackbar('dist','image',0,2000,nothing)

cap = cv2.VideoCapture(1)
# process frames until user exits
while True:
	# capture the next image
	#img_input = input.Capture()
    ret, frame = cap.read()
    a = cv2.getTrackbarPos('a','image')
    g = cv2.getTrackbarPos('b','image')
    b = cv2.getTrackbarPos('y','image')
    f = cv2.getTrackbarPos('f','image')
    dist = cv2.getTrackbarPos('dist','image')

    a =(a -90) * np.pi/180
    beta =(g -90) * np.pi/180
    gamma =(b -90) * np.pi/180
    focalLength =  f


    w = frame.shape[0]
    h = frame.shape[1]
    A1 = np.array([[1, 0, -w/2],[0, 1, -h/2],[0,0,0],[0,0,1]])
    RX = np.array([[1, 0, 0, 0],[0, np.cos(a), -np.sin(a), 0],[0, np.sin(a), np.cos(a), 0],[0, 0, 0, 1 ]])
    RY = np.array([ [np.cos(beta), 0, -np.sin(beta), 0],[0, 1, 0, 0],[np.sin(beta), 0, np.cos(beta), 0],[0, 0, 0, 1]]	)
    RZ = np.array([[np.cos(gamma), -np.sin(gamma), 0, 0],[np.sin(gamma), np.cos(gamma), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    R = RX.dot(RY.dot(RZ))

    T = np.array([[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, dist],[0, 0, 0, 1]]); 

    K = np.array([[focalLength, 0, w/2, 0],[0, focalLength, h/2, 0],[0, 0, 1, 0]]); 


    transformationMat = K.dot(T.dot( (R.dot(A1))))

    warped = cv2.warpPerspective(frame, transformationMat , (frame.shape[0]+200, frame.shape[1]))


    
    cv2.imshow('frame', frame)
    cv2.imshow("Result", warped)
      #  operations on the frame come here
    

    if(cropped):
      imCrop = warped[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
      cv2.imshow("Image", imCrop)
    key = cv2.waitKey(3) & 0xFF
    if key == ord('q'):  # Quit
        break
    if key == ord('c'):  # Quit
      r = cv2.selectROI(warped)

      imCrop = warped[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
      cv2.imshow("Image", imCrop)
      cropped = True
    if key == ord('s'):
      transform_file = open("transform_file.txt", "w")
      for row in transformationMat:
          np.savetxt(transform_file, row)
          transform_file.close()
      crop_file = open("crop_img.txt", "w")
      for row in r:
          np.savetxt(crop_file, row)
          crop_file.close()

cap.release()
cv2.destroyAllWindows()