    
import cv2
import numpy as np



gotten = False
M = np.zeros((3,3))
def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]
refPt = []
cap = cv2.VideoCapture(0)
def click_and_crop(event, x, y, flags, param):
	# grab references to the global variables
    global refPt, cropping, gotten, M
	# if the left mouse button was clicked, record the starting
	# (x, y) coordinates and indicate that cropping is being
	# performed
    if event == cv2.EVENT_LBUTTONDOWN:
        cropping = True
        
	# check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
		# record the ending (x, y) coordinates and indicate that
		# the cropping operation is finished
        refPt.append((x, y))
        print(refPt)
        if(len(refPt) == 4):
            src = np.float32([refPt[0], refPt[1], refPt[2], refPt[3]])    # anti clockwise, start from left undermost
            dst = np.float32([refPt[0], refPt[1], [refPt[1][0],refPt[2][1]], [refPt[0][0],refPt[3][1]]])

            M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
            Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation
            print(M)
            gotten = True
            np.savetxt('wrap_perspective.out', M, delimiter=',')   # X is an array
            # warped_img = cv2.warpPerspective(image, M, (IMAGE_W, IMAGE_H)) # Image warping
        cropping = False
		# draw a rectangle around the region of interest                                              
def main(matrix_coefficients, distortion_coefficients): 
     
    while True:
        ret, frame = cap.read()
        print(ret)
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        cv2.imshow('frame', frame)
        cv2.setMouseCallback("frame", click_and_crop)
        if(gotten):
            warped_img = cv2.warpPerspective(frame, M,800,640)) # Image warping
            cv2.imshow('bird', warped_img)
            
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
    cap.release()
    cv2.destroyAllWindows()
m, d = load_coefficients("calib.txt")
main(m,d)