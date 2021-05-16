#!/usr/bin/python3
#
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import jetson.inference
import jetson.utils

import argparse
import sys
import cv2
from segnet_utils import *

# parse the command line
parser = argparse.ArgumentParser(description="Segment a live camera stream using an semantic segmentation DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.segNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="fcn-resnet18-voc", help="pre-trained model to load, see below for options")
parser.add_argument("--filter-mode", type=str, default="linear", choices=["point", "linear"], help="filtering mode used during visualization, options are:\n  'point' or 'linear' (default: 'linear')")
parser.add_argument("--visualize", type=str, default="overlay,mask", help="Visualization options (can be 'overlay' 'mask' 'overlay,mask'")
parser.add_argument("--ignore-class", type=str, default="void", help="optional name of class to ignore in the visualization results (default: 'void')")
parser.add_argument("--alpha", type=float, default=150.0, help="alpha blending value to use during overlay, between 0.0 and 255.0 (default: 150.0)")
parser.add_argument("--stats", action="store_true", help="compute statistics about segmentation mask class output")

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the segmentation network
net = jetson.inference.segNet(opt.network, sys.argv)

# set the alpha blending value
net.SetOverlayAlpha(opt.alpha)

# create buffer manager
buffers = segmentationBuffers(net, opt)

# create video sources & outputs
#input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)
out = cv2.VideoWriter('video_not_mask.avi', -1, 20.0, (640,480))


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
            # warped_img = cv2.warpPerspective(image, M, (IMAGE_W, IMAGE_H)) # Image warping
		cropping = False
		

cap = cv2.VideoCapture(0)
# process frames until user exits
while True:
	# capture the next image
	#img_input = input.Capture()
	ret, frame = cap.read()
        # operations on the frame come here
	cv2.imshow('frame', frame)
	cv2.setMouseCallback("frame", click_and_crop)
	if(gotten):
		warped_img = cv2.warpPerspective(frame, M,(frame.shape[1],frame.shape[0])) # Image warping
		cv2.imshow('bird', warped_img)
		img_input = jetson.utils.cudaFromNumpy(frame)
		# allocate buffers for this size image
		buffers.Alloc(img_input.shape, img_input.format)

		# process the segmentation network
		net.Process(img_input, ignore_class=opt.ignore_class)

		# generate the overlay
		if buffers.overlay:
			net.Overlay(buffers.overlay, filter_mode=opt.filter_mode)

		# generate the mask
		if buffers.mask:
			net.Mask(buffers.mask, filter_mode=opt.filter_mode)
		buffers.ComputeStats()
		#print(buffers.class_mask_np)
		
		# composite the images
		#if buffers.composite:
		#	jetson.utils.cudaOverlay(buffers.overlay, buffers.composite, 0, 0)
		#	jetson.utils.cudaOverlay(buffers.mask, buffers.composite, buffers.overlay.width, 0)
		mask = jetson.utils.cudaToNumpy(buffers.mask)
		resized = cv2.resize(mask, (frame.shape[1],frame.shape[0]), interpolation = cv2.INTER_AREA)
		#mask_demo = jetson.utils.cudaToNumpy(buffers.overlay)
		cv2.imshow('mask visualize', resized)
		warped_mask = cv2.warpPerspective(resized, M,(mask.shape[1],mask.shape[0]),flags = cv2.INTER_NEAREST) # Image warping
		#cv2.imshow('mask visualize birded', warped_mask )
		#warped_img = cv2.warpPerspective(mask, M,(mask.shape[1],mask.shape[0]),flags = cv2.INTER_NEAREST) # Image warping
		print(mask)
		cv2.imshow('bird', warped_mask )
		print(warped_img.shape)
	
		# render the output image
		#output.Render(buffers.output)

		# update the title bar
		#output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

		# print out performance info
		jetson.utils.cudaDeviceSynchronize()
		net.PrintProfilerTimes()

    # compute segmentation class stats
	#if opt.stats:
		
      
	
	
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
	key = cv2.waitKey(3) & 0xFF
	if key == ord('q'):  # Quit
		break
cap.release()
cv2.destroyAllWindows()    
	# exit on input/output EOS
	#if not input.IsStreaming() or not output.IsStreaming():
	#	break
