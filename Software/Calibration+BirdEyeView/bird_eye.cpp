// OpenCV imports
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// C++ imports
#include <iostream>

// namespaces
using namespace std;
using namespace cv;
#define PI 3.1415926


int frameWidth = 640;
int frameHeight = 480;

/*
 * This code illustrates bird's eye view perspective transformation using opencv
 * Paper: Distance Determination for an Automobile Environment using Inverse Perspective Mapping in OpenCV  
 * Link to paper: https://www.researchgate.net/publication/224195999_Distance_determination_for_an_automobile_environment_using_Inverse_Perspective_Mapping_in_OpenCV
 * Code taken from: http://www.aizac.info/birds-eye-view-homography-using-opencv/
 */ 

int main(int argc, char const *argv[]) {
	
	if(argc < 2) {
      cerr << "Usage: " << argv[0] << " /path/to/video/" << endl;
      cout << "Exiting...." << endl;
      return -1;
    }

    // get file name from the command line
    string filename = argv[1];
    
    // capture object
    VideoCapture capture(filename);
  
    // mat container to receive images
    Mat source, destination;
    
    // check if capture was successful
    if( !capture.isOpened()) throw "Error reading video";


	int alpha_ = 90, beta_ = 90, gamma_ = 90;
	int f_ = 500, dist_ = 500;

	namedWindow("Result", 1);

	createTrackbar("Alpha", "Result", &alpha_, 180);
	createTrackbar("Beta", "Result", &beta_, 180);
	createTrackbar("Gamma", "Result", &gamma_, 180);
	createTrackbar("f", "Result", &f_, 2000);
	createTrackbar("Distance", "Result", &dist_, 2000);

	while( true ) {
		
		capture >> source;

		resize(source, source,Size(frameWidth, frameHeight));

		double focalLength, dist, alpha, beta, gamma; 

		alpha =((double)alpha_ -90) * PI/180;
		beta =((double)beta_ -90) * PI/180;
		gamma =((double)gamma_ -90) * PI/180;
		focalLength = (double)f_;
		dist = (double)dist_;

		Size image_size = source.size();
		double w = (double)image_size.width, h = (double)image_size.height;


		// Projecion matrix 2D -> 3D
		Mat A1 = (Mat_<float>(4, 3)<< 
			1, 0, -w/2,
			0, 1, -h/2,
			0, 0, 0,
			0, 0, 1 );

	
		// Rotation matrices Rx, Ry, Rz

		Mat RX = (Mat_<float>(4, 4) << 
			1, 0, 0, 0,
			0, cos(alpha), -sin(alpha), 0,
			0, sin(alpha), cos(alpha), 0,
			0, 0, 0, 1 );

		Mat RY = (Mat_<float>(4, 4) << 
			cos(beta), 0, -sin(beta), 0,
			0, 1, 0, 0,
			sin(beta), 0, cos(beta), 0,
			0, 0, 0, 1	);

		Mat RZ = (Mat_<float>(4, 4) << 
			cos(gamma), -sin(gamma), 0, 0,
			sin(gamma), cos(gamma), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1	);


		// R - rotation matrix
		Mat R = RX * RY * RZ;

		// T - translation matrix
		Mat T = (Mat_<float>(4, 4) << 
			1, 0, 0, 0,  
			0, 1, 0, 0,  
			0, 0, 1, dist,  
			0, 0, 0, 1); 
		
		// K - intrinsic matrix 
		Mat K = (Mat_<float>(3, 4) << 
			focalLength, 0, w/2, 0,
			0, focalLength, h/2, 0,
			0, 0, 1, 0
			); 


		Mat transformationMat = K * (T * (R * A1));

		warpPerspective(source, destination, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);

		imshow("Result", destination);
		waitKey(100);
	}


	return 0;
}