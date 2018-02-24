// C++ libraries
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include<fstream>
#include <pylon/PylonIncludes.h>



// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using GenApi objects
using namespace GenApi;

// Namespace for using opencv objects.
using namespace cv;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
//static const uint32_t c_countOfImagesToGrab = 20;

#define INTRINSICFILENAME0 "/home/ms/Desktop/pylon-calibration/build/results/cam726/intrinsic1.txt"
#define INTRINSICFILENAME1 "/home/ms/Desktop/pylon-calibration/build/results/cam728/intrinsic2.txt"

#define DISTORTFILENAME0 "/home/ms/Desktop/pylon-calibration/build/results/cam726/distortion_coeffs1.txt"
#define DISTORTFILENAME1  "/home/ms/Desktop/pylon-calibration/build/results/cam728/distortion_coeffs2.txt"

#define ROTATIONFILENAME0  "/home/ms/Desktop/pylon-calibration/build/results/cam726/rotation1.txt"
#define ROTATIONFILENAME1  "/home/ms/Desktop/pylon-calibration/build/results/cam728/rotation2.txt"

#define TRANSLATIONFILENAME0  "/home/ms/Desktop/pylon-calibration/build/results/cam726/translation1.txt"
#define TRANSLATIONFILENAME1  "/home/ms/Desktop/pylon-calibration/build/results/cam728/translation2.txt"
#define ROWS 480
#define COLS 640


int main( int argc, char** argv )
{		
	unsigned long long int i, j;

	timespec tS,tE;
	tS.tv_sec = tS.tv_nsec = tE.tv_sec = tE.tv_nsec = 0;

    Pylon::PylonAutoInitTerm autoInitTerm;
          PylonInitialize();
          static const size_t c_maxCamerasToUse = 2;
         try
         {
              CTlFactory& tlFactory = CTlFactory::GetInstance();
              // Get all attached devices and exit application if no device is found.
              DeviceInfoList_t devices;
              if ( tlFactory.EnumerateDevices(devices) == 0 )
              {
                  throw RUNTIME_EXCEPTION( "No camera present.");
              }
              // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
              CInstantCameraArray cam( min( devices.size(), c_maxCamerasToUse));
              // Create and attach all Pylon Devices.
              for ( size_t i = 0; i < cam.GetSize(); ++i)
              {
                  cam[ i ].Attach( tlFactory.CreateDevice( devices[ i ]));
                  // Print the model name of the camera.
                  cout << "Using device " << cam[ i ].GetDeviceInfo().GetModelName() << endl;
              }
//               CInstantCamera camera1( CTlFactory::GetInstance().CreateFirstDevice());
//             CInstantCamera camera2( CTlFactory::GetInstance().CreateFirstDevice());
//          cout << "Using device " << camera1.GetDeviceInfo().GetModelName() << endl;
//          cout << "Using device " << camera2.GetDeviceInfo().GetModelName() << endl;

          cam[0].MaxNumBuffer = 10;
          cam[1].MaxNumBuffer = 10;

      // create pylon image format converter and pylon image
      CImageFormatConverter formatConverter1;
      formatConverter1.OutputPixelFormat= PixelType_BGR8packed;
      CPylonImage pylonImage1;
      CImageFormatConverter formatConverter2;
      formatConverter2.OutputPixelFormat= PixelType_BGR8packed;
      CPylonImage pylonImage2;

          // Start the grabbing of c_countOfImagesToGrab images.
          // The camera device is parameterized with a default configuration which
          // sets up free-running continuous acquisition.
          cam[0].StartGrabbing(GrabStrategy_LatestImageOnly);
          cam[1].StartGrabbing(GrabStrategy_LatestImageOnly);
         // cv::Mat image1,image2, gray;


    //cv::VideoCapture cam0, cam1;
	//cv::VideoWriter VideoWriter ("MyVideo.avi", CV_FOURCC('M','J','P','G'), 60, cv::Size(COLS, ROWS), true); 
	cv::Mat img0, img1;
    cv::Mat imghsv0, imghsv1; // Input HSV image : Global cause of trackbar
	cv::Mat imgbin0, imgbin1; // Input Binary image : Filtered object is ball : Global cause of trackbar
	cv::Mat intrinsicMatrix0, intrinsicMatrix1;
	cv::Mat distortionCoefficient0, distortionCoefficient1;
	cv::Mat fundamentalMatrix0, fundamentalMatrix1;

	cv::Mat imgG0, imgBG0;
	cv::Mat imgG1, imgBG1;


	cv::Mat pt0cv(1, 1, CV_64FC2);
	cv::Mat pt1cv(1, 1, CV_64FC2);
	cv::Mat pt0Ucv(1, 1, CV_64FC2);
	cv::Mat pt1Ucv(1, 1, CV_64FC2);
	cv::Mat Pt3d(4, 1, CV_64FC1);

    int filter0[6] = {255, 6, 255, 9, 126, 70}; // Arrey for HSV filter, [Hmax, Hmin, Smax, Smin, Vmax, Vmin]
    int filter1[6] = {255, 6, 255, 9, 126, 70}; // Arrey for HSV filter, [Hmax, Hmin, Smax, Smin, Vmax, Vmin]
    double whitecount0, whitecount1, cx0, cy0;


//	cam0.open(0);
//	cam1.open(1);
//	if(!cam0.isOpened() || !cam1.isOpened())
//	{
//		std::cout<<" [ERR] Camera not opened "<<std::endl;
//		std::exit(0);
//	}



//------------------------------------------------------------------------------------------------------------------------------------
	std::fstream file, file2;
	char line[100];

	file.open(INTRINSICFILENAME0, std::fstream::in);
	if(!file.is_open())
	{
		std::cout<<" [ERR] Intrinsic1 Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		intrinsicMatrix0.create(3, 3, CV_64FC1);
		for(i=0; i<intrinsicMatrix0.rows; ++i)
		{
			for(j=0; j<intrinsicMatrix0.cols; ++j)
			{
				file>>line;
				intrinsicMatrix0.at<double>(i,j) = std::atof(line);
			}
		}
		file.close();
    }
	std::cout<<intrinsicMatrix0<<std::endl<<std::endl;

	file.open(INTRINSICFILENAME1, std::fstream::in);
	if(!file.is_open())
	{
		std::cout<<" [ERR] Intrinsic1 Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		intrinsicMatrix1.create(3, 3, CV_64FC1);
		for(i=0; i<intrinsicMatrix1.rows; ++i)
		{
			for(j=0; j<intrinsicMatrix1.cols; ++j)
			{
				file>>line;
				intrinsicMatrix1.at<double>(i,j) = std::atof(line);
			}
		}
		file.close();
	}
	std::cout<<intrinsicMatrix1<<std::endl<<std::endl;

	file.open(DISTORTFILENAME0, std::fstream::in);
	if(!file.is_open())
	{
		std::cout<<" [ERR] Distortion1 Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		distortionCoefficient0.create(5, 1, CV_64FC1);
		for(i=0; i<distortionCoefficient0.rows; ++i)
		{
			for(j=0; j<distortionCoefficient0.cols; ++j)
			{
				file>>line;
				distortionCoefficient0.at<double>(i,j) = std::atof(line);
			}
			//k1[i] = std::atof(line);
		}
		file.close();
	}
	//std::cout<<distortionCoefficient0<<std::endl<<std::endl;

	file.open(DISTORTFILENAME1, std::fstream::in);
	if(!file.is_open())
	{
		std::cout<<" [ERR] Distortion1 Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		distortionCoefficient1.create(5, 1, CV_64FC1);
		for(i=0; i<distortionCoefficient1.rows; ++i)
		{
			for(j=0; j<distortionCoefficient1.cols; ++j)
			{
				file>>line;
				distortionCoefficient1.at<double>(i,j) = std::atof(line);
			}
			//k1[i] = std::atof(line);
		}
		file.close();
	}
	//std::cout<<distortionCoefficient1<<std::endl<<std::endl;


	file.open(ROTATIONFILENAME0, std::fstream::in);
	file2.open(TRANSLATIONFILENAME0, std::fstream::in);
	if(!file.is_open() || !file2.is_open())
	{
		std::cout<<" [ERR] fun1 Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		fundamentalMatrix0.create(3, 4, CV_64FC1);
		for(i=0; i<fundamentalMatrix0.rows; ++i)
		{
			for(j=0; j<fundamentalMatrix0.cols-1; ++j)
			{
				file>>line;
				fundamentalMatrix0.at<double>(i,j) = std::atof(line);
			}
			file2>>line;
			fundamentalMatrix0.at<double>(i,j) = std::atof(line);
		}
		file.close();
		file2.close();
	}
	//std::cout<<fundamentalMatrix0<<std::endl<<std::endl;

	file.open(ROTATIONFILENAME1, std::fstream::in);
	file2.open(TRANSLATIONFILENAME1, std::fstream::in);
	if(!file.is_open() || !file2.is_open())
	{
		std::cout<<" [ERR] fun1 Parameter file not found"<<std::endl;
		std::exit(0);
	}
	else
	{
		fundamentalMatrix1.create(3, 4, CV_64FC1);
		for(i=0; i<fundamentalMatrix1.rows; ++i)
		{
			for(j=0; j<fundamentalMatrix1.cols-1; ++j)
			{
				file>>line;
				fundamentalMatrix1.at<double>(i,j) = std::atof(line);
			}
			file2>>line;
			fundamentalMatrix1.at<double>(i,j) = std::atof(line);
		}
		file.close();
		file2.close();
	}
	// std::cout<<fundamentalMatrix0<<std::endl<<std::endl;
	//std::cout<<fundamentalMatrix1<<std::endl<<std::endl;

		file.open("3d.txt", std::fstream::out);
		//file2.open("time.txt", std::fstream::out);



    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3),cv::Point(-1,-1));
	char key = 'r'; // Key press to stop execution

//	for(int i=0;i<100;i++)
//	{
//		cam0 >> img0;
//		cam1 >> img1;
  //      CGrabResultPtr ptrGrabResult1;
    //    CGrabResultPtr ptrGrabResult2;

        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
      //  cam[0].RetrieveResult(50, ptrGrabResult1, TimeoutHandling_ThrowException);
        //cam[1].RetrieveResult(50, ptrGrabResult2, TimeoutHandling_ThrowException);

  //        /  const uint8_t *pImageBuffer1 = (uint8_t *) ptrGrabResult1->GetBuffer();
    //        const uint8_t *pImageBuffer2 = (uint8_t *) ptrGrabResult2->GetBuffer();

    // Convert the grabbed buffer to pylon imag
    //formatConverter1.Convert(pylonImage1, ptrGrabResult1);
    //formatConverter2.Convert(pylonImage2, ptrGrabResult2);

    // Create an OpenCV image out of pylon image
    //img0= cv::Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t *) pylonImage1.GetBuffer());
    //img1= cv::Mat(ptrGrabResult2->GetHeight(), ptrGrabResult2->GetWidth(), CV_8UC3, (uint8_t *) pylonImage2.GetBuffer());

	//	cvtColor(img0, imgBG0, CV_BGR2GRAY);
	//	cvtColor(img1, imgBG1, CV_BGR2GRAY);
	//}

	key = 'r';
	while(true)
	{
//		clock_gettime(CLOCK_REALTIME, &tS);
//		cam0 >> img0;
//		cam1 >> img1;
        CGrabResultPtr ptrGrabResult1;
        CGrabResultPtr ptrGrabResult2;

        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
        cam[0].RetrieveResult(50, ptrGrabResult1, TimeoutHandling_ThrowException);
        cam[1].RetrieveResult(50, ptrGrabResult2, TimeoutHandling_ThrowException);

            const uint8_t *pImageBuffer1 = (uint8_t *) ptrGrabResult1->GetBuffer();
            const uint8_t *pImageBuffer2 = (uint8_t *) ptrGrabResult2->GetBuffer();

    // Convert the grabbed buffer to pylon imag
    formatConverter1.Convert(pylonImage1, ptrGrabResult1);
    formatConverter2.Convert(pylonImage2, ptrGrabResult2);

    // Create an OpenCV image out of pylon image
    img0= cv::Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t *) pylonImage1.GetBuffer());
    img1= cv::Mat(ptrGrabResult2->GetHeight(), ptrGrabResult2->GetWidth(), CV_8UC3, (uint8_t *) pylonImage2.GetBuffer());
       

	//	cv::blur( img0, imghsv0, cv::Size( 1, 1 ) );
	//	cv::blur( img1, imghsv1, cv::Size( 1, 1 ) );

		cv::cvtColor(img0, imghsv0, CV_BGR2HSV); // Convert colour to HSV
		cv::cvtColor(img1, imghsv1, CV_BGR2HSV); // Convert colour to HSV

		cv::inRange(imghsv0, cv::Scalar(filter0[1], filter0[3], filter0[5]), cv::Scalar(filter0[0], filter0[2], filter0[4]), imgG0); 
		cv::inRange(imghsv1, cv::Scalar(filter1[1], filter1[3], filter1[5]), cv::Scalar(filter1[0], filter1[2], filter1[4]), imgG1); 




		cv::erode(imgG0,imgG0,kernel,cv::Point(-1,-1),2);
		cv::dilate(imgG0,imgG0,kernel,cv::Point(-1,-1),5);
		cv::erode(imgG0,imgG0,kernel,cv::Point(-1,-1),3);

		cv::erode(imgG1,imgG1,kernel,cv::Point(-1,-1),2);
    		cv::dilate(imgG1,imgG1,kernel,cv::Point(-1,-1),5);
		cv::erode(imgG1,imgG1,kernel,cv::Point(-1,-1),3);


        // Find all contours
            std::vector<std::vector<cv::Point> > contours1;
            cv::findContours(imgG0.clone(), contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
            std::vector<std::vector<cv::Point> > contours2;
            cv::findContours(imgG1.clone(), contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
            // Fill holes in each contour
                cv::drawContours(imgG0, contours1, -1, CV_RGB(255, 255, 255), -1);
                cout << contours1.size()<<endl;
                cv::drawContours(imgG1, contours2, -1, CV_RGB(255, 255, 255), -1);
                cout << contours2.size();

                if(contours1.size()!=0 && contours2.size()!=0)
                {

                cout << "first"<<endl;

                    double avg_x1(0), avg_y1(0); // average of contour points
                     double avg_x2(0), avg_y2(0); // average of contour points
                    for (int j = 0; j < contours1[0].size(); ++j)
                    {
                       avg_x1 += contours1[0][j].x;
                       avg_y1 += contours1[0][j].y;

                    }
                    cout << "first"<<endl;


                    avg_x1 /= contours1[0].size();
                    avg_y1 /= contours1[0].size();
                    pt0cv.at<cv::Vec2d>(0, 0)[0] = int(avg_x1);// = int(488);
                    pt0cv.at<cv::Vec2d>(0, 0)[1] = int(avg_y1);// = 149;
                    cout << avg_x1 << " " << avg_y1 << endl;
                    cv::circle( img0, cv::Point(avg_x1, avg_y1), 2, cv::Scalar(0, 0, 255), 8, 0 );
//                        cv::circle(imgG0, {avg_x1, avg_y1}, 5, CV_RGB(5, 100, 100), 5);
                    // Find all contours
                    cout << "firstSDGCJHAS"<<endl;

                        // Fill holes in each contour


//                     double avg_x2, avg_y2; // average of contour points
                                for (int j = 0; j < contours2[0].size(); ++j)
                                {

                                   avg_x2 += contours2[0][j].x;
                                   avg_y2 += contours2[0][j].y;

                                }
                                cout << "firstKASUDYGK"<<endl;

                                avg_x2 /= contours2[0].size();
                                avg_y2 /= contours2[0].size();
                                pt1cv.at<cv::Vec2d>(0, 0)[0] = int(avg_x2);// = int(488);
                                pt1cv.at<cv::Vec2d>(0, 0)[1] = int(avg_y2);// = 149;
                                cout << avg_x2 << " " << avg_y2 << endl;
                                cv::circle( img1, cv::Point(avg_x2, avg_y2), 2, cv::Scalar(0, 0, 255), 8, 0 );
//                                cv::circle(imgG1, {avg_x2, avg_y2}, 5, CV_RGB(5, 100, 100), 5);
                                cout << "firstQLWEFGQLIFLIQUU"<<endl;
}

//                    float radius ;
//                    Point2f center ;
//                    minEnclosingCircle ( contours[i] , center , radius ) ;



//                    cv::circle(imgG1, center, 5, CV_RGB(5, 100, 100), 5);
	// 	// cvtColor(img0, imgG0, CV_BGR2GRAY);
	// 	// absdiff(imgG0, imgBG0, imgG0);

	// 	// cvtColor(img1, imgG1, CV_BGR2GRAY);
	// 	// absdiff(imgG1, imgBG1, imgG1);
		cx0 = 0;
		cy0 = 0;
		whitecount0 = 0;
        for(i=0; i<ROWS; i++)
        {
            for(j=0; j<COLS; j++)
            {
                if(imgG0.data[i*COLS + j] > 180) // If colour is white
                {
                    whitecount0++; // Number of white pixels
//					cx0 += j; //
//					cy0 += i; //
                }
            }
        }

//		if(whitecount0>1)
//		{
//			cx0 /= whitecount0;
//			cy0 /= whitecount0;
//			pt0cv.at<cv::Vec2d>(0, 0)[0] = int(cx0);// = int(488);
//			pt0cv.at<cv::Vec2d>(0, 0)[1] = int(cy0);// = 149;
//		}

//		cv::circle( img0, cv::Point(cx0, cy0), 2, cv::Scalar(0, 0, 255), 8, 0 );
        cx0 = 0;
        cy0 = 0;
        whitecount1 = 0;
        for(i=0; i<ROWS; i++)
        {
            for(j=0; j<COLS; j++)
            {
                if(imgG1.data[i*COLS + j] > 200) // If colour is white
                {
                    whitecount1++; // Number of white pixels
//					cx0 += j; //
//					cy0 += i; //
                }
            }
        }
//		if(whitecount1>1)
//		{
//			cx0 /= whitecount1;
//			cy0 /= whitecount1;
//	 		pt1cv.at<cv::Vec2d>(0, 0)[0] = int(cx0);// = 513;
//	 		pt1cv.at<cv::Vec2d>(0, 0)[1] = int(cy0);// = 147;
//		}

        if(whitecount0>1 && whitecount1>1)
//        if(whitecount0==0)
		{
//            cv::circle( img0, cv::Point(avg_x1, avg_y1), 2, cv::Scalar(0, 0, 255), 8, 0 );
//            cv::circle( img1, cv::Point(avg_x2, avg_y2), 2, cv::Scalar(0, 0, 255), 8, 0 );


			cv::undistortPoints(pt0cv, pt0Ucv, intrinsicMatrix0, distortionCoefficient0);
			cv::undistortPoints(pt1cv, pt1Ucv, intrinsicMatrix1, distortionCoefficient1);
			//std::cout<<" "<<pt0Ucv.at<cv::Vec2f>(0,0)[0]<<" "<<pt0Ucv.at<cv::Vec2f>(0,0)[1]<<std::endl;
			//std::cout<<" "<<pt1Ucv.at<cv::Vec2f>(0,0)[0]<<" "<<pt1Ucv.at<cv::Vec2f>(0,0)[1]<<std::endl;
			cv::triangulatePoints(fundamentalMatrix0, fundamentalMatrix1, pt0Ucv, pt1Ucv, Pt3d);
			std::cout<<Pt3d.at<float>(0,0)/Pt3d.at<float>(3,0)<<" "<<Pt3d.at<float>(1,0)/Pt3d.at<float>(3,0)<<" "<<Pt3d.at<float>(2,0)/Pt3d.at<float>(3,0)<<" "<<std::endl;
			file<<Pt3d.at<double>(0,0)/Pt3d.at<double>(3,0)<<" "<<Pt3d.at<double>(1,0)/Pt3d.at<double>(3,0)<<" "<<Pt3d.at<double>(2,0)/Pt3d.at<double>(3,0)<<"\n";
			clock_gettime(CLOCK_REALTIME, &tE);
			//std::cout << "Time taken is: " <<tE.tv_nsec -  tS.tv_nsec  << std::endl;
			file2<<tE.tv_nsec -  tS.tv_nsec <<"\n";
			//cv::imshow("Bn0",imgG0); // Show filtered image
		}
	 	//VideoWriter.write(img0);
        cv::imshow("Tr0", img0); // Show centroid image
		cv::imshow("Tr1", img1); // Show centroid image
		cv::imshow("Bn1",imgG1); // Show filtered image
        cv::imshow("Bn0",imgG0); // Show filtered image

		key = cv::waitKey(1); // Wait for 1ms for intrupt from user to exit
		if(key == 'q') // If user presses 'q'
			break; // Exit while loop
	}


		file.close();
		file2.close();

    	cv::destroyAllWindows(); // Distroid all display windows
//    	cam0.release();
//    	cam1.release();
        img0.release();
    	img1.release();
        imghsv0.release();
    	imghsv1.release();
        imgG0.release();
    	imgG1.release();

    	return 0;
          }

        catch (GenICam::GenericException &e)
        {
            // Error handling.
            cerr << "An exception occurred." << endl
            << e.GetDescription() << endl;
    //            exitCode = 1;
        }

    //        return exitCode;
        PylonTerminate();

}
