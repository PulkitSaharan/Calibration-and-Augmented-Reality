//
//  main.cpp
//  Project 4
//  Calibration and AugmentedReality
// This function shows the working of the system on press of different keys. This file enable the user to calibrate the camera and project 3D objects
//

#include <iostream>
#include <string>
#include <regex>
using namespace std;

#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/features2d.hpp>
#include "calibrate.hpp"
#include "OBJParser.hpp"

using namespace cv;

// Main function detects targets and puts the required objects in the frame on different key presses
// Command line argument: path to 3d obj file
int main(int argc, char* argv[]) {
    // Declarations
    cv::VideoCapture *capdev;
    cv::Mat frame, distortion_coeff, img, rotational_vec, trans_vec;
    cv::Mat output,aruco_output;

    bool patternfound;
    string calibration_image_path = "/Users/Documents/MSDAE/Computer Vision/Projects/Project 4/AugmentedReality/AugmentedReality/calibration_images";
    cv::Mat actual = cv::imread("/Users/Documents/MSDAE/Computer Vision/Projects/Project 4/AugmentedReality/AugmentedReality/aruco.png");
    cv::Mat img_1 = cv::imread("/Users/Documents/MSDAE/Computer Vision/Projects/Project 4/AugmentedReality/AugmentedReality/eiffel1.png");
    cv::Mat img_2 = cv::imread("/Users/Documents/MSDAE/Computer Vision/Projects/Project 4/AugmentedReality/AugmentedReality/eiffel2.png");
   
    // command line argument for obj file
    string obj_path(argv[1]);
    
    FileStorage fs_write;
    FileStorage fs_read;
    std::vector<cv::Point2f> corner_set;
    std::vector<cv::Vec3f> point_set; //world co-ordinates 3
    std::vector<std::vector<cv::Vec3f> > point_list; // list of world co-ordinates
    std::vector<std::vector<cv::Point2f> > corner_list; //list of image points
    
    std::vector<std::vector<cv::Vec3f> > temp_point_list; // list of world co-ordinates
    std::vector<std::vector<cv::Point2f> > temp_corner_list;
    char pressed_key = 'o';
    int min_calibrate_images = 5;
    string save_path;
 
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<std::vector<int>> face_vertices;
    std::vector<int> face_normals;
    parse_file(obj_path,vertices, normals, face_vertices, face_normals);

    capdev = new cv::VideoCapture(0);
    capdev->set(cv::CAP_PROP_FRAME_WIDTH, 1280);//Setting the width of the video 1280
    capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 720);//Setting the height of the video// 720
    if( !capdev->isOpened() ) {
            printf("Unable to open video device\n");
            return(-1);
    }
    cv::namedWindow("Video", 1); // identifies a window

    // get some properties of the image
    cv::Size refS( (int) capdev->get(cv::CAP_PROP_FRAME_WIDTH ),
                   (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);
    float cols = refS.width/2;
    float rows = refS.height/2;
    
    double mat_init[3][3] = {{1,0,cols},{0,1, rows},{0,0,1}};
    
    // Create camera matrix and initialize
    cv::Mat camera_matrix = cv::Mat(3,3, CV_64FC1, &mat_init);
    cout << "Initialized camera matrix" << endl;
    // file to save camera properties
    fs_read = FileStorage("/Users/shivaninaik/Documents/MSDAE/Computer Vision/Projects/Project 4/AugmentedReality/AugmentedReality/intrinsic.yml", FileStorage::READ);
    fs_read["camera_matrix"] >>  camera_matrix;
    fs_read["distortion_coeff"] >>  distortion_coeff;
    fs_read.release();
    cout << "Read camera matrix" << endl;

    for (;;) {
        *capdev >> frame; // get a new frame from the camera, treat as a stream
        if( frame.empty() ) {
          printf("frame is empty\n");
          break;
        }
    
        // see if there is a waiting keystroke
        char key = cv::waitKey(10);
        if(key != -1)
            pressed_key = key;
        bool aruco_marker_found = false;

        switch(pressed_key)
        {
            
            // show extracted corners
            case 'e':
                img = imread(calibration_image_path+"/3.png");
                extract_draw_corners(img, temp_point_list, temp_corner_list);
                imshow("corner_image", img);
                cv::imshow("Video", frame);
                break;
            // calibrate camera and save current image for calibration
            case 's':
                save_path = calibration_image_path+ "/10.png";
                imwrite(save_path, frame);
                
                point_list.clear();
                corner_list.clear();
                create_image_set(calibration_image_path, point_list, corner_list);
                if(point_list.size() > min_calibrate_images)
                {
                    
                    calibrate_camera(point_list,corner_list,camera_matrix, distortion_coeff);
                    
                    fs_write = FileStorage("/Users/shivaninaik/Documents/MSDAE/Computer Vision/Projects/Project 4/AugmentedReality/AugmentedReality/intrinsic.yml", FileStorage::WRITE);
                    fs_write << "camera_matrix" << camera_matrix;
                    fs_write << "distortion_coeff" << distortion_coeff;
                    fs_write.release();
                }
                else
                    cout<<"Too few images for calibration, please capture more and then calibrate";
                pressed_key = 'o';
                cv::imshow("Video", frame);
                break;
            // detect and draw axes
            case 'd':
                
                patternfound = camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Axes");
                if(patternfound)
                {
                    // print the vectors in real time when detects the grid
                    std::cout << "Rotation Matrix: " << std::endl;
                    for (int i = 0; i < rotational_vec.rows; i++)
                    {
                        for (int j = 0; j < rotational_vec.cols; j++)
                        {
                            std::cout << rotational_vec.at<cv::Vec2f>(i, j) << std::endl;
                        }
                    }
                    
                    std::cout << "Translation Matrix: " << std::endl;
                    for (int i = 0; i < trans_vec.rows; i++)
                    {
                        for (int j = 0; j < trans_vec.cols; j++)
                        {
                            std::cout << trans_vec.at<cv::Vec2f>(i, j) << std::endl;
                        }
                    }
                }
                cv::imshow("Video", frame);
                break;
            //Press 'h' to project chair on target
            case 'h':
                camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Chair");
                cv::imshow("Video", frame);
                break;
            //Press '3' to project Plane on target
            case '3':
                camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Obj", vertices, face_vertices);
                cv::imshow("Video", frame);
                break;
            //Press 'b' to project Cube on target
            case 'b':
                camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Cube");
                cv::imshow("Video", frame);
                break;
            //Press 't' to project table on target
            case 't':
                camera_position(frame, camera_matrix, distortion_coeff, rotational_vec, trans_vec, "Table");
                cv::imshow("Video", frame);
                break;
            //Press 'a' to detect aruco markers
            case 'a':
                aruco_marker_detection(frame, output);
                imshow("corner_image", output);
                cv::imshow("Video", frame);
                break;
            //Press 'p' to overlay an image on the aruco markers
            case 'p':
                aruco_marker_found = aruco_marker_detection(frame, output);
                if(aruco_marker_found)
                {
                    aruco_out(frame, actual, aruco_output);
                    imshow("aruco_overlay", aruco_output);
                }
                cv::imshow("Video", frame);
                break;
            //Press 'r' to see the matching between two images usign ORB
            case 'r':
                ORB_matching(img_1, img_2);
                cv::imshow("Video", frame);

                break;
            //Press '7' to detect Harris corner
            case '7':
                harris_corner(frame);
                cv::imshow("Video", frame);

                break;
            default:
                cv::imshow("Video", frame);
                break;
        }
        // quit if key pressed is 'q'
        if( key == 'q')
            break;
    }

    return(0);
}
