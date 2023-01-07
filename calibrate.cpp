//
//  calibrate.cpp
//  Camera Calibration and AugmentedReality
//  This file contains all the functions used to calibrate the camera,
//project 3-D objects on the target (AR), overlaying an image on AruCO markers and feature matching using ORB.
//  Created by Shivani Naik and Pulkit Saharan on 04/11/22.
//


#include "calibrate.hpp"
#include <filesystem>
#include <string>
#include <iostream>
#include <regex>
#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/features2d.hpp>
using namespace cv;
using namespace std;

// Function to detect chessboard, extract chessboard corners and draw them
void extract_draw_corners(cv::Mat& src, std::vector<std::vector<cv::Vec3f> >& point_list, std::vector<std::vector<cv::Point2f> >& corner_list ) {

    cv::Mat gray;
    std::vector<cv::Vec3f> point_set;
    std::vector<cv::Point2f> corner_set;
    cvtColor(src, gray, cv::COLOR_RGB2GRAY);

    Size patternsize(9, 6);
    // find chessboard
    bool patternfound = findChessboardCorners(gray, patternsize, corner_set,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);

    if (patternfound)
        cornerSubPix(gray, corner_set, Size(11, 11), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    
    drawChessboardCorners(src, patternsize, Mat(corner_set), patternfound);
    // add corner set to point and corner list
    add_to_image_set(corner_set, point_list, corner_list);
 }

// Function to add chessboard coreners to corner and point list
void add_to_image_set(std::vector<cv::Point2f> corner_set, std::vector<std::vector<cv::Vec3f> >& point_list, std::vector<std::vector<cv::Point2f> >& corner_list) {


    //Defining point_set, point_list and corner_list
    std::vector<cv::Vec3f> point_set; //world co-ordinates 3D
    
    Size patternsize(9, 6);

    // add real world coordinates to point set
    for (int i = 0; i < patternsize.height; i++) {
        for (int j = 0; j < patternsize.width; j++) {
            point_set.push_back(cv::Point3f(j, -i, 0));
        }
    }

    // add point set and corner set to point and corner list
    point_list.push_back(point_set);
    corner_list.push_back(corner_set);
}

// Function creates a calibration image set from all images saved in path
void create_image_set(string path, std::vector<std::vector<cv::Vec3f> >& point_list, std::vector<std::vector<cv::Point2f> >& corner_list)
{
    for (const auto & entry : filesystem::directory_iterator(path))
    {
        string filename =entry.path().c_str();
        if(filename.substr(filename.length() - 3) == "png")
        {
            cout << "Processing "<<filename << endl;
            cv::Mat img = imread(filename);
            // extract and add to point and corner list
            extract_draw_corners(img, point_list, corner_list);
        }
    }
}

//Function to calibrate the camera using different images and get the intrinsic paramters
// Gives Camera matrix, distortion coefficient and calibration error
float calibrate_camera(std::vector<std::vector<cv::Vec3f>> &point_list,
                       std::vector<std::vector<cv::Point2f>> &corner_list,
                       cv::Mat &camera_matrix, cv::Mat &distortion_coeff)
{
    Size frame_size(1280,720);

    std::vector<cv::Mat> R, T;
    // Calibrate and get error
    float error = cv::calibrateCamera(point_list,
                                corner_list,
                                frame_size,
                                camera_matrix,
                                distortion_coeff,
                                R,
                                T,
                                cv::CALIB_FIX_ASPECT_RATIO,
                                cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 30, DBL_EPSILON));
    //Print outs the result from calibration

    std::cout << "cameraMatrix : " << camera_matrix << std::endl;
    std::cout << "distortion Coeffs : " << distortion_coeff << std::endl;
    std::cout << "Rotation vector : " << R[0] << std::endl;
    std::cout << "Translation vector : " << T[0] << std::endl;
    cout << "Error:" << error << std::endl;
    return(error);
}

//Function to determine current camera position. Returns R and T matrix
bool camera_position(cv::Mat& src,cv::Mat &camera_matrix, cv::Mat &distortion_coeff,
    cv::Mat& rotational_vec, cv::Mat& trans_vec, string objType, vector<cv::Point3f> vertices, vector<std::vector<int>> face_vertices)
{

    cv::Mat gray;
    std::vector<cv::Vec3f> point_set;
    std::vector<cv::Point2f> corner_set;
    cvtColor(src, gray, cv::COLOR_RGB2GRAY);


   Size patternsize(9, 6);
    //extract corner of frame
    bool patternfound = findChessboardCorners(gray, patternsize, corner_set,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
        + CALIB_CB_FAST_CHECK);


   if (patternfound)
        cornerSubPix(gray, corner_set, Size(11, 11), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


   //extract world co-ordinates
    for (int i = 0; i < patternsize.height; i++) {
        for (int j = 0; j < patternsize.width; j++) {
            point_set.push_back(cv::Point3f(j, -i, 0));
        }
    }

    // If camera detects any pattern then draws different objects

    if (patternfound)
    {
        Scalar green(0,255,0);
        cv::solvePnP(point_set,corner_set, camera_matrix,distortion_coeff, rotational_vec,trans_vec);
        //Projects 3D axes
        if(objType == "Axes")
            draw_axes(src, camera_matrix, distortion_coeff, rotational_vec, trans_vec);
        //Projects a chair
        else if (objType == "Chair")
            draw_chair(src, camera_matrix, distortion_coeff, rotational_vec , trans_vec);
        //Projects object
        else if (objType == "Obj")
            draw_3d_obj_object(src, camera_matrix, distortion_coeff, rotational_vec , trans_vec, vertices, face_vertices);
        //Projects a cube
        else if (objType == "Cube")
            draw_cube(src, camera_matrix, distortion_coeff, rotational_vec, trans_vec);
        //Projects a table
        else if (objType == "Table")
            draw_table(src, camera_matrix, distortion_coeff, rotational_vec, trans_vec);

    }

    return patternfound;
}

//Function to draw 3D axes
void draw_axes(cv::Mat& src,cv::Mat &camera_matrix, cv::Mat &distortion_coeff,
              cv::Mat& R, cv::Mat& T)
{
    vector<cv::Vec3f> real_points;
    std::vector<cv::Point2f> image_points;

    // Axes coordinates in real world
    real_points.push_back(cv::Vec3f({0, 0, 0}));
    real_points.push_back(cv::Vec3f({5, 0, 0}));
    real_points.push_back(cv::Vec3f({0, -5, 0}));
    real_points.push_back(cv::Vec3f({0, 0, 5}));
    
    // get axes projections in image coordinates
    cv::projectPoints(real_points, R, T, camera_matrix, distortion_coeff, image_points);
    
    // draw axes on src image
    cv::arrowedLine(src, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 3);
    cv::arrowedLine(src, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 3);
    cv::arrowedLine(src, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 3);
}

//Function to draw chair

void draw_chair(cv::Mat& src,cv::Mat &camera_matrix, cv::Mat &distortion_coeff,
                cv::Mat& R, cv::Mat& T)
{
    vector<cv::Vec3f> real_points;
    vector<cv::Point2f> image_points;
    Scalar color(255,255,0);
    int thickness = 2;
    
    //Chair leg squares
    real_points.push_back(cv::Vec3f({3, -2, 0}));
    real_points.push_back(cv::Vec3f({3.25, -2, 0}));
    real_points.push_back(cv::Vec3f({3.25, -2.25, 0}));
    real_points.push_back(cv::Vec3f({3, -2.25, 0}));
    
    // chair leg 1
    real_points.push_back(cv::Vec3f({3, -2, 3}));
    real_points.push_back(cv::Vec3f({3.25, -2, 3}));
    real_points.push_back(cv::Vec3f({3.25, -2.25, 3}));
    real_points.push_back(cv::Vec3f({3, -2.25, 3}));
    
    //Chair leg squares
    real_points.push_back(cv::Vec3f({3, -5, 0}));
    real_points.push_back(cv::Vec3f({3.25, -5, 0}));
    real_points.push_back(cv::Vec3f({3.25, -5.25, 0}));
    real_points.push_back(cv::Vec3f({3, -5.25, 0}));
    
    // chair leg 2
    real_points.push_back(cv::Vec3f({3, -5, 3}));
    real_points.push_back(cv::Vec3f({3.25, -5, 3}));
    real_points.push_back(cv::Vec3f({3.25, -5.25, 3}));
    real_points.push_back(cv::Vec3f({3, -5.25, 3}));
    
    //Chair leg squares
    real_points.push_back(cv::Vec3f({6, -2, 0}));
    real_points.push_back(cv::Vec3f({6.25, -2, 0}));
    real_points.push_back(cv::Vec3f({6.25, -2.25, 0}));
    real_points.push_back(cv::Vec3f({6, -2.25, 0}));
    
    // chair leg 3
    real_points.push_back(cv::Vec3f({6, -2, 3}));
    real_points.push_back(cv::Vec3f({6.25, -2, 3}));
    real_points.push_back(cv::Vec3f({6.25, -2.25, 3}));
    real_points.push_back(cv::Vec3f({6, -2.25, 3}));
    //Chair leg squares
    real_points.push_back(cv::Vec3f({6, -5, 0}));
    real_points.push_back(cv::Vec3f({6.25, -5, 0}));
    real_points.push_back(cv::Vec3f({6.25, -5.25, 0}));
    real_points.push_back(cv::Vec3f({6, -5.25, 0}));
    
    // chair leg 4
    real_points.push_back(cv::Vec3f({6, -5, 3}));
    real_points.push_back(cv::Vec3f({6.25, -5, 3}));
    real_points.push_back(cv::Vec3f({6.25, -5.25, 3}));
    real_points.push_back(cv::Vec3f({6, -5.25, 3}));
    
    //Chair base 1
    real_points.push_back(cv::Vec3f({3, -5.25, 3}));
    real_points.push_back(cv::Vec3f({6.25, -5.25, 3}));
    real_points.push_back(cv::Vec3f({6.25, -2, 3}));
    real_points.push_back(cv::Vec3f({3, -2, 3}));
    
    //Chair base 2
    real_points.push_back(cv::Vec3f({3, -5.25, 3.25}));
    real_points.push_back(cv::Vec3f({6.25, -5.25, 3.25}));
    real_points.push_back(cv::Vec3f({6.25, -2, 3.25}));
    real_points.push_back(cv::Vec3f({3, -2, 3.25}));
    
    // chair back 1
    real_points.push_back(cv::Vec3f({6.25, -2, 3.25}));
    real_points.push_back(cv::Vec3f({6.25, -2, 6}));
    real_points.push_back(cv::Vec3f({3, -2, 6}));
    real_points.push_back(cv::Vec3f({3, -2, 3.25}));
    
    // chair back 2
    real_points.push_back(cv::Vec3f({6.25, -2.25, 3.25}));
    real_points.push_back(cv::Vec3f({6.25, -2.25, 6}));
    real_points.push_back(cv::Vec3f({3, -2.25, 6}));
    real_points.push_back(cv::Vec3f({3, -2.25, 3.25}));
    
    // chair back middle line
    real_points.push_back(cv::Vec3f({4.525, -2, 3.25}));
    real_points.push_back(cv::Vec3f({4.525, -2, 6}));
    real_points.push_back(cv::Vec3f({4.725, -2, 6}));
    real_points.push_back(cv::Vec3f({4.725, -2, 3.25}));

    
    real_points.push_back(cv::Vec3f({4.525, -2.25, 3.25}));
    real_points.push_back(cv::Vec3f({4.525, -2.25, 6}));
    real_points.push_back(cv::Vec3f({4.725, -2.25, 6}));
    real_points.push_back(cv::Vec3f({4.725, -2.25, 3.25}));
    
   
    
    cv::projectPoints(real_points, R, T, camera_matrix, distortion_coeff, image_points);

    for(int i = 0; i<4;i++)
    {
        // chair leg square
        cv::line(src, image_points[8*i+0], image_points[8*i+1], color, thickness);
        cv::line(src, image_points[8*i+1], image_points[8*i+2], color, thickness);
        cv::line(src, image_points[8*i+2], image_points[8*i+3], color, thickness);
        cv::line(src, image_points[8*i+3], image_points[8*i+0], color, thickness);
        
        // chair leg
        cv::line(src, image_points[8*i+0], image_points[8*i+4], color, thickness);
        cv::line(src, image_points[8*i+1], image_points[8*i+5], color, thickness);
        cv::line(src, image_points[8*i+2], image_points[8*i+6], color, thickness);
        cv::line(src, image_points[8*i+3], image_points[8*i+7], color, thickness);
    }
    // chair base
    for(int i = 0;i<2;i++)
    {
        cv::line(src, image_points[32+0 +4*i], image_points[32+1+4*i], color, thickness);
        cv::line(src, image_points[32+1+4*i], image_points[32+2+4*i], color, thickness);
        cv::line(src, image_points[32+2+4*i], image_points[32+3+4*i], color, thickness);
        cv::line(src, image_points[32+3+4*i], image_points[32+0+4*i], color, thickness);
    }
    
    // chair base lines
    for(int i = 0; i<4;i++)
        cv::line(src, image_points[32+i], image_points[32+i+4], color, thickness);
    
    //chair back lines
    for(int i = 0;i<2;i++)
    {
        cv::line(src, image_points[40+0+4*i], image_points[40+1+4*i], color, thickness);
        cv::line(src, image_points[40+1+4*i], image_points[40+2+4*i], color, thickness);
        cv::line(src, image_points[40+2+4*i], image_points[40+3+4*i], color, thickness);
        cv::line(src, image_points[40+3+4*i], image_points[40+0+4*i], color, thickness);

        cv::line(src, image_points[40+1+i], image_points[40+5+i], color, thickness);

    }
    
    // chair back middle lines
    for(int i = 0;i<2;i++)
    {
        cv::line(src, image_points[48+0+4*i], image_points[48+1+4*i], color, thickness);
        cv::line(src, image_points[48+2+4*i], image_points[48+3+4*i], color, thickness);

    }
}

// Draw a 3d object from obj files, using parsed vertices and faces
void draw_3d_obj_object(cv::Mat &src,cv::Mat &camera_matrix, cv::Mat &distortion_coeff,
                        cv::Mat& R, cv::Mat& T, vector<cv::Point3f> vertices, vector<std::vector<int>> face_vertices)
{
    vector<cv::Point2f> image_vertices;
    Scalar color(255,0,255);
    int thickness = 1;
    
    // get image points of vertices by projecting using R, T
    cv::projectPoints(vertices, R, T, camera_matrix, distortion_coeff, image_vertices);
    
    // draw lines of every face parsed from obj file
    for(int i = 0; i<face_vertices.size(); i++)
    {
        for(int j = 0; j<face_vertices[i].size() - 1; j++)
        {
            // draw line between consecutive vertices of each face, face_vertices gives vertex index into vertex vector
            cv::line(src,image_vertices[face_vertices[i][j]-1],image_vertices[face_vertices[i][j+1]-1], color, thickness);
        }
    }

}

//function to draw a cube on the target
void draw_cube(cv::Mat& src, cv::Mat& camera_matrix, cv::Mat& distortion_coeff,
    cv::Mat& R, cv::Mat& T)
{
    vector<cv::Vec3f> real_points;
    vector<cv::Point2f> image_points;
    //Scalar color(255, 0, 0);
    // cube points
    real_points.push_back(cv::Vec3f({ 5, -4, 0 }));
    real_points.push_back(cv::Vec3f({ 5, -4, 2 }));
    real_points.push_back(cv::Vec3f({ 3, -4, 0 }));
    real_points.push_back(cv::Vec3f({ 3, -4, 2 }));
    real_points.push_back(cv::Vec3f({ 5, -2, 0 }));
    real_points.push_back(cv::Vec3f({ 5, -2, 2 }));
    real_points.push_back(cv::Vec3f({ 3, -2, 0 }));
    real_points.push_back(cv::Vec3f({ 3, -2, 2 }));


    cv::projectPoints(real_points, R, T, camera_matrix, distortion_coeff, image_points);
    //Lines to connect the points
    cv::line(src, image_points[0], image_points[2], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[4], image_points[6], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[0], image_points[4], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[3], image_points[7], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[2], image_points[3], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[4], image_points[5], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[6], image_points[7], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[0], image_points[1], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[2], image_points[6], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[1], image_points[3], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[5], image_points[7], cv::Scalar(0, 0, 255),2);
    cv::line(src, image_points[1], image_points[5], cv::Scalar(0, 0, 255),2);
    

}

//Function to draw a table
void draw_table(cv::Mat& src, cv::Mat& camera_matrix, cv::Mat& distortion_coeff, cv::Mat& R, cv::Mat& T) {

    vector<cv::Vec3f> real_points;
    vector<cv::Point2f> image_points;

    //left side leg
    real_points.push_back(cv::Vec3f({ 0, -4, 0 }));//0
    real_points.push_back(cv::Vec3f({ 0, -4, 3 }));//1
    real_points.push_back(cv::Vec3f({ 0.5, -4, 0 }));//2
    real_points.push_back(cv::Vec3f({ 0.5, -4, 3 }));//3
    real_points.push_back(cv::Vec3f({ 0, -2, 0 }));//4
    real_points.push_back(cv::Vec3f({ 0.5, -2, 0 }));//5
    real_points.push_back(cv::Vec3f({ 0, -2, 3 }));//6
    real_points.push_back(cv::Vec3f({ 0.5, -2, 3 }));//7
    //right side leg
    real_points.push_back(cv::Vec3f({ 5.5, -4, 0 }));//8
    real_points.push_back(cv::Vec3f({ 5.5, -4, 3 }));//9
    real_points.push_back(cv::Vec3f({ 6, -4, 0 }));//10
    real_points.push_back(cv::Vec3f({ 6, -4, 3 }));//11
    real_points.push_back(cv::Vec3f({5.5, -2, 0 }));//12
    real_points.push_back(cv::Vec3f({ 6, -2, 0 }));//13
    real_points.push_back(cv::Vec3f({ 5.5, -2, 3 }));//14
    real_points.push_back(cv::Vec3f({ 6, -2, 3 }));//15
    //Table top
    real_points.push_back(cv::Vec3f({ 3, -3, 3 }));//16


    cv::projectPoints(real_points, R, T, camera_matrix, distortion_coeff, image_points);
    //Drawing lines of table
    cv::line(src, image_points[0], image_points[2], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[0], image_points[1], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[0], image_points[4], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[2], image_points[3], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[2], image_points[5], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[4], image_points[5], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[4], image_points[6], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[5], image_points[7], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[1], image_points[6], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[1], image_points[3], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[3], image_points[7], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[6], image_points[7], cv::Scalar(0, 255, 255),3);

    cv::line(src, image_points[8],  image_points[10], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[8],  image_points[9],  cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[8],  image_points[12], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[10], image_points[11], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[10], image_points[13], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[12], image_points[13], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[12], image_points[14], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[13], image_points[15], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[9],  image_points[14], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[9],  image_points[11], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[11], image_points[15], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[14], image_points[15], cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[3],  image_points[9],  cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[14], image_points[7],  cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[16], image_points[3],  cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[16], image_points[9],  cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[16], image_points[7],  cv::Scalar(0, 255, 255),3);
    cv::line(src, image_points[16], image_points[14], cv::Scalar(0, 255, 255),3);

}

//Function to detect harris corner
void harris_corner(cv::Mat& src) {
    cv::Mat  src_gray;
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    //threshold value
    int thresh = 200;
    int max_thresh = 255;

    //converting image to grayscale
    cvtColor(src, src_gray, COLOR_BGR2GRAY);

    Mat dst = Mat::zeros(src_gray.size(), CV_32FC1);
    cornerHarris(src_gray, dst, blockSize, apertureSize, k);

    Mat dst_norm, dst_norm_scaled;
    Mat output = src.clone();
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);
    //displaying points on the image/video frame
    for (int i = 0; i < dst_norm.rows; i++)
    {
        for (int j = 0; j < dst_norm.cols; j++)
        {
            if ((int)dst_norm.at<float>(i, j) > thresh)
            {
                circle(output, Point(j, i), 2, Scalar(255, 0, 0), 2, 8, 0);
            }
        }
    }
    imshow("Harris_Corner", output);
}

//Aruco marker detection
bool aruco_marker_detection(cv::Mat& src, cv::Mat& output) {
  
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(src, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

   output= src.clone();
   cv::aruco::drawDetectedMarkers(output, markerCorners, markerIds);
   
   return markerCorners.size() == 4;
}

//Function to overlay an image on the ArUco markers
void aruco_out(cv::Mat& target,cv::Mat& actual, cv::Mat& output) {
    //declaration
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(target, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

   //Creating vector of size 4 to store corner points of destination image
    vector<Point2f> pts_dst(4);
    //initialize the vector with size 4
    pts_dst.resize(4);
   
  for (int i = 0;i < markerCorners.size(); i++) {
      if (markerIds[i] == 40) {
          pts_dst[0] = markerCorners[i][0];
      }
      else if (markerIds[i] == 124) {
          pts_dst[1] = markerCorners[i][0];
      }
      if (markerIds[i] == 98) {
          pts_dst[2] = markerCorners[i][1];
      }
      if (markerIds[i] == 203) {
          pts_dst[3] = markerCorners[i][3];
      }
  }
  //Creating vector of size 4 to store corner points of source image
     vector<Point2f> pts_src;
     pts_src.push_back(Point2f(0, 0));
     pts_src.push_back(Point2f(actual.size().width, 0));
     pts_src.push_back(Point2f(0, actual.size().height));
     pts_src.push_back(Point2f(actual.size().width, actual.size().height));

    //calculate homography
    cv::Mat h_matrix = cv::findHomography(pts_src, pts_dst);
   
   // Warped image
    Mat warpedImage;
    // Warp source image to destination based on homography
    warpPerspective(actual, warpedImage, h_matrix, target.size());

    output = target.clone();

   for (int i = 0; i < warpedImage.rows; i++) {
       for (int j = 0; j < warpedImage.cols; j++) {
           if (warpedImage.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0)) {
               output.at<cv::Vec3b>(i, j) = warpedImage.at<cv::Vec3b>(i, j);
           }
       }
   }

}

//Function to get the matching features of two images using ORB algorithm
void ORB_matching(cv::Mat& img_1, cv::Mat& img_2) {

    // declaring keypoint vector for both the images
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    //Creating ORB feature detector and descriptor extractor
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    //create brute force method of matching which using Hamming distance metrics
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    
    //Creating keypoints for both the images using ORB Detector
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
    //Computing descriptors for both the images using ORB descriptor
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    Mat outimg1;
    //Draws keypoints in image 1
     drawKeypoints(img_1, keypoints_1, outimg1, cv::Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
    imshow("outimg1", outimg1);
    //imwrite("outimg1.jpg", outimg1);

    Mat outimg2;
    //Draws keypoints in image 2
    drawKeypoints(img_2, keypoints_2, outimg2, cv::Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
    imshow("outimg2", outimg2);
    //imwrite("outimg2.jpg", outimg2);

    vector<DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);

    double min_dist = 100, max_dist = 0;
    //Creating matches between two images using brute force distance
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("Max dist : %f \n", max_dist);
    printf("Min dist : %f \n", min_dist);

    //Keeping good matches between two images using brute force distance
    std::vector< DMatch > good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match, cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0));
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch, cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0));
    imshow("img_match", img_match);
    imshow("good_match", img_goodmatch);

}
