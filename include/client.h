#ifndef CLIENT_H
#define CLIENT_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <iostream>
#include "slampkt.h"
#include "cmdpkt.h"
#include "Frame.h"
#include "server.h"
#include "ORBextractor.h"
#include "ORBVocabulary.h"
using namespace std; 

class Server; 
class Client{
public:
    Client(int id, int connf, const string settingFile, Server* server);
    void Close(); 
    void receiveLoop();
    void trackLoop(); 
    void InsertFrame(ORB_SLAM2::Frame *pF);
    bool CheckNewFrame();
    // send a number to the client
    bool sendMsg(int num);
    ORB_SLAM2::Frame* GetNewFrame();
public:
    Server* server_;
    // client id starts from 0 
    int id_;
    int connfd_;
    int nFeaturesInit;
    int nFeatures; 
    std::thread client_thread_; 
    std::thread tracking_thread_; 
    bool recvFlag; 
    bool initFlag; 
    ORB_SLAM2::ORBextractor* extractor_;

    std::mutex mMutexClient;

    std::list<ORB_SLAM2::Frame*> mlNewFrames;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    // not used for monocular slam
    float mbf;
    float mThDepth; 

    // Vector for tracking time statistics
    vector<double> vTimesTrack;
    vector<cv::Mat> trajectory; 
    vector<double> vTimestamps; 
    vector<int> trajectory_gt_points; 
}; 

#endif