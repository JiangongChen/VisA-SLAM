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
public:
    Server* server_;
    // client id starts from 0 
    int id_;
    int connfd_;
    std::thread client_thread_; 
    bool recvFlag; 
    ORB_SLAM2::ORBextractor* extractor_;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    // not used for monocular slam
    float mbf;
    float mThDepth; 
}; 

#endif