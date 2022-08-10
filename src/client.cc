# include "client.h"

Client::Client(int id, int connfd, const string settingFile, Server* server):
id_(id), connfd_(connfd), recvFlag(true) {
    server_ = server; 
    cv::FileStorage fSettings(settingFile, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];
    mThDepth = mbf*(float)fSettings["ThDepth"]/fx;

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;


    float rows = fSettings["Camera.rows"];
    float cols = fSettings["Camera.cols"];

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    extractor_ = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    client_thread_ = std::thread(&Client::receiveLoop,this); 
}

void Client::Close(){
    recvFlag = false; 
    close(connfd_);
    //client_thread_.join();
}

// fd: file descriptor
void Client::receiveLoop() {
    // test simple connection
    /*char buffer[1024] = { 0 };
    char* msg = "Hello from server";
    int valread = read(connfd_, buffer, 1024);
    printf("%s\n", buffer);
    send(connfd_, msg, strlen(msg), 0);
    printf("Hello message sent\n");
    printf("client %d closed \n",id_);*/

    unsigned char header[2] = { 0 };
    unsigned char buffer[1024] = {0}; 
    int valread = 0; 
    while (valread!=-1&&recvFlag){
        valread = read(connfd_, header, 2);
        if (valread!=2) {
            cout << "client " << id_ << " disconnected!" << endl; 
            break;
        }
        // get the packet size
        unsigned short size = ((unsigned short) header[0])*256 + (unsigned short)header[1]; 
        //cout << "current packet size: " << size << endl; 
        unsigned char payload[size];
        int recv = 0;
        while (recv < size){
        if (recv + 1024 < size)
            valread = read(connfd_,buffer,1024);
        else
            valread = read(connfd_,buffer,size-recv);
        // copy to target payload
        for (int i=recv;i<recv+valread;i++)
            payload[i] = buffer[i-recv];
        recv = recv + valread; 
        }
        SlamPkt* pkt = new SlamPkt(payload,size); 
        vector<cv::KeyPoint> keypoints_ = pkt->getKeyPoints(); 
        cv::Mat descriptors_ = pkt->getDescriptors();
        int frameID = pkt->getFrameId(); 
        int gtID = pkt->getGroundTruthId(); 
        ORB_SLAM2::Frame* frame = new ORB_SLAM2::Frame(keypoints_, descriptors_, frameID, id_, gtID, extractor_, new ORB_SLAM2::ORBVocabulary(), mK, mDistCoef, mbf, mThDepth);
        server_->InsertFrame(frame); 

        //cout << "frame id " << frameID << endl;
        //cout << "keypoints " << keypoints_[10].pt << " " << keypoints_[71].pt << endl;
        //cout << "descriptors " << descriptors_.row(10) << endl;
    }
}