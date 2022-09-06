# include "client.h"

Client::Client(int id, int connfd, const string settingFile, Server* server):
id_(id), connfd_(connfd), nFeaturesInit(1000), nFeatures(200), recvFlag(true), initFlag(true) {
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

    // add new tracking thread in SLAM system
    server_->system->AddClient(id_); 

    // send the number of feature points for initialization
    sendMsg(nFeaturesInit); 

    client_thread_ = std::thread(&Client::receiveLoop,this); 
    tracking_thread_ = std::thread(&Client::trackLoop,this);
}

void Client::Close(){
    recvFlag = false; 
    close(connfd_);
    //client_thread_.join();
    tracking_thread_.join();
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
        int frameID = pkt->getFrameId()+100000*id_; 
        int gtID = pkt->getGroundTruthId(); 
        ORB_SLAM2::Frame* frame = new ORB_SLAM2::Frame(keypoints_, descriptors_, frameID, id_, gtID, extractor_, server_->system->getVocabulary(), mK, mDistCoef, mbf, mThDepth);
        //server_->InsertFrame(frame); 
        InsertFrame(frame); 

        //cout << "frame id " << frameID << endl;
        //cout << "keypoints " << keypoints_[10].pt << " " << keypoints_[71].pt << endl;
        //cout << "descriptors " << descriptors_.row(10) << endl;
    }
}

void Client::trackLoop(){
    while(recvFlag){
        if (CheckNewFrame()){
            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
            #endif
            ORB_SLAM2::Frame* im= GetNewFrame();  
            //int clientID = im->clientId; 
            cout << "client: " << id_ << " frame id: " << im->mnId << " number of feature points: " << im->mvKeys.size() << endl;  
            cv::Mat tcw = server_->system->TrackEdge(im);
            // detect the state of tracking to change the number of feature points
            if (!initFlag && server_->system->GetTrackingState(id_)!=2) {
                sendMsg(nFeaturesInit);
                initFlag = true; 
            }
            if (initFlag && server_->system->GetTrackingState(id_)==2) {
                sendMsg(nFeatures); 
                initFlag = false; 
            }
            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
            #endif
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            trajectory.push_back(tcw); 
            vTimesTrack.push_back(ttrack); 
            vTimestamps.push_back(im->mTimeStamp);
            trajectory_gt_points.push_back(im->groundTruthID);
        }
        else
            usleep(3000); 
    }
}

bool Client::sendMsg(int num){
    CmdPkt* pkt = new CmdPkt(0,num);  
    unsigned char* head = pkt->getHead();
    if (head == nullptr) return false;
    send(connfd_, head, 2, 0);
    int size = pkt->getTotalLength();
    unsigned char* payload = pkt->getPayload();
    send(connfd_, payload, size, 0);
    return true; 
}

void Client::InsertFrame(ORB_SLAM2::Frame *pF)
{
    unique_lock<mutex> lock(mMutexClient);
    mlNewFrames.push_back(pF);
}

bool Client::CheckNewFrame()
{
    unique_lock<mutex> lock(mMutexClient);
    return(!mlNewFrames.empty());
}

ORB_SLAM2::Frame* Client::GetNewFrame()
{
    unique_lock<mutex> lock(mMutexClient);
    ORB_SLAM2::Frame* mpCurrentFrame = mlNewFrames.front();
    mlNewFrames.pop_front();
    return mpCurrentFrame; 
}
