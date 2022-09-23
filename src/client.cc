# include "client.h"

Client::Client(int id, int connfd, const string settingFile, Server* server, int total_c_num):
id_(id), connfd_(connfd), nFeaturesInit(1000), nFeatures(500), k_track_(5), recvFlag(true), recvFlagAcoustic(true), initFlag(true), num_users(total_c_num) {
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

void Client::AddAcoustic(int conn){
    connfd_ac_ = conn; 
    int yes = 1; 
    int result = setsockopt(connfd_ac_,
                            IPPROTO_TCP,
                            TCP_NODELAY,
                            (char *) &yes, 
                            sizeof(int));    // 1 - on, 0 - off
    if (result < 0)
          cout << "set TCP_NODELAY failed. " << endl; 
    for (int i=0;i<num_users;i++){
      queue<int> queue; 
      intervals.push_back(queue); 
    }
    acoustic_thread_ = std::thread(&Client::acousticLoop,this); 
}

void Client::Close(){
    recvFlag = false; 
    recvFlagAcoustic = false;
    close(connfd_);
    close(connfd_ac_); 
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
        long stamp = pkt->getTimeStamp(); 
        ORB_SLAM2::Frame* frame = new ORB_SLAM2::Frame(keypoints_, descriptors_, frameID, id_, gtID, stamp, extractor_, server_->system->getVocabulary(), mK, mDistCoef, mbf, mThDepth);
        //server_->InsertFrame(frame); 
        InsertFrame(frame); 

        //cout << "frame id " << frameID << endl;
        //cout << "keypoints " << keypoints_[10].pt << " " << keypoints_[71].pt << endl;
        //cout << "descriptors " << descriptors_.row(10) << endl;
        //cout << fixed << "time stamp " << frame->mTimeStamp << endl;
    }
}

void Client::acousticLoop(){
    char buffer[1024] = { 0 };
    std::ostringstream ss;
    ss << id_ << "\n"; 
    const char* start_msg = ss.str().c_str(); 
    send(connfd_ac_, start_msg, strlen(start_msg), 0); 
    std::cout << "client " << id_ << " acoustic established" << endl; 
    int valread = read(connfd_ac_, buffer, 1024);
    while(recvFlagAcoustic) {
        printf("received %s", buffer);
        string msg(buffer); 
        vector<string> tokens = split(msg,' ');
        int group_num = tokens.size()/2; 
        for (int i=0;i<group_num;i++) {
          cout << "client " << id_ << " calculated interval of client " << tokens[2*i+0] << ": " << tokens[2*i+1] << endl; 
          intervals[(int)atof(tokens[2*i+0].c_str())].push((int)atof(tokens[2*i+1].c_str())); 
        }
        cout << endl; 
        valread = read(connfd_ac_, buffer, 1024);
    }
    std::cout << "client " << id_ << " acoustic stopped" << endl; 
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
            // for the users (except user 0) which tracks normally, only track frames every k_track frames
            if (id_!=0 && !initFlag && im->mnId%k_track_!=0)
                continue;
            //int clientID = im->clientId; 
            //cout << "client: " << id_ << " frame id: " << im->mnId << " number of feature points: " << im->mvKeys.size() << endl;  
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
            updateTraj(tcw,ttrack,im->mTimeStamp,im->groundTruthID); 
        }
        else
            usleep(3000); 
    }
}

void Client::updateTraj(cv::Mat tcw, double ttrack, double timeStamp, int gt_id){
    unique_lock<mutex> lock(mMutexClient);
    trajectory.push_back(tcw); 
    vTimesTrack.push_back(ttrack); 
    vTimestamps.push_back(timeStamp);
    trajectory_gt_points.push_back(gt_id);
}

int Client::getLatestTraj(cv::Mat &mat){
    unique_lock<mutex> lock(mMutexClient);
    int size = trajectory.size()-1; 
    while(size>=0){
        mat = trajectory[size];
        if (!mat.empty()) break; 
        size--; 
    }
    return size;  
}

double Client::getLatestTS(){
    unique_lock<mutex> lock(mMutexClient);
    int size = vTimestamps.size()-1; 
    if (size>=0){
        return vTimestamps[size];  
    }
    return -1;  
}

void Client::rewriteTraj(int poseId, cv::Mat mat){
    unique_lock<mutex> lock(mMutexClient);
    if (poseId >= trajectory.size()) return;
    trajectory[poseId] = mat; 
    trajectory_gt_points[poseId] = -1; 
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

void Client::sendMsgAcoustic(char *msg) {
    send(connfd_ac_, msg, strlen(msg), 0); 
    //cout << "send msg " << msg << " to client " << id_ << endl; 
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
