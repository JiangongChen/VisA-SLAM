/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<Converter.h>
#include<server.h>
#include<Frame.h>

using namespace std;

void LoadGroundTruth(const string &strFile, vector<cv::Mat> &gtTraj); 
void DrawTrajectory(const vector<cv::Mat> & esti, const vector<cv::Mat>& gt, vector<int> gt_points);
void SaveTrajectory(const string& filename, const vector<cv::Mat>& trajectory, vector<double> timeStamps, vector<int> trajGTpts);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_pixel path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // read ground truth data TODO: add align
    //string strGt = string(argv[3]) + "/associate.txt";
    //vector<cv::Mat> gtTrajectory; 
    //LoadGroundTruth(strGt, gtTrajectory);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System *SLAM = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<double> vTimesTrack;
    vector<cv::Mat> trajectory; 
    vector<double> vTimestamps; 
    vector<int> trajectory_gt_points; 

    cout << endl << "-------" << endl;

    // Main loop
    Server* server = new Server(argv[2], SLAM); 

    server->StartListening(); 
    while(server->listenFlag){
        if (server->CheckNewFrame()){
            #ifdef COMPILEDWITHC11
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            #else
                    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
            #endif
            ORB_SLAM2::Frame* im= server->GetNewFrame();  
            im->mpORBvocabulary = SLAM->getVocabulary(); // update the vocabulary since we set an empty one before 
            cout << "frame id: " << im->mnId << endl;  
            cv::Mat tcw = SLAM->TrackEdge(im);
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

    // request close of the server
    server->Close(); 

    cout << "request stop thread" << endl; 
    // Stop all threads
    SLAM->Shutdown();

    // Tracking time statistics
    int nImages = vTimesTrack.size(); 
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    //draw and save the trajectory
    vector<cv::Mat> gt; 
    DrawTrajectory(trajectory, gt, trajectory_gt_points);
    SaveTrajectory("allTrajectory.txt",trajectory,vTimestamps,trajectory_gt_points);

    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadGroundTruth(const string& strFile, vector<cv::Mat>& gtTraj) {
    ifstream fin(strFile);
    if (!fin) {
        cout << "cannot find " << strFile << "!" << endl;
        return;
    }


    while (!fin.eof())
    {
        string rgb_time, rgb_file, ground_truth_time, tx, ty, tz, qx, qy, qz, qw;
        fin >> rgb_time >> rgb_file >> ground_truth_time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
        Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()));
        cv::Mat pose; 
        gtTraj.push_back(pose);

        if (fin.good() == false)
            break;
    }
    fin.close();
}

void DrawTrajectory(const vector<cv::Mat>& esti, const vector<cv::Mat>& gt, vector<int> gt_points) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        if (!gt.empty()) {
            for (size_t i = 0; i < gt.size() - 1; i++) {// keep ground truth size the same as esti
                cv::Mat tcw = gt[i];
                cv::Mat tcw2 = gt[i + 1];
                if (tcw.empty() || tcw2.empty()) continue;
                glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
                glBegin(GL_LINES);
                cv::Mat t = -tcw.rowRange(0, 3).colRange(0, 3).t() * tcw.rowRange(0, 3).col(3);
                Eigen::Vector3d p1(t.at<float>(0), t.at<float>(1), t.at<float>(2));
                cv::Mat t2 = -tcw2.rowRange(0, 3).colRange(0, 3).t() * tcw2.rowRange(0, 3).col(3);
                Eigen::Vector3d p2(t2.at<float>(0), t2.at<float>(1), t2.at<float>(2));
                glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
                glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
                glEnd();
            }
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            cv::Mat tcw = esti[i];
            cv::Mat tcw2 = esti[i + 1];
            if (tcw.empty()||tcw2.empty()) continue;
            if (gt_points[i] == 0){
                glLineWidth(2);
                glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            }
            else {
                glLineWidth(6);
                glColor3f(0.0f, 0.0f, 1.0f); // blue for annotated ground truth measurement points
            }
            glBegin(GL_LINES);
            cv::Mat t = -tcw.rowRange(0, 3).colRange(0, 3).t() * tcw.rowRange(0, 3).col(3);
            Eigen::Vector3d p1(t.at<float>(0), t.at<float>(1), t.at<float>(2));
            cv::Mat t2 = -tcw2.rowRange(0, 3).colRange(0, 3).t() * tcw2.rowRange(0, 3).col(3);
            Eigen::Vector3d p2(t2.at<float>(0), t2.at<float>(1), t2.at<float>(2));
            glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
            glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

void SaveTrajectory(const string & filename, const vector<cv::Mat>&trajectory, vector<double> timeStamps, vector<int> trajGTpts) {
    ofstream f;
    f.open(filename.c_str());
    f << "# timeStamp tx ty tz qx qy qz qw groundTruthPoints" << endl; 
    f << fixed;
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        cv::Mat tcw = trajectory[i]; 
        if (tcw.empty()) continue; 
        cv::Mat R = tcw.rowRange(0, 3).colRange(0, 3).t();
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
        cv::Mat t = -R * tcw.rowRange(0, 3).col(3);
        f << setprecision(6) << timeStamps[i] << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
            << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << trajGTpts[i] << endl;
    }

    f.close();
    cout << endl << "all trajectory saved!" << endl;
}
