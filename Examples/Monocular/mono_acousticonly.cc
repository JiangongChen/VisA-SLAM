#include<System.h>
#include<Converter.h>
#include<Frame.h>
#include<Optimizer.h>
#include "ORBextractor.h"
#include "ORBVocabulary.h"
#include "slampkt.h"

using namespace std;
using namespace ORB_SLAM2;

void loadGroundTruth(string str_path, vector<double> &times, vector<g2o::SE3Quat> &gt); 
void loadTrajectory(string str_path, vector<double> &times, vector<g2o::SE3Quat> &traj); 
void DrawTrajectory(const vector<g2o::SE3Quat> & esti, const vector<g2o::SE3Quat> & gt);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_sequence path_to_trajectory feature_points_num" << endl;
        return 1;
    }

    vector<double> gt_times;
    vector<g2o::SE3Quat> gt;
    loadGroundTruth(argv[1],gt_times,gt);
    
    vector<double> traj_times; 
    vector<g2o::SE3Quat> traj;
    loadTrajectory(argv[2],traj_times,traj); 

    int start_id = 0; 
    for (size_t i=0; i<gt_times.size(); i++){
        if (traj_times[0] - gt_times[i] < 0.01){
            start_id = i;
            cout << "align with ground truth, start with id " << start_id << endl; 
            break; 
        }
    }

    // transform from trajectory coordinates to ground truth coordinates
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tran;
    double scale = 1; 

    string fp_num_str = argv[3]; 
    int feature_point_num = (int) atof(fp_num_str.c_str()); 

    if (feature_point_num==500){
        // 500 feature points
        Rot <<
            -0.977, 0.089, -0.192,
            0.211,  0.462, -0.861,
            0.012, -0.883, -0.47;
        tran << 0.297, 2.467, 1.591;
        scale = 2.291558;
    }
    else if (feature_point_num==600){
        // 600 feature points
        Rot <<
            -0.999, -0.01,  -0.033,
            0.024,  0.494, -0.869,
            0.025, -0.87,  -0.493;
        tran << -0.654, 2.662, 1.764;
        scale = 2.458992;
    }
    else if (feature_point_num==700){
        // 700 feature points
        Rot <<
            -1.,    -0.012, -0.023,
            0.015,  0.479, -0.878,
            0.021, -0.878, -0.479;
        tran << -0.649, 2.633, 1.714;
        scale = 2.431670;
    }
    else if (feature_point_num==800){
        // 800 feature points
        Rot <<
            -1.,    -0.008, -0.025,
            0.018,  0.482, -0.876,
            0.019, -0.876, -0.482;
        tran << -0.657, 2.639, 1.725;
        scale = 2.419962;
    }
    else if (feature_point_num==900){
        // 900 feature points
        Rot <<
            -1.,    -0.007, -0.026,
            0.02,   0.482, -0.876,
            0.019, -0.876, -0.482;
        tran << -0.665, 2.658, 1.729;
        scale = 2.419214;
    }
    else if (feature_point_num==1000){
        // 1000 feature points
        Rot <<
            -0.999, -0.012, -0.03, 
            0.02,   0.485, -0.875,
            0.025, -0.875, -0.484;
        tran << -0.663, 2.661, 1.737;
        scale = 2.460156;
    }
    else{
        cout << "cannot find feauture point number " << feature_point_num << endl; 
        return 0; 
    }

    uint64_t now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
    cv::RNG* rng = new cv::RNG(now);

    vector<g2o::SE3Quat> traj_optimized; 
    // optimize the pose by acoustic ranging distance
    for (size_t i=0;i<traj.size();i++){
        int frameID = i+start_id; 
        vector<cv::KeyPoint> kps;
        //Frame* frame = new Frame(kps, cv::Mat::eye(4,4,CV_32F), frameID, ORBextractor(0,0,0,0,0), new ORB_SLAM2::ORBVocabulary(), cv::Mat::eye(4,4,CV_32F), cv::Mat::eye(4,4,CV_32F), 0, 0);
        Eigen::Vector3d pose = scale*Rot*traj[i].translation()+tran;
        Eigen::Vector3d pose_gt = gt[frameID].translation(); 
        Optimizer::PoseOptimizationDistance(pose,pose_gt,rng);
        traj_optimized.push_back(g2o::SE3Quat(traj[i].rotation(),pose)); 
    }

    // calculate the error
    double error = 0;
    for (size_t i=0; i<traj_optimized.size();i++){
        Eigen::Vector3d pose_est = traj_optimized[i].translation();
        Eigen::Vector3d pose_gt = gt[start_id+i].translation();
        double error_cur = (pose_est - pose_gt).norm(); 
        error += error_cur; 
        // cout << i << " " << pose_est << endl; 
        // cout << pose_gt << endl; 
    }
    cout << "calculated error: " << error/traj_optimized.size() << endl; 

    DrawTrajectory(traj_optimized,gt); 
    return 0; 
}

void loadGroundTruth(string str_path, vector<double> &times, vector<g2o::SE3Quat> &gt ){
    // read ground truth trajectory
    ifstream fin(str_path + "/associate.txt");
    if (!fin) {
        cout << "cannot find " << str_path << "/associate.txt!" << endl;
        return;
    }

    while (!fin.eof())
    {
        string rgb_time, rgb_file, ground_truth_time, tx, ty, tz, qx, qy, qz, qw;
        fin >> rgb_time >> rgb_file >> ground_truth_time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        
        Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
        Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()) );
        g2o::SE3Quat pose(q,pos);
        gt.push_back(pose);
        times.push_back(atof(rgb_time.c_str()));

        if (fin.good() == false)
            break;
    }
    
    fin.close();

    return;
}

void loadTrajectory(string str_path, vector<double> &times, vector<g2o::SE3Quat> &traj ){
    // read trajectory by ORB SLAM
    ifstream fin(str_path);
    if (!fin) {
        cout << "cannot find " << str_path << "!" << endl;
        return;
    }

    while (!fin.eof())
    {
        string trajectory_time, tx, ty, tz, qx, qy, qz, qw;
        fin >> trajectory_time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        
        Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
        Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()) );
        g2o::SE3Quat pose(q,pos);
        traj.push_back(pose);
        times.push_back(atof(trajectory_time.c_str()));

        if (fin.good() == false)
            break;
    }
    
    fin.close();

    return;
}

void DrawTrajectory(const vector<g2o::SE3Quat> & esti, const vector<g2o::SE3Quat> & gt) {
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
                g2o::SE3Quat tcw = gt[i];
                g2o::SE3Quat tcw2 = gt[i + 1];
                glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
                glBegin(GL_LINES);
                Eigen::Vector3d p1 = tcw.translation();
                Eigen::Vector3d p2 = tcw2.translation();
                glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
                glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
                glEnd();
            }
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            g2o::SE3Quat tcw = esti[i];
            g2o::SE3Quat tcw2 = esti[i + 1];
            glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            glBegin(GL_LINES);
            Eigen::Vector3d p1 = tcw.translation();
            Eigen::Vector3d p2 = tcw2.translation();
            glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
            glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}