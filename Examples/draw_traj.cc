
#include<iostream>
#include<vector>
#include<unistd.h>

#include<opencv2/core/core.hpp>
#include<pangolin/pangolin.h>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std; 
void loadGroundTruth(string str_path, vector<double> &times, vector<g2o::SE3Quat> &gt); 
void loadTrajectory(string str_path, vector<double> &times, vector<g2o::SE3Quat> &traj, vector<int> &gt_marks); 
void DrawTrajectory(const vector<g2o::SE3Quat> & esti, const vector<g2o::SE3Quat> & gt, vector<int> &gt_marks);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./draw_traj path_to_aligned_trajectory(with ground truth mark) path_to_ground_truth" << endl;
        return 1;
    }

    vector<double> traj_times; 
    vector<int> gt_marks; 
    vector<g2o::SE3Quat> traj;
    loadTrajectory(argv[1],traj_times,traj,gt_marks); 

    vector<double> gt_times;
    vector<g2o::SE3Quat> gt;
    loadGroundTruth(argv[2],gt_times,gt);
    //for (size_t i=0; i<gt.size();i++)
    //    cout << gt[i] << endl; 
    
    DrawTrajectory(traj,gt,gt_marks); 
}

void loadGroundTruth(string str_path, vector<double> &times, vector<g2o::SE3Quat> &gt ){
    // read ground truth trajectory
    ifstream fin(str_path);
    if (!fin) {
        cout << "cannot find " << str_path << "!" << endl;
        return;
    }

    while (!fin.eof())
    {
        string ground_truth_time, tx, ty, tz, qx, qy, qz, qw, index;
        fin >> ground_truth_time >> tx >> ty >> tz >> qx >> qy >> qz >> qw >> index;
        
        if (fin.good() == false)
            break;

        Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
        Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()) );
        g2o::SE3Quat pose(q,pos);
        gt.push_back(pose);
        times.push_back(atof(ground_truth_time.c_str()));

    }
    
    fin.close();

    return;
}

void loadTrajectory(string str_path, vector<double> &times, vector<g2o::SE3Quat> &traj, vector<int> &gt_marks){
    // read trajectory by ORB SLAM
    ifstream fin(str_path);
    if (!fin) {
        cout << "cannot find " << str_path << "!" << endl;
        return;
    }
    int cnt = 0; 
    while (!fin.eof())
    {
        string trajectory_time, tx, ty, tz, qx, qy, qz, qw, mark;
        fin >> trajectory_time >> tx >> ty >> tz >> qx >> qy >> qz >> qw >> mark;
        if (cnt < 3)
            cout << " traj: " << trajectory_time <<  " tx: " << tx << " ty: " << ty << " tz: " << tz 
            << " qx: " << qx << " qy: " << qy << " qz: " << qz << " qw: " << qw << " mark: " << mark; 
        cnt++; 
        if (fin.good() == false)
            break;

        Eigen::Vector3d pos(atof(tx.c_str()), atof(ty.c_str()), atof(tz.c_str()));
        //Eigen::Quaterniond q(atof(qw.c_str()), atof(qx.c_str()), atof(qy.c_str()), atof(qz.c_str()) );
        Eigen::Quaterniond q; 
        g2o::SE3Quat pose(q,pos);
        traj.push_back(pose);
        times.push_back(atof(trajectory_time.c_str()));
        gt_marks.push_back((int)atof(mark.c_str()));

    }
    
    fin.close();

    return;
}

void DrawTrajectory(const vector<g2o::SE3Quat> & esti, const vector<g2o::SE3Quat> & gt, vector<int> &gt_marks) {
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

    int end_i = 2000; 
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);

        //draw axis
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f); // red for x axis
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(1.0, 0.0, 0.0);
        glColor3f(0.0f, 1.0f, 0.0f); // green for y axis
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 1.0, 0.0);
        glColor3f(0.0f, 0.0f, 1.0f); // blue for z axis
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, 1.0);
        glEnd(); 

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

        if (end_i >= esti.size()) end_i = esti.size() - 1; 
        for (size_t i = 0; i < end_i; i++) {
            g2o::SE3Quat tcw = esti[i];
            g2o::SE3Quat tcw2 = esti[i + 1];
            if (gt_marks[i]==0){
                glLineWidth(2);
                glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            }
            else {
                glLineWidth(6);
                glColor3f(0.0f, 0.0f, 0.0f);  // black for marked points
            }
            glBegin(GL_LINES);
            Eigen::Vector3d p1 = tcw.translation();
            Eigen::Vector3d p2 = tcw2.translation();
            glVertex3d(p1(0, 0), p1(1, 0), p1(2, 0));
            glVertex3d(p2(0, 0), p2(1, 0), p2(2, 0));
            glEnd();
        }
        std::ostringstream ss;
        ss << "frame: " << end_i; 
        pangolin::GlFont::I().Text(ss.str()).Draw(0,0,0);
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
        end_i++; 
    }
}

