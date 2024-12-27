//
// Created by hby on 2024/12/19.
//
#include <iostream>
#include <unistd.h>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>

std::string groundtruth_file = "groundtruth.txt";
std::string estimated_file = "estimated.txt";

typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

TrajectoryType ReadTajectory(const std::string &path);
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

int main(int argc, char **argv) {
    TrajectoryType groundtruth = ReadTajectory(groundtruth_file);
    TrajectoryType estimated = ReadTajectory(estimated_file);

    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    double rmse = 0;
    for (size_t i; i < estimated.size(); i++) {
        Sophus::SE3d p1 = estimated[i];
        Sophus::SE3d p2 = groundtruth[i];
        double error = (p1.inverse() * p2).log().norm();
        rmse += error;
    }
    rmse = sqrt(rmse / double(estimated.size()));
    std::cout << "rmse = " << rmse << std::endl;

    DrawTrajectory(groundtruth, estimated);
    return 0;
}

TrajectoryType ReadTajectory(const std::string &path) {
    std::ifstream fin(path);
    TrajectoryType trajectory;
    if(!fin) {
        std::cerr << "trajectory " << path << " not found." << std::endl;
        return trajectory;
    }
    while(!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p1(Eigen::Quaternion(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(p1);
    }
    return trajectory;
}


void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -0.8, 0, 0, 0, 0.0, -1.0, 0.0)
            );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size(); i++) {
            glColor3f(0.0f, 0.1f, 1.0f);  // blue for ground truth
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);

    }

}
