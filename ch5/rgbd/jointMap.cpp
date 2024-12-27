//
// Created by hby on 2024/12/21.
//
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

int main(int argc, char **argv) {
    std::vector<cv::Mat> colorImgs, depthImgs;
    TrajectoryType poses;

    std::ifstream fin("pose.txt");
    if (!fin) {
        std::cerr << "Wrong path!" << std::endl;
        return 1;
    }

    for (int i = 0; i < 5; i++) {
        boost::format fmt("./%s/%d.%s"); //图像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        //cv::IMREAD_COLOR (默认, 值为 1) 将图像加载为彩色图像（BGR 三通道），即使图像本身是灰度图也会转换为三通道。
        //cv::IMREAD_GRAYSCALE (值为 0)	将图像加载为单通道灰度图。
        //cv::IMREAD_UNCHANGED (值为 -1)	按原始格式加载图像，不进行任何通道数或数据深度的转换。
        //深度图通常是 16 位无符号整数（uint16_t），而不是常见的 8 位图像。
        //如果不指定 -1，默认会使用 cv::IMREAD_COLOR，这将导致数据被强制转换为 8 位，失去精度。
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));

        //定义了一个固定长度为 7 的数组，所有元素初始化为 0
        double data[7] = {0};

        for (auto &d:data)
            fin >> d;
        Sophus::SE3d pose(Eigen::Quaternion(data[6], data[3], data[4], data[5]),
                          Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
    }

    // 计算点云并拼接
    // 相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    //用于将深度图中存储的原始深度值(mm)转换为实际的深度(m)
    double depthScale = 1000.0;
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    //预先为 pointcloud 分配存储空间，以提高性能并减少内存重新分配的开销
    pointcloud.reserve(1000000);

    for (int i = 0; i < 5; i++) {
        std::cout << "Converting image: " << i + 1 << std::endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Sophus::SE3d T = poses[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                //深度传感器（如 Kinect、RealSense）通常以 16 位无符号整数存储深度值。
                unsigned int d = depth.ptr<unsigned short>(v)[u];
                if (d == 0) continue;
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                // u = fx * (X / Z) + cx
                // v = fy * (Y / Z) + cy
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;

                Vector6d p;
                //head<N>(): 提取前 N 个元素（从索引 0 开始）。
                p.head<3>() = pointWorld;
                p[5] = color.data[v * color.step + u * color.channels()];
                p[4] = color.data[v * color.step + u * color.channels() + 1];
                p[3] = color.data[v * color.step + u * color.channels() + 2];
                pointcloud.push_back(p);
            }
    }
    std::cout << "There are " << pointcloud.size() <<" point cloud." << std::endl;
    showPointCloud(pointcloud);
    return 0;


}
void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {
    if (pointcloud.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    //启用深度测试，使得远离视点的物体在渲染时不会遮挡靠近视点的物体。
    glEnable(GL_DEPTH_TEST);
    //启用混合，使得透明物体的渲染效果更加平滑
    glEnable(GL_BLEND);
    //设置混合函数，控制透明物体如何与背景合成，常用于绘制带透明效果的物体
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //pangolin::OpenGlRenderState 定义了渲染状态，包括视图矩阵（相机位置、方向等）和投影矩阵（视角、近平面、远平面等）。
    pangolin::OpenGlRenderState s_cam(
            //设置透视投影矩阵。参数依次为：窗口宽度、窗口高度、焦距、水平和垂直视野范围、近平面和远平面。
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            //设置视图变换矩阵（相机的视点和观察方向）。
            // 它使得视点位于 (0, -0.1, -1.8)，并朝向 (0, 0, 0)，向上方向为 (0.0, -1.0, 0.0)。
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    //创建一个显示视图
    pangolin::View &d_cam = pangolin::CreateDisplay()
            //设置显示视图在窗口中的边界和大小。这里它设置为整个窗口的大小，并根据宽高比调整视图。
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                    //为显示窗口设置交互处理程序
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        //清空颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //激活视图和相机设置
        d_cam.Activate(s_cam);
        //设置背景颜色为白色
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        //包含在 glBegin 和 glEnd 之间的代码是定义绘图的内容
        //GL_POINTS 表示将以点的形式绘制每个顶点。
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            //每调用一次 glVertex，都会绘制一个点。
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);

    }
    return;
}
