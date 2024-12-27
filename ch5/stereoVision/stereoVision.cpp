//
// Created by hby on 2024/12/21.
//
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

std::string left_file = "left.png";
std::string right_file = "right.png";

void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);

int main(int argc, char **argv) {
    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 基线
    double b = 0.573;

    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    /* 1. cv::Ptr<cv::StereoSGBM>
     * cv::Ptr 是 OpenCV 提供的一个智能指针模板，用于自动管理内存，避免手动释放对象
     * 2. StereoSGBM::create
     * 这是一个静态工厂方法，用于创建一个 StereoSGBM 对象，并初始化其参数。
     * */
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
    //disparity_sgbm：保存 SGBM 算法输出的原始视差图，其类型为 CV_16S（16 位带符号整数）。
    //disparity：保存最终的视差图，类型转换为 CV_32F（32 位浮点数）。
    cv::Mat disparity_sgbm, disparity;
    //通过 SGBM 立体匹配器计算视差图。
    //输入：左右两张对齐的灰度图（left 和 right）。
    //输出：视差图 disparity_sgbm，默认类型为 CV_16S。
    sgbm->compute(left, right, disparity_sgbm);
    //convertTo：
    //用于将矩阵从一种数据类型转换为另一种数据类型。
    //目标类型：CV_32F，将视差图的每个像素值从整数转为浮点数。
    //缩放因子：1.0 / 16.0f：
    //将原始视差值除以 16，恢复实际视差值。
    //例如，168 转换为浮点型后会变为 10.5。
    //结果保存在 disparity 中。
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    // 生成点云
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;

    for (int v = 0; v < left.rows; v++) {
        for (int u = 0; u < left.cols; u++) {
            if (disparity.at<float>(v,u) <= 0.0 || disparity.at<float>(v,u) >= 96.0) continue;

            Eigen::Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0);

            // 根据双目模型计算 point 的位置
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }
        cv::imshow("disparity", disparity / 96.0);
        cv::waitKey(0);
        showPointCloud(pointcloud);
        return 0;
    }

}

void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud) {
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
            glColor3f(p[3], p[3], p[3]);
            //每调用一次 glVertex，都会绘制一个点。
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);

    }
    return;
}
