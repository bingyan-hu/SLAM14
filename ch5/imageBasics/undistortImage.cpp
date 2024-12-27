//
// Created by hby on 2024/12/21.
//
/*会间接包含 OpenCV 的常用模块，比如核心模块 (core)、
 * 图像处理模块 (imgproc)、高层接口模块 (highgui)、
 * 视频分析模块 (video)、机器学习模块 (ml)、特征检测模块
 * (features2d)、深度学习模块 (dnn) 等
 * */
#include <opencv2/opencv.hpp>
#include <string>

std::string image_file = "distorted.png";

int main(int argc, char **argv) {
    //opencv去畸变: cv::Undistort()

    //distort parameters 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    //intrinsics 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    /*CV_8U：表示数据类型为 8 位无符号整数（unsigned char），每个像素的取值范围为 0-255，
   * 适合存储图像像素强度值。C1：表示通道数为 1，即单通道，典型用于灰度图。
   * */
    cv::Mat image = cv::imread(image_file, 0);   // 图像是灰度图，CV_8UC1
    int rows = image.rows, cols = image.cols;
    //创建一个空白图像 image_undistort，大小和类型与原始图像一致，用于存储去畸变处理后的结果
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);

    for (int v = 0; v < rows; v++) {
        for (int u = 0; u < cols; u++) {
            double x = (u - cx) / fx, y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_distorted = x * (1 + k1 * pow(r, 2) + k2 * pow(r, 4)) + 2 * p1 * x * y
                    + p2 * (pow(r, 2) + 2 * pow(x, 2));
            double y_distorted = y * (1 + k1 * pow(r, 2) + k2 * pow(r, 4)) + 2 * p2 * x * y
                                 + p1 * (pow(r, 2) + 2 * pow(y, 2));
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;

            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows)
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            else
                image_undistort.at<uchar>(v, u) = 0;
        }
    }

    cv::imshow("distorted image", image);
    cv::imshow("undistorted image", image_undistort);
    cv::waitKey();
    return 0;
}
