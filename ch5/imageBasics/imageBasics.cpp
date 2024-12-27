//
// Created by hby on 2024/12/20.
//
#include <iostream>
//用于处理时间点、时间间隔、时钟等
//主要用于精确控制时间、测量程序执行时间、处理延迟和定时任务等操作
#include <chrono>

/*<opencv2/core/core.hpp>
 * 主要用于矩阵处理和数值运算
 * 1. 数据结构（Mat）：提供了 cv::Mat 类，这是 OpenCV 中用于表示图像、视频帧等数据的基本数据结构。
 * cv::Mat 类用于存储和操作多维数据（例如图像像素矩阵）。
 * 2. 数学运算：提供各种数学运算函数，支持矩阵运算、线性代数运算、统计函数等
 * 3. 基础操作：支持像素级图像处理、颜色空间转换、几何变换（旋转、平移等）、滤波等基本操作。
 * */
#include <opencv2/core/core.hpp>
/*<opencv2/highgui/highgui.hpp>
 * 该头文件提供了 OpenCV 的图形用户界面（GUI）功能，主要用于图像显示和用户交互
 * 1. 图像显示：提供 cv::imshow() 等函数，用于显示图像或视频
 * 2. 窗口控制：创建图像显示窗口，设置窗口的名称、大小、关闭操作等
 * 3. 用户交互：可以处理键盘输入（例如 cv::waitKey()），允许程序在显示图像时等待用户输入，执行某些操作。
 * 4. 视频输入和输出：通过 cv::VideoCapture 和 cv::VideoWriter 类，可以读取和写入视频文件
 */
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    cv::Mat image;
    image = cv::imread(argv[1]);

    if (image.data == nullptr) {
        std::cerr << "file " << argv[1] <<" is not exist." << std::endl;
        return 0;
    }

    //channels(): 1. gray 1 ;2. RGB 3; 3. RGBA 4
    std::cout << "Image's width is " << image.cols << ", height is " << image.rows << ", channels are " <<
    image.channels() << std::endl;

    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        std::cout << "please input an RGB image or gray image." << std::endl;
        return 0;
    }

    // 遍历图像, 请注意以下遍历方式亦可使用于随机像素访问
    // 使用 std::chrono 来给算法计时
    //std::chrono::steady_clock 是 C++ 中一种时钟类型，专门用于测量时间间隔:单调递增;不受系统时间调整影响
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++) {
        // 用cv::Mat::ptr获得图像的行指针
        unsigned char *row_pr = image.ptr<unsigned char>(y);
        for (size_t x = 0; x < image.cols; x++) {
            unsigned char *date_ptr = &row_pr[x * image.channels()];

            for (int c = 0; c != image.channels(); c++) {
                unsigned char date = date_ptr[c];
            }
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //std::chrono::duration_cast 是 std::chrono 库中用于类型转换的工具函数，用来将一个 duration 对象从一种单位转换为另一种单位
    std::chrono::duration<double> time_used = std::chrono::duration_cast < std::chrono::duration <double >> (t2 - t1);
    std::cout << "It takes " << time_used.count() << " s to traverse image." << std::endl;

    // 关于 cv::Mat 的拷贝
    // 直接赋值并不会拷贝数据
    cv::Mat image_another = image;
    // 修改 image_another 会导致 image 发生变化
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 将左上角100*100的块置零 black
    cv::imshow("image", image);
    cv::waitKey(0);


    cv::Mat image_clone = image.clone();
    /*这行代码的作用是将 image_clone 图像的左上角（从 (0, 0) 到 (100, 100)）的 100x100
     * 区域的所有像素值设置为 255，即将该区域填充为白色。
     * cv::Rect 是 OpenCV 中用于表示矩形区域的类:(0, 0) 到 (100, 100) 之间的区域
     * setTo() 是 cv::Mat 类的一个方法，用来设置图像的像素值
     * */
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();

    return 0;


}

