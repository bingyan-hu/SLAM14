//
// Created by hby on 2024/12/18.
//
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>

//using namespace std;
//using namespace Eigen;

//How to use Eigen/Geometry
int main(int argc, char **argv){
    //3D Rotation Matrix: Matrix3d/Matrix3f
    Eigen::Matrix3d rotation_matrix  = Eigen::Matrix3d::Identity();

    //Rotation Vector
    Eigen::AngleAxisd rotation_vector(M_PI / 2, Eigen::Vector3d(0, 0, 1));
    std::cout.precision(3);
    //no match for ‘operator<<’
    //std::cout << "rotation vector = \n" << rotation_vector << std::endl;
    std::cout << "rotation matrix = \n" << rotation_vector.matrix() << std::endl;
    rotation_matrix = rotation_vector.toRotationMatrix();
    std::cout << "rotation matrix = \n" << rotation_matrix << std::endl;

    //use AngleAxis to rotate
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    std::cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << std::endl;

    //use RotationMatrix to rotate
    v_rotated = rotation_matrix * v;
    std::cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << std::endl;

    //euler angle
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);//ZYX: yaw-pitch-roll
    std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl;

    // 欧氏变换矩阵使用 Eigen::Isometry
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  //3d x; 4*4 matrix  //T = [R t;0 1]
    T.rotate(rotation_vector);  //Rotate according to rotation_vector
    T.pretranslate(Eigen::Vector3d(1, 3, 4));  //translate vector: (1,3,4)
    std::cout << "Transform matrix = \n" << T.matrix() << std::endl;

    //use Isometry to transform matrix
    Eigen::Vector3d v_transformed = T * v;    // R*v+t
    std::cout << "v transformed = \n" << v_transformed.transpose() << std::endl;

    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可
    //Eigen::Affine3d affine = Eigen::Affine3d(rotation_matrix/rotation_vector/quaternion);
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.rotate(rotation_matrix);//affine.rotate(rotation_vector);
    affine.translate(Eigen::Vector3d(1, 2, 3));
    //affine.scale(2.0);
    affine.scale(Eigen::Vector3d(2, 1, 3));
    std::cout << "Affine transformation matrix:\n" << affine.matrix() << std::endl;
    Eigen::Vector3d point(1, 1, 1);
    Eigen::Vector3d transformed_point = affine * point;
    std::cout << "Transformed point: " << transformed_point.transpose() << std::endl;

    //Projective3d
    Eigen::Projective3d projective = Eigen::Projective3d::Identity();
    Eigen::Matrix4d projection;
    projection << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,  // 透视投影分量
            0, 0, 0, 1;
    projective.matrix() = projection;
    // 变换点（齐次坐标点）
    Eigen::Vector4d p(1, 1, 1, 1); // 齐次坐标
    Eigen::Vector4d transformed_p = projective * p;
    // 打印变换结果
    std::cout << "Transformed point (homogeneous): " << transformed_point.transpose() << std::endl;
    // 归一化为三维点
    //static assertion failed: YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES
    //Eigen::Vector3d normalized_point = transformed_point.hnormalized();
    Eigen::Vector3d normalized_point = transformed_p.head<3>() / transformed_p.w();
    std::cout << "Transformed point (normalized): " << normalized_point.transpose() << std::endl;

    //quaternion
    //AngleAxis <=> Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl;
    //rotationMatrix <=> Quaternion
    q = Eigen::Quaterniond(rotation_matrix);
    std::cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << std::endl;

    v_rotated = q * v;
    std::cout << "(1,0,0) after rotation = " << v_rotated.transpose() << std::endl;
    // 用常规向量乘法表示，则应该如下计算
    std::cout << "should be equal to " << (q * Eigen::Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << std::endl;

    return 0;

}
