//
// Created by hby on 2024/12/17.
//
#include <iostream>
//std::
//using namespace std;

#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

// Eigen::
//using namespace Eigen;

#define MATRIX_SIZE 50


int main(int argc, char **argv){
    Eigen::Matrix<float, 2, 3> matrix_23;

    //typedef Eigen::Matrix<double, 3, 1> Vector3d
    Eigen::Vector3d v_3d;
    Eigen::Matrix<float, 3, 1> vd_3d;

    //typedef Eigen::Matrix<double, 3, 3> Matrix3d
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();

    //typedef Matrix<double, Dynamic, Dynamic> MatrixXd
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    //matrix_dynamic.resize(2, 3);
    Eigen::MatrixXd matrix_x;

    matrix_23 << 1, 2, 3, 4, 5, 6;
    std::cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << std::endl;

    std::cout << "print matrix 2x3:" << std::endl;
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 3; j++) std::cout << matrix_23(i, j) << "\t";
        std::cout << std::endl;
    }

    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    //Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    //MatrixType cast<newType>() const;
    //float-->double
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "matrix_23 * v_3d : [1,2,3;4,5,6]*[1,2,3]: "<< result.transpose() << std::endl;
    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    std::cout << "matrix_23 * vd_3d : [1,2,3;4,5,6]*[4,5,6]: "<< result.transpose() << std::endl;

    //error: YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES
    //Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    //Matrix operations
    std::cout << "Zeros matrix_33: \n" << matrix_33 << std::endl;
    matrix_33 = Eigen::Matrix3d::Random();
    std::cout << "random matrix_33: \n" << matrix_33 << std::endl;
    std::cout << "transpose matrix_33: \n" << matrix_33.transpose() << std::endl;
    std::cout << "sum matrix_33: \n" << matrix_33.sum() << std::endl;
    std::cout << "trace matrix_33: \n" << matrix_33.trace() << std::endl;
    std::cout << "10 times matrix_33: \n" << matrix_33 * 10 << std::endl;
    std::cout << "inverse matrix_33: \n" << matrix_33.inverse() << std::endl;
    std::cout << "det matrix_33: \n" << matrix_33.determinant() << std::endl;

    //Eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vector = \n" << eigen_solver.eigenvectors() << std::endl;

    //Solving Equations: matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();
    //Direct inversion
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "time of normal inverse is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    //Matrix Decomposition(ex: QR)
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    std::cout << "time of QR decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    //For a positive definite matrix, using cholesky
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    return 0;


}