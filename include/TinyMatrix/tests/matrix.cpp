#include "../TinyMatrix.h"
#include <cmath>
using namespace TinyMatrix;

int main() {
    Matrix<float,3,3> mat1;
    std::cout << "mat1 = \n" << mat1;
    Matrix<float,4,4> mat2 = Matrix<float,4,4>::Identity();
    std::cout << "mat2 = \n" << mat2;
    std::cout << "mat2[2] = \n" << mat2.GetColumn(2);
    std::cout << "Row 2 = \n" << mat2.GetRow(2);

    float mat3_data[2][3] = {
        {1, 2, 3},
        {4, 5, 6},
    };

    Matrix<float, 2,3> mat3 = Matrix<float,2,3>(mat3_data);
    std::cout << "mat3 = \n" << mat3;
    std::cout << "mat3(1,0) = " << mat3(1,0) << std::endl;

    Matrix<float,3,4> mat4 = mat3.Resize<3,4>();
    std::cout << "mat4 = \n" << mat4;

    // initializer_list
    Matrix<float,3,2> mat5 = {
        1, 2,
        3, 4,
        5, 6
    };
    std::cout << "mat5 = \n" << mat5;

    auto mat6 = mat3.SubMatrix<2,2>(0,1);
    std::cout << "mat6 = \n" << mat6;

    double alpha = 0.4;
    double mat_data_v1[2][2] = {
            {sin(alpha),sin(alpha+1.57)},
            {cos(alpha),cos(alpha+1.57)},

    };

    Matrix<double,2,2> mat_1 = {
            cos(alpha),cos(alpha+1.57),
            sin(alpha),sin(alpha+1.57)
    };
    Matrix<double,2,2> mat_2 = {
            3,0,
            0,1
    };
    Matrix<double,2,2> mat_3 = mat_1.Transpose();

    Matrix<double,2,2> mat_4 = mat_1*mat_2*mat_3;
    std::cout<<"mat_2"<<mat_2;

    std::cout<<"mat_4"<<mat_4;
    Matrix<double,2,2> mat_5 = {
            2,3,
            2,1
    };


    double a = mat_4(0,0);
    double b = mat_4(0,1);
    double c = mat_4(1,0);
    double d = mat_4(1,1);

    int solver_ = false;
    double right_value = ((pow(0.5*(a+d),2)) - (a*d - b*c));
    solver_ = right_value >= 0.0;
    double numda1 = 0.5*(a+d) + sqrt(right_value);
    double numda2 = 0.5*(a+d) - sqrt(right_value);


    std::cout<<"solver_"<<solver_<<"numda1: "<<numda1<<",numda2: "<<numda2;
    Matrix<double,2,1> eginv1 = {
            b,
            numda1 - a
    };
    Matrix<double,2,1> eginv2 = {
            numda2 - d,
            c
    };
    Matrix<double,2,2> eginv = {
            b,(numda2 - d),
            numda1 - a,c
    };
    eginv = (1.0/sqrt(pow(eginv(0,0),2) + pow(eginv(1,0),2)))*eginv;
    Matrix<double,2,2>mat_6 = {numda1,0,0,numda2};
    Matrix<double,2,2> mat_7 = eginv*mat_6*eginv.Inverse();

    std::cout<<"eginv1"<<eginv1<<"eginv2"<<eginv2<<"eginv"<<eginv<<"eginv_-1"<<eginv.Inverse();
    std::cout<<"mat_7"<<mat_7<<"mat_6"<<mat_6<<"mat_1"<<mat_1<<"mat_3"<<mat_3;


    double alpha2 = atan2(eginv(1,0),eginv(0,0));
    std::cout<<"alpha:"<<alpha<<"alpha2:"<<alpha2;



}
