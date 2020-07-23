#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace std;

int main(){
    // Homogenous
    Eigen::Vector3f p(2.0, 1.0, 1.0);
    Eigen::Matrix3f R;
    R << cos(M_PI / 4), -sin(M_PI / 4), 1,
         sin(M_PI / 4), cos(M_PI / 4), 2,
         0, 0, 1;
    cout << R * p << endl;
    return 0;
}
