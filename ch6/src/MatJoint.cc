#include "MatJoint.h"

MatJoint::MatJoint(/* args */)
{
}

MatJoint::MatJoint(const int size):_size(size)
{
    this->vecArr.reserve(_size);
}


void MatJoint::Append(std::vector<double>& vecJoint)
{
    this->vecArr.insert(this->vecArr.end(), vecJoint.begin(), vecJoint.end());
}


void MatJoint::Append(const Eigen::Matrix3d& H)
{
    std::vector<double> mat;
    for (int i = 0; i < 3; i++)
    {
        mat.push_back(H(i, 0));
        mat.push_back(H(i, 1));
        mat.push_back(H(i, 2));
    }
    this->vecArr.insert(this->vecArr.end(), mat.begin(), mat.end());
}

void MatJoint::Append(const Eigen::Vector3d& d)
{
    std::vector<double>  vec;
    for (int i = 0; i < 3; i++)
    {
        vec.push_back(d(i));
    }
    this->vecArr.insert(this->vecArr.end(), vec.begin(), vec.end());
}


void MatJoint::Vec2Matrix(Eigen::MatrixXd& mat)
{
    // this->matEigen = new Eigen::MatrixXd(row, col);
    // mat(row, col);
    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            mat(i, j) = this->vecArr[i*mat.cols()+j];
        }
    }
}

void MatJoint::Vec2Matrix(Eigen::VectorXd& vec)
{
    // this->matEigen = new Eigen::MatrixXd(row, col);
    // mat(row, col);
    for (int i = 0; i < vec.rows(); i++)
    {
        for (int j = 0; j < vec.cols(); j++)
        {
            vec(i, j) = this->vecArr[i*vec.cols()+j];
        }
    }
}


MatJoint::~MatJoint()
{
}

