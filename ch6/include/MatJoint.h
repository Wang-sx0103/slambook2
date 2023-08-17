#if !defined(MATJOINT_H)
#define MATJOINT_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

class MatJoint
{
private:
    int _size;
    std::vector<double> vecArr;
    Eigen::MatrixXd* matEigen;

public:
    MatJoint();
    MatJoint(const int);
    void Append(std::vector<double>&);
    void Append(const Eigen::Matrix3d&);
    void Append(const Eigen::Vector3d&);
    Eigen::MatrixXd* Vec2Matrix(const int row, const int col);
    void Vec2Matrix(Eigen::MatrixXd&);
    void Vec2Matrix(Eigen::VectorXd&);
    ~MatJoint();
};

#endif // MATJOINT_H
