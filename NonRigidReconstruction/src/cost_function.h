#ifndef COST_FUNCTION_H_LF
#define COST_FUNCTION_H_LF
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <lib_math/lib_math.h>


/**
 * @brief The CostFunctionRot class
 * Refers to "Embdedded Deformation", but here I calcuate
 *    Rot(R) = (|| R'*R - I ||_F)^2
 */
class CostFunctionRot : public ceres::CostFunction
{
public:
    CostFunctionRot(float cost_function_weight);

    ~CostFunctionRot(){}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals, double** jacobians) const;

private:
    double _cost_func_weight;
};


/**
 * @brief The RegCostFunction class
 * Regularization term
 * Refers to "Embdedded Deformation"
 */
class CostFunctionReg : public ceres::CostFunction
{
public:
    CostFunctionReg(double cost_function_weight,
                    const Eigen::Vector3d& g_j, const Eigen::Vector3d& g_k,
                    double alpha_j_k = 1.0);

    ~CostFunctionReg(){}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals, double** jacobians) const;

private:
    double _cost_func_weight;
    Eigen::Vector3d _g_j;
    Eigen::Vector3d _g_k;
};


/**
 * @brief The CostFunctionData class
 * The data term
 * Refers to "Fusion4d"
 */
class CostFunctionData
{
public:
    CostFunctionData(double cost_function_weight,
                     const Eigen::Vector3d& visible_points,
                     const Eigen::Vector3d& visible_points_normal,
                     const std::vector<Eigen::Vector3d>& neighbor_nodes,
                     const Eigen::VectorXd& w_j,
                     float fxy, float cx, float cy, float t,
                     const cv::Mat& depthmap,
                     const Eigen::Matrix3d& global_R,
                     const Eigen::Vector3d& global_t);

    ~CostFunctionData(){}

    // Refers to Fusion4d Eq(2),Eq(5)
    template <typename T>
    bool operator()(T const* const* parameters, T* residuals) const {
        T deformed_pt[3] = {T(0), T(0), T(0)};
        T deformed_n[3] = {T(0), T(0), T(0)};
        for(size_t i = 0; i < _neighbor_nodes.size(); i++) {
            const T& R11 = parameters[i][0];
            const T& R21 = parameters[i][1];
            const T& R31 = parameters[i][2];
            const T& R12 = parameters[i][3];
            const T& R22 = parameters[i][4];
            const T& R32 = parameters[i][5];
            const T& R13 = parameters[i][6];
            const T& R23 = parameters[i][7];
            const T& R33 = parameters[i][8];
            const T& t0 = parameters[i][9];
            const T& t1 = parameters[i][10];
            const T& t2 = parameters[i][11];

//            deformed_pt += _w_j[i] *
//                    (R_j * (_visible_point - _neighbor_nodes[i]) +
//                     _neighbor_nodes[i] + t_j);
            deformed_pt[0] += _w_j[i] *
                    (R11 * (_visible_point[0] - _neighbor_nodes[i][0]) +
                     R12 * (_visible_point[1] - _neighbor_nodes[i][1]) +
                     R13 * (_visible_point[2] - _neighbor_nodes[i][2]) +
                     _neighbor_nodes[i][0] + t0);
            deformed_pt[1] += _w_j[i] *
                    (R21 * (_visible_point[0] - _neighbor_nodes[i][0]) +
                     R22 * (_visible_point[1] - _neighbor_nodes[i][1]) +
                     R23 * (_visible_point[2] - _neighbor_nodes[i][2]) +
                     _neighbor_nodes[i][1] + t1);
            deformed_pt[2] += _w_j[i] *
                    (R31 * (_visible_point[0] - _neighbor_nodes[i][0]) +
                     R32 * (_visible_point[1] - _neighbor_nodes[i][1]) +
                     R33 * (_visible_point[2] - _neighbor_nodes[i][2]) +
                     _neighbor_nodes[i][2] + t2);


//            deformed_n += _w_j[i] * R_j * _visible_point_normal;
            deformed_n[0] += _w_j[i] * (R11 * _visible_point_normal[0] +
                                        R12 * _visible_point_normal[1] +
                                        R13 * _visible_point_normal[2]);
            deformed_n[1] += _w_j[i] * (R21 * _visible_point_normal[0] +
                                        R22 * _visible_point_normal[1] +
                                        R23 * _visible_point_normal[2]);
            deformed_n[2] += _w_j[i] * (R31 * _visible_point_normal[0] +
                                        R32 * _visible_point_normal[1] +
                                        R33 * _visible_point_normal[2]);
        }
        //deformed_n.normalize();
        T n_norm = sqrt(deformed_n[0]*deformed_n[0] +
                deformed_n[1]*deformed_n[1] + deformed_n[2]*deformed_n[2]);
        deformed_n[0] /= n_norm;
        deformed_n[1] /= n_norm;
        deformed_n[2] /= n_norm;

        // Transformed by global
        T deformed_pt_G[3] = {T(0), T(0), T(0)};
        deformed_pt_G[0] = _global_R(0, 0) * deformed_pt[0] +
                _global_R(0, 1) * deformed_pt[1] +
                _global_R(0, 2) * deformed_pt[2] + _global_t[0];
        deformed_pt_G[1] = _global_R(1, 0) * deformed_n[0] +
                _global_R(1, 1) * deformed_pt[1] +
                _global_R(1, 2) * deformed_pt[2] + _global_t[1];
        deformed_pt_G[2] = _global_R(2, 0) * deformed_n[0] +
                _global_R(2, 1) * deformed_pt[1] +
                _global_R(2, 2) * deformed_pt[2] + _global_t[2];
        deformed_pt[0] = deformed_pt_G[0];
        deformed_pt[1] = deformed_pt_G[1];
        deformed_pt[2] = deformed_pt_G[2];

        T deformed_n_G[3] = {T(0), T(0), T(0)};
        deformed_n_G[0] = _global_R(0, 0) * deformed_n[0] +
                _global_R(0, 1) * deformed_n[1] +
                _global_R(0, 2) * deformed_n[2];
        deformed_n_G[1] = _global_R(1, 0) * deformed_n[0] +
                _global_R(1, 1) * deformed_n[1] +
                _global_R(1, 2) * deformed_n[2];
        deformed_n_G[2] = _global_R(2, 0) * deformed_n[0] +
                _global_R(2, 1) * deformed_n[1] +
                _global_R(2, 2) * deformed_n[2];
        deformed_n[0] = deformed_n_G[0];
        deformed_n[1] = deformed_n_G[1];
        deformed_n[2] = deformed_n_G[2];


        T u = (deformed_pt[0] + T(_t) / 2.0) / deformed_pt[2] * T(_fxy) + T(_cx);
        T v = deformed_pt[1] / deformed_pt[2] * T(_fxy) + T(_cy);
        T d;
        _depth->Evaluate(v, u, &d);
        if(d < 30) {
            residuals[0] = T(0);
        }
        else{
            T back_projected_point[3];
            back_projected_point[0] = (u - T(_cx)) / T(_fxy) * d - T(_t) / 2.0;
            back_projected_point[1] = (v - T(_cy)) / T(_fxy) * d;
            back_projected_point[2] = d;

            residuals[0] =
                    deformed_n[0] * (deformed_pt[0] - back_projected_point[0]) +
                    deformed_n[1] * (deformed_pt[1] - back_projected_point[1]) +
                    deformed_n[2] * (deformed_pt[2] - back_projected_point[2]);
        }

        return true;
    }

    static ceres::CostFunction* create(
            double cost_function_weight,
            const Eigen::Vector3d& visible_points,
            const Eigen::Vector3d& visible_points_normal,
            const std::vector<Eigen::Vector3d>& neighbor_nodes,
            const Eigen::VectorXd& w_j,
            float fxy, float cx, float cy, float t,
            const cv::Mat& depthmap,
            const Eigen::Matrix3d& global_R,
            const Eigen::Vector3d& global_t);

private:
    double _cost_func_weight;
    Eigen::Vector3d _visible_point;
    Eigen::Vector3d _visible_point_normal;
    std::vector<Eigen::Vector3d> _neighbor_nodes;
    Eigen::VectorXd _w_j;
    float _fxy;
    float _cx;
    float _cy;
    float _t;

    Eigen::Matrix3d _global_R; // global R
    Eigen::Vector3d _global_t; // global t

    std::unique_ptr<ceres::Grid2D<float, 1>> _grid;
    std::unique_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>> _depth;
};


/**
 * @brief The CostFunctionCorr class
 * Correspondence term
 * Refers to "Fusion4d" followed by "MIS-SLAM"
 */
class CostFunctionCorr : public ceres::CostFunction
{
public:
    CostFunctionCorr(double cost_function_weight,
                     const Eigen::Vector3d& v_prev,
                     const std::vector<Eigen::Vector3d>& neighbor_nodes,
                     const Eigen::VectorXd& w_j,
                     const Eigen::Vector3d& v_curr,
                     const Eigen::Matrix3d& global_R,
                     const Eigen::Vector3d& global_t);

    ~CostFunctionCorr(){}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals, double** jacobians) const;
private:
    double _cost_func_weight;
    Eigen::Vector3d _v_prev;
    std::vector<Eigen::Vector3d> _neighbor_nodes;
    Eigen::VectorXd _w_j;
    Eigen::Vector3d _v_curr;

    Eigen::Matrix3d _global_R; // global R
    Eigen::Vector3d _global_t; // global t
};

#endif // COST_FUNCTION_H_LF
