#ifndef COST_FUNCTION_H_LF
#define COST_FUNCTION_H_LF
#include <ceres/ceres.h>
#include <Eigen/Dense>


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
class CostFunctionData : public ceres::CostFunction
{
public:
    CostFunctionData(double cost_function_weight);

    ~CostFunctionData(){}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals, double** jacobians) const;
private:
    double _cost_func_weight;
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
                     const Eigen::Vector3d& v_curr);

    ~CostFunctionCorr(){}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals, double** jacobians) const;
private:
    double _cost_func_weight;
    Eigen::Vector3d _v_prev;
    std::vector<Eigen::Vector3d> _neighbor_nodes;
    Eigen::VectorXd _w_j;
    Eigen::Vector3d _v_curr;
};

#endif // COST_FUNCTION_H_LF
