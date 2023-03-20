/**
 * [1] http://ceres-solver.org/nnls_modeling.html?highlight=costfunction#_CPPv4N5ceres12CostFunctionE
 * ceres::CostFunction 类
 *   输入参数块的数量和大小被记录在 CostFunction::parameter_block_sizes_
 *   输出残差的个数被记录在 CostFunction::num_residuals_
 * 从这个类继承的用户代码应该使用相应的访问器设置这两个成员，即:
 *
 *   1  std::vector<int>* block_sizes =  mutable_parameter_block_sizes();
 *   2  block_sizes->push_back(第1个输入参数块的参数数量);
 *   3  block_sizes->push_back(第2个输入参数块的参数数量);
 *   4
 *   5  set_num_residuals(输出残差的个数);
 *
 * 当添加到Problem::AddResidualBlock()时，该信息将由Problem验证
 *
 * For the fucntion:
 * bool CostFunction::Evaluate(double const* const* parameters,
 *                             double *residuals, double **jacobians) const
 *    FOR: Compute the residual vector and the Jacobian matrices.
 *
 * When jacobians[i] != nullptr, the user is required to compute the Jacobian of
 * the residual vector with respect to parameters[i] and store it in this array,
 * i.e.
 *
 *                                                    \partial residual[r]
 *  jacobian[i][r*parameter_block_sizes_[i] + c] = ---------------------------
 *                                                  \partial parameters[i][c]
 *
 */
#include "cost_function.h"

/* -------------------------------------------------------------------------- */
/*                           class CostFunctionRot                            */
/* -------------------------------------------------------------------------- */

CostFunctionRot::CostFunctionRot(float cost_function_weight)
    : _cost_func_weight(sqrt(cost_function_weight))
{
    std::vector<int>* block_sizes =  mutable_parameter_block_sizes();
    block_sizes->push_back(12);
    set_num_residuals(9);
}


bool CostFunctionRot::Evaluate(double const* const* parameters,
                               double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Matrix3d> R(parameters[0]);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Map<Eigen::MatrixXd> E(residuals, 3, 3);
    E = _cost_func_weight*(R.transpose()*R - I);

    if (jacobians != nullptr){
        if(jacobians[0] != nullptr){

            // Consider Rot(R)(0,0)=(R'*R - I)(0,0)
            jacobians[0][ 0] = _cost_func_weight*2*R(0,0);	// dE(0,0)/dR(0,0)
            jacobians[0][ 1] = _cost_func_weight*2*R(1,0);	// dE(0,0)/dR(1,0)
            jacobians[0][ 2] = _cost_func_weight*2*R(2,0);	// dE(0,0)/dR(2,0)
            jacobians[0][ 3] = 0;							// dE(0,0)/dR(0,1)
            jacobians[0][ 4] = 0;							// dE(0,0)/dR(1,1)
            jacobians[0][ 5] = 0;							// dE(0,0)/dR(2,1)
            jacobians[0][ 6] = 0;							// dE(0,0)/dR(0,2)
            jacobians[0][ 7] = 0;							// dE(0,0)/dR(1,2)
            jacobians[0][ 8] = 0;							// dE(0,0)/dR(2,2)
            jacobians[0][ 9] = 0;							// dE(0,0)/dt(0)
            jacobians[0][10] = 0;							// dE(0,0)/dt(1)
            jacobians[0][11] = 0;							// dE(0,0)/dt(2)

            // Consider Rot(R)(0,1)=(R'*R - I)(0,1)
            jacobians[0][ 0 + 12] = _cost_func_weight*R(0,1);
            jacobians[0][ 1 + 12] = _cost_func_weight*R(1,1);
            jacobians[0][ 2 + 12] = _cost_func_weight*R(2,1);
            jacobians[0][ 3 + 12] = _cost_func_weight*R(0,0);
            jacobians[0][ 4 + 12] = _cost_func_weight*R(1,0);
            jacobians[0][ 5 + 12] = _cost_func_weight*R(2,0);
            jacobians[0][ 6 + 12] = 0;
            jacobians[0][ 7 + 12] = 0;
            jacobians[0][ 8 + 12] = 0;
            jacobians[0][ 9 + 12] = 0;
            jacobians[0][10 + 12] = 0;
            jacobians[0][11 + 12] = 0;

            // Consider Rot(R)(0,2)=(R'*R - I)(0,2)
            jacobians[0][ 0 + 2*12] = _cost_func_weight*R(0,2);
            jacobians[0][ 1 + 2*12] = _cost_func_weight*R(1,2);
            jacobians[0][ 2 + 2*12] = _cost_func_weight*R(2,2);
            jacobians[0][ 3 + 2*12] = 0;
            jacobians[0][ 4 + 2*12] = 0;
            jacobians[0][ 5 + 2*12] = 0;
            jacobians[0][ 6 + 2*12] = _cost_func_weight*R(0,0);
            jacobians[0][ 7 + 2*12] = _cost_func_weight*R(1,0);
            jacobians[0][ 8 + 2*12] = _cost_func_weight*R(2,0);
            jacobians[0][ 9 + 2*12] = 0;
            jacobians[0][10 + 2*12] = 0;
            jacobians[0][11 + 2*12] = 0;

            // Consider Rot(R)(1,0)=(R'*R - I)(1,0)
            jacobians[0][ 0 + 3*12] = _cost_func_weight*R(0,1);
            jacobians[0][ 1 + 3*12] = _cost_func_weight*R(1,1);
            jacobians[0][ 2 + 3*12] = _cost_func_weight*R(2,1);
            jacobians[0][ 3 + 3*12] = _cost_func_weight*R(0,0);
            jacobians[0][ 4 + 3*12] = _cost_func_weight*R(1,0);
            jacobians[0][ 5 + 3*12] = _cost_func_weight*R(2,0);
            jacobians[0][ 6 + 3*12] = 0;
            jacobians[0][ 7 + 3*12] = 0;
            jacobians[0][ 8 + 3*12] = 0;
            jacobians[0][ 9 + 3*12] = 0;
            jacobians[0][10 + 3*12] = 0;
            jacobians[0][11 + 3*12] = 0;

            // Consider Rot(R)(1,1)=(R'*R - I)(1,1)
            jacobians[0][ 0 + 4*12] = 0;
            jacobians[0][ 1 + 4*12] = 0;
            jacobians[0][ 2 + 4*12] = 0;
            jacobians[0][ 3 + 4*12] = _cost_func_weight*2*R(0,1);
            jacobians[0][ 4 + 4*12] = _cost_func_weight*2*R(1,1);
            jacobians[0][ 5 + 4*12] = _cost_func_weight*2*R(2,1);
            jacobians[0][ 6 + 4*12] = 0;
            jacobians[0][ 7 + 4*12] = 0;
            jacobians[0][ 8 + 4*12] = 0;
            jacobians[0][ 9 + 4*12] = 0;
            jacobians[0][10 + 4*12] = 0;
            jacobians[0][11 + 4*12] = 0;

            // Consider Rot(R)(1,2)=(R'*R - I)(0,2)
            jacobians[0][ 0 + 5*12] = 0;
            jacobians[0][ 1 + 5*12] = 0;
            jacobians[0][ 2 + 5*12] = 0;
            jacobians[0][ 3 + 5*12] = _cost_func_weight*R(0,2);
            jacobians[0][ 4 + 5*12] = _cost_func_weight*R(1,2);
            jacobians[0][ 5 + 5*12] = _cost_func_weight*R(2,2);
            jacobians[0][ 6 + 5*12] = _cost_func_weight*R(0,1);
            jacobians[0][ 7 + 5*12] = _cost_func_weight*R(1,1);
            jacobians[0][ 8 + 5*12] = _cost_func_weight*R(2,1);
            jacobians[0][ 9 + 5*12] = 0;
            jacobians[0][10 + 5*12] = 0;
            jacobians[0][11 + 5*12] = 0;

            // Consider Rot(R)(2,0)=(R'*R - I)(2,0)
            jacobians[0][ 0 + 6*12] = _cost_func_weight*R(0,2);
            jacobians[0][ 1 + 6*12] = _cost_func_weight*R(1,2);
            jacobians[0][ 2 + 6*12] = _cost_func_weight*R(2,2);
            jacobians[0][ 3 + 6*12] = 0;
            jacobians[0][ 4 + 6*12] = 0;
            jacobians[0][ 5 + 6*12] = 0;
            jacobians[0][ 6 + 6*12] = _cost_func_weight*R(0,0);
            jacobians[0][ 7 + 6*12] = _cost_func_weight*R(1,0);
            jacobians[0][ 8 + 6*12] = _cost_func_weight*R(2,0);
            jacobians[0][ 9 + 6*12] = 0;
            jacobians[0][10 + 6*12] = 0;
            jacobians[0][11 + 6*12] = 0;

            // Consider Rot(R)(2,1)=(R'*R - I)(2,1)
            jacobians[0][ 0 + 7*12] = 0;
            jacobians[0][ 1 + 7*12] = 0;
            jacobians[0][ 2 + 7*12] = 0;
            jacobians[0][ 3 + 7*12] = _cost_func_weight*R(0,2);
            jacobians[0][ 4 + 7*12] = _cost_func_weight*R(1,2);
            jacobians[0][ 5 + 7*12] = _cost_func_weight*R(2,2);
            jacobians[0][ 6 + 7*12] = _cost_func_weight*R(0,1);
            jacobians[0][ 7 + 7*12] = _cost_func_weight*R(1,1);
            jacobians[0][ 8 + 7*12] = _cost_func_weight*R(2,1);
            jacobians[0][ 9 + 7*12] = 0;
            jacobians[0][10 + 7*12] = 0;
            jacobians[0][11 + 7*12] = 0;

            // Consider Rot(R)(2,2)=(R'*R - I)(2,2)
            jacobians[0][ 0 + 8*12] = 0;
            jacobians[0][ 1 + 8*12] = 0;
            jacobians[0][ 2 + 8*12] = 0;
            jacobians[0][ 3 + 8*12] = 0;
            jacobians[0][ 4 + 8*12] = 0;
            jacobians[0][ 5 + 8*12] = 0;
            jacobians[0][ 6 + 8*12] = _cost_func_weight*2*R(0,2);
            jacobians[0][ 7 + 8*12] = _cost_func_weight*2*R(1,2);
            jacobians[0][ 8 + 8*12] = _cost_func_weight*2*R(2,2);
            jacobians[0][ 9 + 8*12] = 0;
            jacobians[0][10 + 8*12] = 0;
            jacobians[0][11 + 8*12] = 0;

        }
    }
    return true;
}


/* -------------------------------------------------------------------------- */
/*                           class CostFunctionReg                            */
/* -------------------------------------------------------------------------- */


CostFunctionReg::CostFunctionReg(double cost_function_weight,
        const Eigen::Vector3d &g_j, const Eigen::Vector3d &g_k, double alpha_j_k)
    : _cost_func_weight(sqrt(cost_function_weight) * alpha_j_k)
    , _g_j(g_j)
    , _g_k(g_k)
{
    std::vector<int>* block_sizes = mutable_parameter_block_sizes();
    block_sizes->push_back(12);
    block_sizes->push_back(12);
    set_num_residuals(3);
}


bool CostFunctionReg::Evaluate(double const* const* parameters,
                               double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Matrix3d> R_j(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_j(parameters[0] + 9);
    Eigen::Map<const Eigen::Matrix3d> R_k(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> t_k(parameters[1] + 9);

    Eigen::Map<Eigen::Vector3d> E(residuals, 3);
    E = _cost_func_weight * ( R_j*(_g_k - _g_j) + _g_j + t_j - (_g_k + t_k) );

    Eigen::Vector3d dg = _g_k - _g_j;

    if (jacobians != nullptr){
        if(jacobians[0] != nullptr){

            jacobians[0][ 0] = _cost_func_weight * dg[0];      // dE(0)/dRj(0,0)
            jacobians[0][ 1] = 0;                              // dE(0)/dRj(1,0)
            jacobians[0][ 2] = 0;                              // dE(0)/dRj(2,0)
            jacobians[0][ 3] = _cost_func_weight * dg[1];      // dE(0)/dRj(0,1)
            jacobians[0][ 4] = 0;                              // dE(0)/dRj(1,1)
            jacobians[0][ 5] = 0;                              // dE(0)/dRj(2,1)
            jacobians[0][ 6] = _cost_func_weight * dg[2];      // dE(0)/dRj(0,2)
            jacobians[0][ 7] = 0;                              // dE(0)/dRj(1,2)
            jacobians[0][ 8] = 0;                              // dE(0)/dRj(2,2)
            jacobians[0][ 9] = _cost_func_weight;              // dE(0)/dtj(0)
            jacobians[0][10] = 0;                              // dE(0)/dtj(1)
            jacobians[0][11] = 0;                              // dE(0)/dtj(2)

            jacobians[0][ 0 + 12] = 0;
            jacobians[0][ 1 + 12] = _cost_func_weight * dg[0];
            jacobians[0][ 2 + 12] = 0;
            jacobians[0][ 3 + 12] = 0;
            jacobians[0][ 4 + 12] = _cost_func_weight * dg[1];
            jacobians[0][ 5 + 12] = 0;
            jacobians[0][ 6 + 12] = 0;
            jacobians[0][ 7 + 12] = _cost_func_weight * dg[2];
            jacobians[0][ 8 + 12] = 0;
            jacobians[0][ 9 + 12] = 0;
            jacobians[0][10 + 12] = _cost_func_weight;
            jacobians[0][11 + 12] = 0;

            jacobians[0][ 0 + 12*2] = 0;
            jacobians[0][ 1 + 12*2] = 0;
            jacobians[0][ 2 + 12*2] = _cost_func_weight * dg[0];
            jacobians[0][ 3 + 12*2] = 0;
            jacobians[0][ 4 + 12*2] = 0;
            jacobians[0][ 5 + 12*2] = _cost_func_weight * dg[1];
            jacobians[0][ 6 + 12*2] = 0;
            jacobians[0][ 7 + 12*2] = 0;
            jacobians[0][ 8 + 12*2] = _cost_func_weight * dg[2];
            jacobians[0][ 9 + 12*2] = 0;
            jacobians[0][10 + 12*2] = 0;
            jacobians[0][11 + 12*2] = _cost_func_weight;


            jacobians[1][ 0] = 0;                       // dE(0)/dRk(0,0)
            jacobians[1][ 1] = 0;                       // dE(0)/dRk(1,0)
            jacobians[1][ 2] = 0;                       // dE(0)/dRk(2,0)
            jacobians[1][ 3] = 0;                       // dE(0)/dRk(0,1)
            jacobians[1][ 4] = 0;                       // dE(0)/dRk(1,1)
            jacobians[1][ 5] = 0;                       // dE(0)/dRk(2,1)
            jacobians[1][ 6] = 0;                       // dE(0)/dRk(0,2)
            jacobians[1][ 7] = 0;                       // dE(0)/dRk(1,2)
            jacobians[1][ 8] = 0;                       // dE(0)/dRk(2,2)
            jacobians[1][ 9] = - _cost_func_weight;     // dE(0)/dtk(0)
            jacobians[1][10] = 0;                       // dE(0)/dtk(1)
            jacobians[1][11] = 0;                       // dE(0)/dRk(2)

            jacobians[1][ 0 + 12] = 0;
            jacobians[1][ 1 + 12] = 0;
            jacobians[1][ 2 + 12] = 0;
            jacobians[1][ 3 + 12] = 0;
            jacobians[1][ 4 + 12] = 0;
            jacobians[1][ 5 + 12] = 0;
            jacobians[1][ 6 + 12] = 0;
            jacobians[1][ 7 + 12] = 0;
            jacobians[1][ 8 + 12] = 0;
            jacobians[1][ 9 + 12] = 0;
            jacobians[1][10 + 12] = - _cost_func_weight;
            jacobians[1][11 + 12] = 0;

            jacobians[1][ 0 + 12*2] = 0;
            jacobians[1][ 1 + 12*2] = 0;
            jacobians[1][ 2 + 12*2] = 0;
            jacobians[1][ 3 + 12*2] = 0;
            jacobians[1][ 4 + 12*2] = 0;
            jacobians[1][ 5 + 12*2] = 0;
            jacobians[1][ 6 + 12*2] = 0;
            jacobians[1][ 7 + 12*2] = 0;
            jacobians[1][ 8 + 12*2] = 0;
            jacobians[1][ 9 + 12*2] = 0;
            jacobians[1][10 + 12*2] = 0;
            jacobians[1][11 + 12*2] = - _cost_func_weight;
        }
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/*                           class CostFunctionData                           */
/* -------------------------------------------------------------------------- */

CostFunctionData::CostFunctionData(double cost_function_weight,
        const Eigen::Vector3d& visible_point,
        const Eigen::Vector3d& visible_point_normal,
        const std::vector<Eigen::Vector3d>& neighbor_nodes,
        const Eigen::VectorXd& w_j,
        float fxy, float cx, float cy, float t,
        const cv::Mat& depthmap,
        const Eigen::Matrix3d& global_R, const Eigen::Vector3d& global_t)
    : _cost_func_weight(cost_function_weight)
    , _visible_point(visible_point)
    , _visible_point_normal(visible_point_normal)
    , _neighbor_nodes(neighbor_nodes)
    , _w_j(w_j)
    , _fxy(fxy), _cx(cx), _cy(cy), _t(t)
    , _global_R(global_R), _global_t(global_t)
{
    _grid.reset(new ceres::Grid2D<float, 1>(
                    depthmap.ptr<float>(0), 0, depthmap.rows, 0, depthmap.cols));
    _depth.reset(new ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>(*_grid));
}


ceres::CostFunction* CostFunctionData::create(
        double cost_function_weight,
        const Eigen::Vector3d &visible_points,
        const Eigen::Vector3d& visible_points_normal,
        const std::vector<Eigen::Vector3d> &neighbor_nodes,
        const Eigen::VectorXd &w_j,
        float fxy, float cx, float cy, float t,
        const cv::Mat& depthmap,
        const Eigen::Matrix3d& global_R,
        const Eigen::Vector3d& global_t)
{
    auto cost_func = new ceres::DynamicAutoDiffCostFunction<CostFunctionData>
            (new CostFunctionData(cost_function_weight, visible_points,
                                  visible_points_normal, neighbor_nodes,
                                  w_j, fxy, cx, cy, t, depthmap,
                                  global_R, global_t));
    for (size_t i = 0; i < neighbor_nodes.size(); ++i){
        cost_func->AddParameterBlock(12);
    }
    cost_func->SetNumResiduals(1);

    return cost_func;
}


/* -------------------------------------------------------------------------- */
/*                           class CostFunctionCorr                           */
/* -------------------------------------------------------------------------- */

CostFunctionCorr::CostFunctionCorr(
        double cost_function_weight, const Eigen::Vector3d& v_prev,
        const std::vector<Eigen::Vector3d>& neighbor_nodes,
        const Eigen::VectorXd& w_j,
        const Eigen::Vector3d& v_curr,
        const Eigen::Matrix3d& global_R,
        const Eigen::Vector3d& global_t)
    : _cost_func_weight(cost_function_weight)
    , _v_prev(v_prev)
    , _neighbor_nodes(neighbor_nodes)
    , _w_j(w_j)
    , _v_curr(v_curr)
    , _global_R(global_R), _global_t(global_t)
{
    std::vector<int>* block_sizes =  mutable_parameter_block_sizes();
    for (size_t i = 0; i < neighbor_nodes.size(); ++i){
        block_sizes->push_back(12);
    }
    set_num_residuals(3);
}


bool CostFunctionCorr::Evaluate(double const* const* parameters,
                               double *residuals, double **jacobians) const
{
    Eigen::Map<Eigen::Vector3d> E(residuals, 3);
    Eigen::Vector3d new_vert(0, 0, 0);
    for(size_t i = 0; i < _neighbor_nodes.size(); i++) {
        Eigen::Map<const Eigen::Matrix3d> R_j(parameters[i]);
        Eigen::Map<const Eigen::Vector3d> t_j(parameters[i] + 9);

        new_vert += _w_j[i] * (R_j * (_v_prev - _neighbor_nodes[i]) +
                               _neighbor_nodes[i] + t_j);
    }
    new_vert = _global_R * new_vert + _global_t;

    E = _cost_func_weight * (new_vert - _v_curr);

    if (jacobians != nullptr){
        if(jacobians[0] != nullptr){
            for (int i = 0; i < _w_j.size(); ++i)
            {
                double coeff = _cost_func_weight * _w_j[i];
                auto& node = _neighbor_nodes[i];
                float tx = _v_prev[0] - node[0];
                float ty = _v_prev[1] - node[1];
                float tz = _v_prev[2] - node[2];

                jacobians[i][ 0] = coeff * _global_R(0,0) * tx;
                jacobians[i][ 1] = coeff * _global_R(0,1) * tx;
                jacobians[i][ 2] = coeff * _global_R(0,2) * tx;
                jacobians[i][ 3] = coeff * _global_R(0,0) * ty;
                jacobians[i][ 4] = coeff * _global_R(0,1) * ty;
                jacobians[i][ 5] = coeff * _global_R(0,2) * ty;
                jacobians[i][ 6] = coeff * _global_R(0,0) * tz;
                jacobians[i][ 7] = coeff * _global_R(0,1) * tz;
                jacobians[i][ 8] = coeff * _global_R(0,2) * tz;
                jacobians[i][ 9] = coeff * _global_R(0,0);
                jacobians[i][10] = coeff * _global_R(0,1);
                jacobians[i][11] = coeff * _global_R(0,2);

                jacobians[i][ 0 + 12] = coeff * _global_R(1,0) * tx;
                jacobians[i][ 1 + 12] = coeff * _global_R(1,1) * tx;
                jacobians[i][ 2 + 12] = coeff * _global_R(1,2) * tx;
                jacobians[i][ 3 + 12] = coeff * _global_R(1,0) * ty;
                jacobians[i][ 4 + 12] = coeff * _global_R(1,1) * ty;
                jacobians[i][ 5 + 12] = coeff * _global_R(1,2) * ty;
                jacobians[i][ 6 + 12] = coeff * _global_R(1,0) * tz;
                jacobians[i][ 7 + 12] = coeff * _global_R(1,1) * tz;
                jacobians[i][ 8 + 12] = coeff * _global_R(1,2) * tz;
                jacobians[i][ 9 + 12] = coeff * _global_R(1,0);
                jacobians[i][10 + 12] = coeff * _global_R(1,1);
                jacobians[i][11 + 12] = coeff * _global_R(1,2);

                jacobians[i][ 0 + 12*2] = coeff * _global_R(2,0) * tx;
                jacobians[i][ 1 + 12*2] = coeff * _global_R(2,1) * tx;
                jacobians[i][ 2 + 12*2] = coeff * _global_R(2,2) * tx;
                jacobians[i][ 3 + 12*2] = coeff * _global_R(2,0) * ty;
                jacobians[i][ 4 + 12*2] = coeff * _global_R(2,1) * ty;
                jacobians[i][ 5 + 12*2] = coeff * _global_R(2,2) * ty;
                jacobians[i][ 6 + 12*2] = coeff * _global_R(2,0) * tz;
                jacobians[i][ 7 + 12*2] = coeff * _global_R(2,1) * tz;
                jacobians[i][ 8 + 12*2] = coeff * _global_R(2,2) * tz;
                jacobians[i][ 9 + 12*2] = coeff * _global_R(2,0);
                jacobians[i][10 + 12*2] = coeff * _global_R(2,1);
                jacobians[i][11 + 12*2] = coeff * _global_R(2,2);
            }
        }
    }

    return true;
}
