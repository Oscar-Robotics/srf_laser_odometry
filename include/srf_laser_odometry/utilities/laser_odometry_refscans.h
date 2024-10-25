/*********************************************************************
 *
 * SPDX-License-Identifier: LGPL-3.0
 *
 *  Authors: Mariano Jaimez Tarifa and Javier Monroy
 *           MAPIR group, University of Malaga, Spain
 *           http://mapir.uma.es
 *
 *  Date: January 2016
 *
 * This pkgs offers a fast and reliable estimation of 2D odometry based on planar laser scans.
 * SRF is a fast and precise method to estimate the planar motion of a lidar from consecutive range scans.
 * SRF presents a dense method for estimating planar motion with a laser scanner. Starting from a symmetric
 * representation of geometric consistency between scans, we derive a precise range flow constraint and
 * express the motion of the scan observations as a function of the rigid motion of the scanner.
 * In contrast to existing techniques, which align the incoming scan with either the previous one or the last
 * selected keyscan, we propose a combined and efficient formulation to jointly align all these three scans at
 * every iteration. This new formulation preserves the advantages of keyscan-based strategies but is more robust
 * against suboptimal selection of keyscans and the presence of moving objects.
 *
 *  More Info: http://mapir.isa.uma.es/work/SRF-Odometry
 *********************************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <vector>

namespace srf
{

using Pose2d = Eigen::Isometry2f;
using Pose3d = Eigen::Isometry3d;

class SRF_RefS
{
  public:
    template <typename T> using vector_t = std::vector<T, Eigen::aligned_allocator<T>>;

    // Scans and cartesian coordinates: 1 - New, 2 - Old, 3 - Ref
    Eigen::ArrayXf range_wf;
    vector_t<Eigen::ArrayXf> range_1, range_2, range_3;
    vector_t<Eigen::ArrayXf> range_12, range_13, range_warped;
    vector_t<Eigen::ArrayXf> xx_1, xx_2, xx_3, xx_12, xx_13, xx_warped;
    vector_t<Eigen::ArrayXf> yy_1, yy_2, yy_3, yy_12, yy_13, yy_warped;
    vector_t<Eigen::ArrayXf> range_3_warpedTo2, xx_3_warpedTo2, yy_3_warpedTo2;

    // Rigid transformations and velocities (twists: vx, vy, w)
    vector_t<Pose2d> transformations;   // T13
    Eigen::Matrix3f overall_trans_prev; // T23
    Eigen::Vector3f kai_abs, kai_loc;
    Eigen::Vector3f kai_loc_old, kai_loc_level;

    // Solver
    Eigen::MatrixXf A, Aw;
    Eigen::VectorXf B, Bw;
    Eigen::Matrix3f cov_odo;

    // Aux variables
    Eigen::ArrayXf dtita_12, dtita_13;
    Eigen::ArrayXf dt_12, dt_13;
    Eigen::ArrayXf weights_12, weights_13;
    Eigen::Array<bool, Eigen::Dynamic, 1> null_12, null_13;
    Eigen::Array<bool, Eigen::Dynamic, 1> outliers;

    float fovh;
    unsigned int cols, cols_i;
    unsigned int width;
    unsigned int ctf_levels;
    unsigned int image_level, level;
    unsigned int num_valid_range;
    unsigned int iter_irls;
    float g_mask[5];
    bool no_ref_scan;
    bool new_ref_scan;

    // Laser poses (most recent and previous)
    Pose2d laser_pose;
    Pose2d laser_oldpose;
    Pose2d last_increment;
    bool test;
    unsigned int method; // 0 - consecutive scan alignment, 1 - keyscan alignment, 2 - multi-scan (hybrid) alignment

    // Methods
    void initialize(unsigned int size, float FOV_rad, unsigned int odo_method);
    void create_scan_pyramid();
    void calculate_coord();
    void perform_warping();
    void perform_best_warping();
    void warp_scan_3_to_2();
    void calculate_range_derivatives();
    void compute_weights();
    void solve_system_quad_residuals_3_scans();
    void solve_system_smooth_trunc_quad_3_scans();
    void solve_system_smooth_trunc_quad_only_1_3();
    void solve_system_smooth_trunc_quad_only_1_2();
    // void solveSystemMCauchy();
    // void solveSystemMTukey();
    // void solveSystemTruncatedQuad();

    void filter_level_solution();
    void pose_update();
    void update_reference_scan();
    void odometry_calculation();

    inline const Pose2d &get_increment() const
    {
        return last_increment;
    }
    const Eigen::Matrix3f &get_increment_covariance() const
    {
        return cov_odo;
    }
};

} // namespace srf
