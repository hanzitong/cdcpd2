#include "cdcpd/optimizer.h"

// Include OSQP here to avoid macro conflicts with OpenCV
#ifdef USE_OSQP
#include <osqp/osqp.h>
#endif

#include <arc_utilities/enumerate.h>
#include <rclcpp/rclcpp.hpp>

#include <arc_utilities/eigen_ros_conversions.hpp>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point_3;
typedef K::Ray_3 Ray_3;
typedef K::Vector_3 Vector;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> AABB_face_graph_primitive;
typedef CGAL::AABB_traits<K, AABB_face_graph_primitive> AABB_face_graph_traits;

namespace PMP = CGAL::Polygon_mesh_processing;

#ifdef USE_FULL_CGAL_FEATURES
typedef PMP::Face_location<Mesh, FT> Face_location;
#endif

using Eigen::Matrix2Xi;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xf;
using Eigen::Matrix4Xf;
using Eigen::MatrixXf;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXf;

using std::max;
using std::min;
constexpr auto const LOGNAME = "optimizer";

static Eigen::Vector3f const bounding_box_extend;

// Helper functions for CGAL conversions

static Vector3f cgalVec2EigenVec(Vector cgal_v) { return Vector3f(cgal_v[0], cgal_v[1], cgal_v[2]); }

static Vector3f Pt3toVec(const Point_3 pt) { return Vector3f(float(pt.x()), float(pt.y()), float(pt.z())); }

std::tuple<Points, Normals> Optimizer::nearest_points_and_normal(const Matrix3Xf &last_template,
                                                                 Objects const &objects) {
  for (auto const &object : objects) {
    // Meshes
    if (object.meshes.size() != object.mesh_poses.size()) {
      RCLCPP_ERROR(rclcpp::get_logger(LOGNAME),
                   "got %zu meshes but %zu mesh poses, they should match.",
                   object.meshes.size(), object.mesh_poses.size());
    } else {
      for (auto mesh_idx = 0u; mesh_idx < object.meshes.size(); ++mesh_idx) {
        auto const mesh = object.meshes[mesh_idx];
        auto const mesh_pose = object.mesh_poses[mesh_idx];
        // apply the pose transform to all the vertices in the mesh
        shape_msgs::msg::Mesh mesh_transformed;
        for (auto &vertex : mesh_transformed.vertices) {
          auto const transform = ConvertTo<Eigen::Isometry3d>(mesh_pose);
          auto const transformed_vertex = transform * ConvertTo<Eigen::Vector3d>(vertex);
          vertex = ConvertTo<geometry_msgs::msg::Point>(transformed_vertex);
        }
        auto const obstacle_constraints = nearest_points_and_normal_mesh(last_template, mesh_transformed);
      }
    }

    // Planes
    if (object.planes.size() != object.plane_poses.size()) {
      RCLCPP_ERROR(rclcpp::get_logger(LOGNAME),
                   "got %zu planes but %zu plane poses, they should match.",
                   object.planes.size(), object.plane_poses.size());
    } else {
      for (auto plane_idx = 0u; plane_idx < object.planes.size(); ++plane_idx) {
        auto plane = object.planes[plane_idx];
        auto const plane_pose = object.plane_poses[plane_idx];
        shape_msgs::msg::Plane plane_transformed;
        auto const transform = ConvertTo<Eigen::Isometry3d>(plane_pose);
        // NOTE: We ignore the d coefficient here because the gazebo plugin I use to generate these always sets it to 0.
        // the d coefficient is redundant since the plane also has a pose
        Eigen::Vector3d plane_normal(plane.coef[0], plane.coef[1], plane.coef[2]);
        auto const transformed_coef = transform * plane_normal;
        plane.coef[0] = transformed_coef[0];
        plane.coef[1] = transformed_coef[1];
        plane.coef[2] = transformed_coef[2];
        auto const obstacle_constraints = nearest_points_and_normal_plane(last_template, plane_transformed);
      }
    }

    // Primitives
    if (object.primitives.size() != object.primitive_poses.size()) {
      RCLCPP_ERROR(rclcpp::get_logger(LOGNAME),
                   "got %zu primitives but %zu primitive poses, they should match.",
                   object.primitives.size(), object.primitive_poses.size());
    } else {
      for (auto primitive_idx = 0u; primitive_idx < object.primitives.size(); ++primitive_idx) {
        auto const primitive = object.primitives[primitive_idx];
        auto const primitive_pose = object.primitive_poses[primitive_idx];

        switch (primitive.type) {
          case shape_msgs::msg::SolidPrimitive::BOX: {
            auto const obstacle_constraints = nearest_points_and_normal_box(last_template, primitive, primitive_pose);
            break;
          }
          case shape_msgs::msg::SolidPrimitive::CYLINDER: {
            auto const obstacle_constraints =
                nearest_points_and_normal_cylinder(last_template, primitive, primitive_pose);
            break;
          }
          case shape_msgs::msg::SolidPrimitive::SPHERE: {
            auto const obstacle_constraints =
                nearest_points_and_normal_sphere(last_template, primitive, primitive_pose);
            break;
          }
          default:
            RCLCPP_ERROR(rclcpp::get_logger(LOGNAME), "Unsupported shape type %d", static_cast<int>(primitive.type));
            break;
        }
      }
    }
  }

  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  return {};
}

std::tuple<Points, Normals> Optimizer::nearest_points_and_normal_box(const Matrix3Xf &last_template,
                                                                     shape_msgs::msg::SolidPrimitive const &box,
                                                                     geometry_msgs::msg::Pose const &pose) {
  auto const position = ConvertTo<Vector3f>(pose.position);
  auto const orientation = ConvertTo<Eigen::Quaternionf>(pose.orientation).toRotationMatrix();
  auto const box_x = box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
  auto const box_y = box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
  auto const box_z = box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];

  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());

  Matrix4Xf homo_last_template = last_template.colwise().homogeneous();
  Matrix4Xf transform(4, 4);
  transform.block<3, 3>(0, 0) = orientation;
  transform(3, 3) = 1.0;
  transform.block<3, 1>(0, 3) = position;

  Matrix4Xf tf_inv = transform.inverse();

  Vector3f box_x_dir = orientation.col(0);
  Vector3f box_y_dir = orientation.col(1);
  Vector3f box_z_dir = orientation.col(2);

  for (int i = 0; i < last_template.cols(); i++) {
    Vector4f pts_box = tf_inv * homo_last_template.col(i);
    float x = pts_box(0);
    float y = pts_box(1);
    float z = pts_box(2);

    if (x > box_x / 2 || x < -box_x / 2 || y > box_y / 2 || y < -box_y / 2 || z > box_z / 2 || z < -box_z / 2)
    // If the point is not inside the box
    {
      int c_x, c_y, c_z;  // coeffient in front of box_x_dir
      c_x = (x > box_x / 2) ? 1 : ((x < -box_x / 2) ? -1 : 0);
      c_y = (y > box_y / 2) ? 1 : ((y < -box_y / 2) ? -1 : 0);
      c_z = (z > box_z / 2) ? 1 : ((z < -box_z / 2) ? -1 : 0);
      normalVecs.col(i) = c_x * box_x_dir + c_y * box_y_dir + c_z * box_z_dir;
      const Vector3f nearestPt_box_frame(c_x * box_x / 2 + (1 - abs(c_x)) * x, c_y * box_y / 2 + (1 - abs(c_y)) * y,
                                         c_z * box_z / 2 + (1 - abs(c_z)) * z);
      nearestPts.col(i) = (transform * nearestPt_box_frame.homogeneous()).head(3);
    } else {
      float ratio_x = 2 * x / box_x;
      float ratio_y = 2 * y / box_y;
      float ratio_z = 2 * z / box_z;
      Vector3f nearestPt_box_frame;
      if (abs(ratio_x) > abs(ratio_y) && abs(ratio_x) > abs(ratio_z)) {
        float sign_x = ratio_x > 0 ? 1.0 : -1.0;
        normalVecs.col(i) = sign_x * box_x_dir;
        nearestPt_box_frame(0) = sign_x * box_x / 2;
        nearestPt_box_frame(1) = y;
        nearestPt_box_frame(2) = z;
      } else if (abs(ratio_y) > abs(ratio_x) && abs(ratio_y) > abs(ratio_z)) {
        float sign_y = ratio_y > 0 ? 1.0 : -1.0;
        normalVecs.col(i) = sign_y * box_y_dir;
        nearestPt_box_frame(0) = x;
        nearestPt_box_frame(1) = sign_y * box_y / 2;
        nearestPt_box_frame(2) = z;
      } else {
        float sign_z = ratio_z > 0 ? 1.0 : -1.0;
        normalVecs.col(i) = sign_z * box_z_dir;
        nearestPt_box_frame(0) = x;
        nearestPt_box_frame(1) = y;
        nearestPt_box_frame(2) = sign_z * box_z / 2;
      }
      nearestPts.col(i) = (transform * nearestPt_box_frame.homogeneous()).head(3);
    }
  }

  return {nearestPts, normalVecs};
}

std::tuple<Points, Normals> Optimizer::nearest_points_and_normal_sphere(const Matrix3Xf &last_template,
                                                                        shape_msgs::msg::SolidPrimitive const &,
                                                                        geometry_msgs::msg::Pose const &) {
  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  return {nearestPts, normalVecs};
}

std::tuple<Points, Normals> Optimizer::nearest_points_and_normal_plane(const Matrix3Xf &last_template,
                                                                       shape_msgs::msg::Plane const &) {
  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  return {nearestPts, normalVecs};
}

std::tuple<Points, Normals> Optimizer::nearest_points_and_normal_cylinder(const Matrix3Xf &last_template,
                                                                          shape_msgs::msg::SolidPrimitive const &cylinder,
                                                                          geometry_msgs::msg::Pose const &pose) {
  auto const position = ConvertTo<Vector3f>(pose.position);
  // NOTE: Yixuan, should orientation be roll, pitch, yaw here?
  // Answer: As what I can recall, the orientation is the unit vector along center axis
  auto const orientation = ConvertTo<Eigen::Quaternionf>(pose.orientation).toRotationMatrix().eulerAngles(0, 1, 2);
  auto const radius = cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
  auto const height = cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];

  // find of the nearest points and corresponding normal vector on the cylinder
  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  for (int i = 0; i < last_template.cols(); i++) {
    Vector3f pt;
    pt << last_template.col(i);
    Vector3f unitVecH = orientation / orientation.norm();
    Vector3f unitVecR = (pt - position) - ((pt - position).transpose() * unitVecH) * unitVecH;
    unitVecR = unitVecR / unitVecR.norm();
    float h = unitVecH.transpose() * (pt - position);
    float r = unitVecR.transpose() * (pt - position);
    Vector3f nearestPt;
    Vector3f normalVec;
    if (h > height / 2 && r >= radius) {
      nearestPt = unitVecR * radius + unitVecH * height / 2 + position;
      normalVec = unitVecR + unitVecH;
    } else if (h < -height / 2 && r >= radius) {
      nearestPt = unitVecR * radius - unitVecH * height / 2 + position;
      normalVec = unitVecR - orientation / orientation.norm();
    } else if (h > height / 2 && r < radius) {
      nearestPt = r * unitVecR + height / 2 * unitVecH + position;
      normalVec = unitVecH;
    } else if (h < -height / 2 && r < radius) {
      nearestPt = r * unitVecR - height / 2 * unitVecH + position;
      normalVec = -unitVecH;
    } else if (h <= height / 2 && h >= -height / 2 && r < radius) {
      if (height / 2 - h < radius - r) {
        nearestPt = r * unitVecR + height / 2 * unitVecH + position;
        normalVec = unitVecH;
      } else if (h + height / 2 < radius - r) {
        nearestPt = r * unitVecR + height / 2 * unitVecH + position;
        normalVec = -unitVecH;
      } else {
        nearestPt = radius * unitVecR + h * unitVecH + position;
        normalVec = unitVecR;
      }
    } else if (h <= height / 2 && h >= -height / 2 && r >= radius) {
      nearestPt = radius * unitVecR + h * unitVecH + position;
      normalVec = unitVecR;
    }
    normalVec = normalVec / normalVec.norm();
    for (int j = 0; j < 3; ++j) {
      nearestPts(j, i) = nearestPt(j);
      normalVecs(j, i) = normalVec(j);
    }
  }
  return {nearestPts, normalVecs};
}

std::tuple<Points, Normals> Optimizer::nearest_points_and_normal_mesh(const Matrix3Xf &last_template,
                                                                      shape_msgs::msg::Mesh const &shapes_mesh) {
  // This function requires full CGAL Polygon_mesh_processing features
  // For now, return empty results as this is not critical for basic OSQP optimization
  RCLCPP_WARN(rclcpp::get_logger(LOGNAME), 
              "nearest_points_and_normal_mesh: Full CGAL mesh processing not implemented");
  
  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  nearestPts.setZero();
  normalVecs.setZero();
  
  return {nearestPts, normalVecs};
}

std::tuple<MatrixXf, MatrixXf> nearest_points_line_segments(const Matrix3Xf &last_template, const Matrix2Xi &E) {
  // find the nearest points on the line segments
  // refer to the website https://math.stackexchange.com/questions/846054/closest-points-on-two-line-segments
  MatrixXf startPts(
      4, E.cols() * E.cols());  // Matrix: 3 * E^2: startPts.col(E*cols()*i + j) is the nearest point on edge i w.r.t. j
  MatrixXf endPts(
      4, E.cols() * E.cols());  // Matrix: 3 * E^2: endPts.col(E*cols()*i + j) is the nearest point on edge j w.r.t. i
  for (int i = 0; i < E.cols(); ++i) {
    Vector3f P1 = last_template.col(E(0, i));
    Vector3f P2 = last_template.col(E(1, i));

    for (int j = 0; j < E.cols(); ++j) {
      Vector3f P3 = last_template.col(E(0, j));
      Vector3f P4 = last_template.col(E(1, j));

      float R21 = (P2 - P1).squaredNorm();
      float R22 = (P4 - P3).squaredNorm();
      float D4321 = (P4 - P3).dot(P2 - P1);
      float D3121 = (P3 - P1).dot(P2 - P1);
      float D4331 = (P4 - P3).dot(P3 - P1);

      float s;
      float t;

      if (R21 * R22 - D4321 * D4321 != 0) {
        s = min(max((-D4321 * D4331 + D3121 * R22) / (R21 * R22 - D4321 * D4321), 0.0f), 1.0f);
        t = min(max((D4321 * D3121 - D4331 * R21) / (R21 * R22 - D4321 * D4321), 0.0f), 1.0f);
      } else {
        // means P1 P2 P3 P4 are on the same line
        float P13 = (P3 - P1).squaredNorm();
        s = 0;
        t = 0;
        float P14 = (P4 - P1).squaredNorm();
        if (P14 < P13) {
          s = 0;
          t = 1;
        }
        float P23 = (P3 - P2).squaredNorm();
        if (P23 < P14 && P23 < P13) {
          s = 1;
          t = 0;
        }
        float P24 = (P4 - P2).squaredNorm();
        if (P24 < P23 && P24 < P14 && P24 < P13) {
          s = 1;
          t = 1;
        }
      }

      for (int dim = 0; dim < 3; ++dim) {
        startPts(dim, E.cols() * i + j) = (1 - s) * P1(dim) + s * P2(dim);
        endPts(dim, E.cols() * i + j) = (1 - t) * P3(dim) + t * P4(dim);
      }
      startPts(3, E.cols() * i + j) = s;
      endPts(3, E.cols() * i + j) = t;
    }
  }
  return {startPts, endPts};
}

std::tuple<Points, Normals> Optimizer::test_box(const Eigen::Matrix3Xf &last_template,
                                                shape_msgs::msg::SolidPrimitive const &box,
                                                geometry_msgs::msg::Pose const &pose) {
  return nearest_points_and_normal_box(last_template, box, pose);
}

Optimizer::Optimizer(const Eigen::Matrix3Xf initial_template, const Eigen::Matrix3Xf last_template,
                     const float stretch_lambda, const float obstacle_cost_weight)
    : initial_template_(initial_template),
      last_template_(last_template),
      stretch_lambda_(stretch_lambda),
      obstacle_cost_weight_(obstacle_cost_weight) {}

Matrix3Xf Optimizer::operator()(const Matrix3Xf &Y, const Matrix2Xi &E, const std::vector<FixedPoint> &fixed_points,
                                ObstacleConstraints const &obstacle_constraints, const double max_segment_length) {
  // Y: Y^t in Eq. (21)
  // E: E in Eq. (21)
  
#ifdef USE_OSQP
  // OSQP-based optimization
  const ssize_t num_vectors = Y.cols();
  const ssize_t num_vars = 3 * num_vectors;
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(LOGNAME), 
                      "Starting OSQP optimization with " << num_vectors << " points");
  
  // Build QP problem: min 0.5 * x'Px + q'x
  // subject to: l <= Ax <= u
  
  // Initialize sparse P matrix (objective function)
  std::vector<c_float> P_data;
  std::vector<c_int> P_row_indices;
  std::vector<c_int> P_col_ptr;
  
  P_col_ptr.push_back(0);
  
  // Diagonal terms: minimize deviation from CPD result Y
  for (ssize_t i = 0; i < num_vars; i++) {
    P_data.push_back(2.0);  // Quadratic cost
    P_row_indices.push_back(i);
    P_col_ptr.push_back(P_data.size());
  }
  
  // Linear term q: -2*Y (to complete the square for ||x - Y||^2)
  std::vector<c_float> q(num_vars);
  for (ssize_t i = 0; i < num_vectors; i++) {
    q[i * 3 + 0] = -2.0 * Y(0, i);
    q[i * 3 + 1] = -2.0 * Y(1, i);
    q[i * 3 + 2] = -2.0 * Y(2, i);
  }
  
  // Add obstacle avoidance to objective (soft constraint as penalty)
  for (const auto& obs : obstacle_constraints) {
    c_float weight = obstacle_cost_weight_;
    int idx = obs.point_idx;
    
    // Add penalty: weight * ||Y[idx] - obs.point||^2
    for (int d = 0; d < 3; d++) {
      int var_idx = idx * 3 + d;
      // Find or add diagonal element
      bool found = false;
      for (size_t k = P_col_ptr[var_idx]; k < P_col_ptr[var_idx + 1]; k++) {
        if (P_row_indices[k] == var_idx) {
          P_data[k] += 2.0 * weight;
          found = true;
          break;
        }
      }
      if (!found) {
        // This shouldn't happen with diagonal initialization above
        RCLCPP_WARN(rclcpp::get_logger(LOGNAME), "Unexpected: diagonal element not found");
      }
      
      q[var_idx] -= 2.0 * weight * obs.point(d);
    }
  }
  
  // Build constraint matrix A
  std::vector<c_float> A_data;
  std::vector<c_int> A_row_indices;
  std::vector<c_int> A_col_ptr;
  std::vector<c_float> l_bounds;
  std::vector<c_float> u_bounds;
  
  int constraint_count = 0;
  
  // Constraint 1: Fixed points (equality constraints)
  if (!fixed_points.empty() && gripper_constraints_satisfiable(fixed_points)) {
    for (const auto &fp : fixed_points) {
      for (int d = 0; d < 3; d++) {
        int var_idx = fp.template_index * 3 + d;
        
        l_bounds.push_back(fp.position(d));
        u_bounds.push_back(fp.position(d));
        
        constraint_count++;
      }
    }
  }
  
  // Build A matrix in CSC format (column-wise)
  for (ssize_t col = 0; col < num_vars; col++) {
    A_col_ptr.push_back(A_data.size());
    
    // Check if this column has fixed point constraints
    for (size_t fp_idx = 0; fp_idx < fixed_points.size(); fp_idx++) {
      for (int d = 0; d < 3; d++) {
        int var_idx = fixed_points[fp_idx].template_index * 3 + d;
        if (var_idx == col) {
          // This variable has a constraint
          int row = fp_idx * 3 + d;
          A_data.push_back(1.0);
          A_row_indices.push_back(row);
        }
      }
    }
  }
  A_col_ptr.push_back(A_data.size());
  
  // Setup matrices
  csc P_csc;
  P_csc.m = num_vars;
  P_csc.n = num_vars;
  P_csc.nzmax = P_data.size();
  P_csc.nz = -1;  // CSC format
  P_csc.x = P_data.data();
  P_csc.i = P_row_indices.data();
  P_csc.p = P_col_ptr.data();
  
  csc A_csc;
  A_csc.m = constraint_count;
  A_csc.n = num_vars;
  A_csc.nzmax = A_data.empty() ? 0 : A_data.size();
  A_csc.nz = -1;
  A_csc.x = A_data.empty() ? nullptr : A_data.data();
  A_csc.i = A_row_indices.empty() ? nullptr : A_row_indices.data();
  A_csc.p = A_col_ptr.data();
  
  // Setup OSQP
  OSQPSettings settings;
  OSQPWorkspace* workspace = nullptr;
  OSQPData data;
  
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.max_iter = 2000;
  settings.eps_abs = 1e-3;
  settings.eps_rel = 1e-3;
  settings.polish = true;
  
  data.n = num_vars;
  data.m = constraint_count;
  data.P = &P_csc;
  data.q = q.data();
  data.A = constraint_count > 0 ? &A_csc : nullptr;
  data.l = constraint_count > 0 ? l_bounds.data() : nullptr;
  data.u = constraint_count > 0 ? u_bounds.data() : nullptr;
  
  // Solve
  osqp_setup(&workspace, &data, &settings);
  osqp_solve(workspace);
  
  // Extract solution
  Matrix3Xf Y_opt(3, num_vectors);
  if (workspace->info->status_val == OSQP_SOLVED || 
      workspace->info->status_val == OSQP_SOLVED_INACCURATE) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(LOGNAME),
                        "OSQP optimization completed: iter=" << workspace->info->iter << 
                        ", obj=" << workspace->info->obj_val);
    
    for (ssize_t i = 0; i < num_vectors; i++) {
      Y_opt(0, i) = workspace->solution->x[i * 3 + 0];
      Y_opt(1, i) = workspace->solution->x[i * 3 + 1];
      Y_opt(2, i) = workspace->solution->x[i * 3 + 2];
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(LOGNAME),
                 "OSQP optimization failed with status: %d", workspace->info->status_val);
    // Fallback to input
    Y_opt = Y;
  }
  
  // Cleanup
  osqp_cleanup(workspace);
  
  return Y_opt;
  
#else
  // Fallback if OSQP not available
  RCLCPP_WARN(rclcpp::get_logger(LOGNAME), 
              "Optimization disabled - USE_OSQP not defined");
  return Y;
#endif
}

bool Optimizer::gripper_constraints_satisfiable(const std::vector<FixedPoint> &fixed_points) const {
  for (auto const &p1 : fixed_points) {
    for (auto const &p2 : fixed_points) {
      float const current_distance = (p1.position - p2.position).squaredNorm();
      float const original_distance =
          (initial_template_.col(p1.template_index) - initial_template_.col(p2.template_index)).squaredNorm();

      if (current_distance > original_distance * stretch_lambda_ * stretch_lambda_) {
        return false;
      }
    }
  }
  return true;
}
