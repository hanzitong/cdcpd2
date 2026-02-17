#pragma once

#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>

// Include OSQP before OpenCV to avoid macro conflicts
#ifdef USE_OSQP
// OSQP will be included only in optimizer.cpp to avoid macro conflicts
#endif

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/convex_decomposition_3.h>
#include <CGAL/minkowski_sum_3.h>
#include <CGAL/subdivision_method_3.h>

// OSQP is used for optimization (GUROBI support removed)

// Forward declarations
struct FixedPoint {
  Eigen::Vector3f position;
  int template_index;
};

using Matrix3Xf = Eigen::Matrix<float, 3, Eigen::Dynamic>;
using Matrix2Xi = Eigen::Matrix<int, 2, Eigen::Dynamic>;
using Vector3f = Eigen::Vector3f;
using RowVector3f = Eigen::RowVector3f;

using Objects = std::vector<moveit_msgs::msg::CollisionObject>;
using Points = Eigen::Matrix3Xf;
using Normals = Eigen::Matrix3Xf;
using Point = Eigen::Vector3f;
using Normal = Eigen::Vector3f;
using PointNormal = std::tuple<Point, Normal>;  // Hyperplane, for enforcing that tracked points aren't inside obstacles
using Points = Eigen::Matrix3Xf;
using Normals = Eigen::Matrix3Xf;
using Point = Eigen::Vector3f;
using Normal = Eigen::Vector3f;
using PointNormal = std::tuple<Point, Normal>;  // Hyperplane, for enforcing that tracked points aren't inside obstacles
struct ObstacleConstraint {
  unsigned int point_idx;
  Eigen::Vector3f point;
  Eigen::Vector3f normal;
};
using ObstacleConstraints = std::vector<ObstacleConstraint>;

class Optimizer {
 public:
  Optimizer(const Eigen::Matrix3Xf initial_template, const Eigen::Matrix3Xf last_template, float stretch_lambda,
            float obstacle_cost_weight);

  [[nodiscard]] Eigen::Matrix3Xf operator()(const Eigen::Matrix3Xf &Y, const Eigen::Matrix2Xi &E,
                                            const std::vector<FixedPoint> &fixed_points,
                                            ObstacleConstraints const &points_normals, double max_segment_length);

  std::tuple<Points, Normals> test_box(const Eigen::Matrix3Xf &last_template, shape_msgs::msg::SolidPrimitive const &box,
                                       geometry_msgs::msg::Pose const &pose);

 private:
  [[nodiscard]] bool gripper_constraints_satisfiable(const std::vector<FixedPoint> &fixed_points) const;

  [[nodiscard]] std::tuple<Points, Normals> nearest_points_and_normal_box(const Eigen::Matrix3Xf &last_template,
                                                                          shape_msgs::msg::SolidPrimitive const &box,
                                                                          geometry_msgs::msg::Pose const &pose);

  [[nodiscard]] std::tuple<Points, Normals> nearest_points_and_normal_sphere(const Eigen::Matrix3Xf &last_template,
                                                                             shape_msgs::msg::SolidPrimitive const &sphere,
                                                                             geometry_msgs::msg::Pose const &pose);

  [[nodiscard]] std::tuple<Points, Normals> nearest_points_and_normal_plane(const Eigen::Matrix3Xf &last_template,
                                                                            shape_msgs::msg::Plane const &plane);

  [[nodiscard]] std::tuple<Points, Normals> nearest_points_and_normal_cylinder(
      const Eigen::Matrix3Xf &last_template, shape_msgs::msg::SolidPrimitive const &cylinder,
      geometry_msgs::msg::Pose const &pose);

  [[nodiscard]] std::tuple<Points, Normals> nearest_points_and_normal_mesh(const Eigen::Matrix3Xf &last_template,
                                                                           shape_msgs::msg::Mesh const &shapes_mesh);

  [[nodiscard]] std::tuple<Points, Normals> nearest_points_and_normal(const Eigen::Matrix3Xf &last_template,
                                                                      Objects const &objects);

  Eigen::Matrix3Xf initial_template_;
  Eigen::Matrix3Xf last_template_;
  float stretch_lambda_;
  float obstacle_cost_weight_;
};

