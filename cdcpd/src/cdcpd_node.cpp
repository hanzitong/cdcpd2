#include <arc_utilities/enumerate.h>
#include <cdcpd/cdcpd.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <jsk_recognition_msgs/msg/bounding_box.hpp>  // Temporarily disabled - package not installed
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/eigen_ros_conversions.hpp>

#include "cdcpd_ros/kinect_sub.h"

constexpr auto const LOGNAME = "cdcpd_node";
constexpr auto const PERF_LOGGER = "perf";

Eigen::Vector3f extent_to_env_size(Eigen::Vector3f const& bbox_lower, Eigen::Vector3f const& bbox_upper) {
  return (bbox_upper - bbox_lower).cwiseAbs() + 2 * bounding_box_extend;
};

Eigen::Vector3f extent_to_center(Eigen::Vector3f const& bbox_lower, Eigen::Vector3f const& bbox_upper) {
  return (bbox_upper + bbox_lower) / 2;
};

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
namespace gm = geometry_msgs::msg;
namespace vm = visualization_msgs::msg;
namespace ehc = EigenHelpersConversions;

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int num_points, float length);

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int num_points, const Eigen::Vector3f& start_position,
                                                               const Eigen::Vector3f& end_position);

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points, float const length) {
  Eigen::Vector3f start_position(-length / 2, 0, 1.0);
  Eigen::Vector3f end_position(length / 2, 0, 1.0);
  return makeRopeTemplate(num_points, start_position, end_position);
}

std::pair<Eigen::Matrix3Xf, Eigen::Matrix2Xi> makeRopeTemplate(int const num_points,
                                                               const Eigen::Vector3f& start_position,
                                                               const Eigen::Vector3f& end_position) {
  Eigen::Matrix3Xf template_vertices(3, num_points);  // Y^0 in the paper
  Eigen::VectorXf thetas = Eigen::VectorXf::LinSpaced(num_points, 0, 1);
  for (auto i = 0u; i < num_points; ++i) {
    auto const theta = thetas.row(i);
    template_vertices.col(i) = (end_position - start_position) * theta + start_position;
  }
  Eigen::Matrix2Xi template_edges(2, num_points - 1);
  template_edges(0, 0) = 0;
  template_edges(1, template_edges.cols() - 1) = num_points - 1;
  for (int i = 1; i <= template_edges.cols() - 1; ++i) {
    template_edges(0, i) = i;
    template_edges(1, i - 1) = i;
  }
  return {template_vertices, template_edges};
}

PointCloud::Ptr makeCloud(Eigen::Matrix3Xf const& points) {
  // HSV color mapping for mask generation
  PointCloud::Ptr cloud(new PointCloud);
  for (int i = 0; i < points.cols(); ++i) {
    auto const& c = points.col(i);
    cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
  }
  return cloud;
}

cv::Mat getHsvMask(rclcpp::Node::SharedPtr node, cv::Mat const& rgb) {
  auto const hue_min = node->declare_parameter("hue_min", 340.0);
  auto const sat_min = node->declare_parameter("saturation_min", 0.4);
  auto const val_min = node->declare_parameter("value_min", 0.4);
  auto const hue_max = node->declare_parameter("hue_max", 20.0);
  auto const sat_max = node->declare_parameter("saturation_max", 1.0);
  auto const val_max = node->declare_parameter("value_max", 1.0);

  cv::Mat rgb_f;
  rgb.convertTo(rgb_f, CV_32FC3);
  rgb_f /= 255.0;  // get RGB 0.0-1.0
  cv::Mat color_hsv;
  cvtColor(rgb_f, color_hsv, cv::COLOR_RGB2HSV);

  cv::Mat mask1;
  cv::Mat mask2;
  cv::Mat hsv_mask;
  auto hue_min1 = hue_min;
  auto hue_max2 = hue_max;
  if (hue_min > hue_max) {
    hue_max2 = 360;
    hue_min1 = 0;
  }
  cv::inRange(color_hsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max2, sat_max, val_max), mask1);
  cv::inRange(color_hsv, cv::Scalar(hue_min1, sat_min, val_min), cv::Scalar(hue_max, sat_max, val_max), mask2);
  bitwise_or(mask1, mask2, hsv_mask);

  return hsv_mask;
}

void print_bodies(moveit::core::RobotState const& state) {
  std::vector<moveit::core::AttachedBody const*> bs;
  std::cout << "Attached Bodies:\n";
  state.getAttachedBodies(bs);
  for (auto const& b : bs) {
    std::cout << b->getName() << '\n';
  }
}

class CDCPD_Moveit_Node : public rclcpp::Node {
public:
  std::string collision_body_prefix{"cdcpd_tracked_point_"};
  std::string robot_namespace_;
  std::string robot_description_param_;  // Renamed for clarity - this is a parameter name
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr masked_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr template_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_template_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher;
  rclcpp::Publisher<vm::Marker>::SharedPtr order_pub;
  rclcpp::Publisher<vm::MarkerArray>::SharedPtr contact_marker_pub;
  // rclcpp::Publisher<jsk_recognition_msgs::msg::BoundingBox>::SharedPtr bbox_pub;  // Disabled - jsk not installed
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  moveit::core::RobotModelConstPtr model_;  // Changed to ConstPtr for compatibility
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  std::string moveit_frame{"robot_root"};
  std::string kinect_tf_name = "kinect2_rgb_optical_frame";
  double min_distance_threshold{0.01};
  bool moveit_ready{false};
  bool moveit_enabled{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  explicit CDCPD_Moveit_Node(std::string const &robot_namespace)
      : Node("cdcpd_node"),
        robot_namespace_(robot_namespace),
        robot_description_param_("robot_description") {  // Just the parameter name, no namespace prefix
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Note: Cannot call shared_from_this() in constructor
    // Initialization that requires shared_from_this() will be done in init()
  }

  void init() {
    // Initialize scene monitor (requires shared_from_this)
    // Use robot_description parameter name without namespace
    scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        shared_from_this(), robot_description_param_);
    
    auto const scene_topic = robot_namespace_ + "/move_group/monitored_planning_scene";
    auto const service_name = robot_namespace_ + "/get_planning_scene";
    scene_monitor_->startSceneMonitor(scene_topic);
    moveit_ready = scene_monitor_->requestPlanningSceneState(service_name);
    if (not moveit_ready) {
      RCLCPP_WARN(this->get_logger(), "Could not get the moveit planning scene. This means no obstacle constraints.");
    }

    model_ = scene_monitor_->getRobotModel();

    // Publishers for the data, some visualizations, others consumed by other nodes
    original_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cdcpd/original", 10);
    masked_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cdcpd/masked", 10);
    downsampled_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cdcpd/downsampled", 10);
    template_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cdcpd/template", 10);
    pre_template_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cdcpd/pre_template", 10);
    output_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cdcpd/output", 10);
    order_pub = this->create_publisher<vm::Marker>("cdcpd/order", 10);
    contact_marker_pub = this->create_publisher<vm::MarkerArray>("contacts", 10);
    // bbox_pub = this->create_publisher<jsk_recognition_msgs::msg::BoundingBox>("cdcpd/bbox", 10);

    // Moveit Visualization (requires shared_from_this)
    auto const viz_robot_state_topic = "cdcpd_moveit_node/robot_state";
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), "robot_root", viz_robot_state_topic, scene_monitor_);
    visual_tools_->loadRobotStatePub(viz_robot_state_topic, false);

    auto const kinect_name = this->declare_parameter("kinect_name", "kinect2");

    // For use with TF and "fixed points" for the constrain step
    kinect_tf_name = kinect_name + "_rgb_optical_frame";
    auto const left_tf_name = this->declare_parameter("left_tf_name", "");
    auto const right_tf_name = this->declare_parameter("right_tf_name", "");
    auto const num_points = this->declare_parameter("rope_num_points", 11);
    auto const left_node_idx = this->declare_parameter("left_node_idx", num_points - 1);
    auto const right_node_idx = this->declare_parameter("right_node_idx", 1);
    Eigen::MatrixXi gripper_idx(1, 2);
    gripper_idx << left_node_idx, right_node_idx;

    // Initial connectivity model of rope
    auto const rope_length = this->declare_parameter<float>("rope_length", 1.0);
    auto const max_segment_length = rope_length / static_cast<float>(num_points);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "max segment length " << max_segment_length);
    
    // Variables to be initialized
    pcl::PointCloud<pcl::PointXYZ>::Ptr tracked_points;
    Eigen::Matrix2Xi template_edges;
    
    // Only wait for TF if gripper frame names are provided
    if (!left_tf_name.empty() && !right_tf_name.empty()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for TF frames: %s and %s...", 
                  left_tf_name.c_str(), right_tf_name.c_str());
      
      while (rclcpp::ok()) {
        try {
          if (tf_buffer_->canTransform(kinect_tf_name, left_tf_name, tf2::TimePointZero) and
              tf_buffer_->canTransform(kinect_tf_name, right_tf_name, tf2::TimePointZero)) {
            break;
          }
        } catch (tf2::TransformException const& ex) {
          RCLCPP_WARN(this->get_logger(), "Waiting for transform: %s", ex.what());
          rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
      }
      
      auto const left_gripper = tf_buffer_->lookupTransform(kinect_tf_name, left_tf_name, tf2::TimePointZero);
      auto const right_gripper = tf_buffer_->lookupTransform(kinect_tf_name, right_tf_name, tf2::TimePointZero);

      Eigen::Vector3f const start_position =
        ehc::GeometryVector3ToEigenVector3d(left_gripper.transform.translation).cast<float>();
      Eigen::Vector3f const end_position =
          ehc::GeometryVector3ToEigenVector3d(right_gripper.transform.translation).cast<float>();
      auto const [template_vertices, template_edges] = makeRopeTemplate(num_points, start_position, end_position);
      tracked_points = makeCloud(template_vertices);
      RCLCPP_INFO(this->get_logger(), "Template initialized from TF frames");
    } else {
      RCLCPP_WARN(this->get_logger(), "No gripper TF frames specified. Using default template.");
      // Use default positions if no TF frames are specified
      Eigen::Vector3f const start_position(-rope_length / 2.0f, 0.0f, 0.0f);
      Eigen::Vector3f const end_position(rope_length / 2.0f, 0.0f, 0.0f);
      auto const [template_vertices, template_edges] = makeRopeTemplate(num_points, start_position, end_position);
      tracked_points = makeCloud(template_vertices);
    }

    // Construct the initial template as a PCL cloud
    // auto tracked_points = makeCloud(template_vertices);  // Moved into the if/else above

    // CDCPD parameters
    auto const alpha = this->declare_parameter("alpha", 0.5);
    auto const lambda = this->declare_parameter("lambda", 1.0);
    auto const k_spring = this->declare_parameter("k", 100.0);
    auto const beta = this->declare_parameter("beta", 1.0);
    auto const zeta = this->declare_parameter("zeta", 10.0);
    min_distance_threshold = this->declare_parameter("min_distance_threshold", 0.01);
    auto const obstacle_cost_weight = this->declare_parameter("obstacle_cost_weight_", 0.001);
    auto const use_recovery = this->declare_parameter("use_recovery", false);
    auto const kinect_channel = this->declare_parameter("kinect_channel", "qhd");
    
    auto node_ptr = shared_from_this();
    auto cdcpd = CDCPD(node_ptr, tracked_points, template_edges, use_recovery, alpha, beta, lambda, k_spring, zeta,
                       obstacle_cost_weight);

    auto const callback = [this, &cdcpd, &tracked_points, left_tf_name, right_tf_name, max_segment_length, gripper_idx, node_ptr]
                          (cv::Mat const& rgb, cv::Mat const& depth, cv::Matx33d const& intrinsics) {
      auto const t0 = this->now();
      smmap::AllGrippersSinglePose q_config;
      // Left Gripper
      if (not left_tf_name.empty()) {
        try {
          auto const gripper = tf_buffer_->lookupTransform(kinect_tf_name, left_tf_name, tf2::TimePointZero);
          auto const config = ehc::GeometryTransformToEigenIsometry3d(gripper.transform);
          RCLCPP_DEBUG_STREAM(this->get_logger(), "left gripper: " << config.translation());
          // q_config.push_back(config);  // Disabled - need to convert to GripperPose
          smmap::GripperPose gp;
          gp.position = config.translation();
          gp.orientation = Eigen::Quaterniond(config.rotation());
          q_config.push_back(gp);

        } catch (tf2::TransformException const& ex) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
              "Unable to lookup transform from %s to %s: %s", kinect_tf_name.c_str(), left_tf_name.c_str(), ex.what());
        }
      }
      // Right Gripper
      if (not right_tf_name.empty()) {
        try {
          auto const gripper = tf_buffer_->lookupTransform(kinect_tf_name, right_tf_name, tf2::TimePointZero);
          auto const config = ehc::GeometryTransformToEigenIsometry3d(gripper.transform);
          RCLCPP_DEBUG_STREAM(this->get_logger(), "right gripper: " << config.translation());
          // q_config.push_back(config);  // Disabled - need to convert to GripperPose
          smmap::GripperPose gp;
          gp.position = config.translation();
          gp.orientation = Eigen::Quaterniond(config.rotation());
          q_config.push_back(gp);

        } catch (tf2::TransformException const& ex) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
              "Unable to lookup transform from %s to %s: %s", kinect_tf_name.c_str(), right_tf_name.c_str(), ex.what());
        }
      }

      // Perform and record the update
      auto const hsv_mask = getHsvMask(node_ptr, rgb);
      auto const n_grippers = q_config.size();
      // const smmap::AllGrippersSinglePoseDelta q_dot{n_grippers, kinematics::Vector6d::Zero()};  // Disabled
      smmap::AllGrippersSinglePoseDelta q_dot;  // Empty for now

      // publish bbox
      {
        // jsk_recognition_msgs::msg::BoundingBox bbox_msg;
        // bbox_msg.header.stamp = this->now();
        // bbox_msg.header.frame_id = kinect_tf_name;

        auto const bbox_size = extent_to_env_size(cdcpd.last_lower_bounding_box, cdcpd.last_upper_bounding_box);
        auto const bbox_center = extent_to_center(cdcpd.last_lower_bounding_box, cdcpd.last_upper_bounding_box);
        // bbox_msg.pose.position.x = bbox_center.x();
        // bbox_msg.pose.position.y = bbox_center.y();
        // bbox_msg.pose.position.z = bbox_center.z();
        // bbox_msg.pose.orientation.w = 1;
        // bbox_msg.dimensions.x = bbox_size.x();
        // bbox_msg.dimensions.y = bbox_size.y();
        // bbox_msg.dimensions.z = bbox_size.z();
        // bbox_pub->publish(bbox_msg);
      }

      // publish the template before processing
      {
        auto time = this->now();
        sensor_msgs::msg::PointCloud2 pcl_msg;
        pcl::toROSMsg(*tracked_points, pcl_msg);
        pcl_msg.header.frame_id = kinect_tf_name;
        pcl_msg.header.stamp = time;
        pre_template_publisher->publish(pcl_msg);
      }

      ObstacleConstraints obstacle_constraints;
      if (moveit_ready and moveit_enabled) {
        obstacle_constraints = get_moveit_obstacle_constriants(tracked_points);
      }

      auto const out = cdcpd(rgb, depth, hsv_mask, intrinsics, tracked_points, obstacle_constraints, max_segment_length,
                             q_dot, q_config, gripper_idx);
      tracked_points = out.optimized_output;

      // Update the frame ids
      {
        out.original_cloud->header.frame_id = kinect_tf_name;
        out.masked_point_cloud->header.frame_id = kinect_tf_name;
        out.downsampled_cloud->header.frame_id = kinect_tf_name;
        out.cpd_output->header.frame_id = kinect_tf_name;
        out.optimized_output->header.frame_id = kinect_tf_name;
      }

      // Add timestamp information
      {
        auto time = this->now();
        auto pcl_time = pcl_conversions::toPCL(time);
        out.original_cloud->header.stamp = pcl_time;
        out.masked_point_cloud->header.stamp = pcl_time;
        out.downsampled_cloud->header.stamp = pcl_time;
        out.cpd_output->header.stamp = pcl_time;
        out.optimized_output->header.stamp = pcl_time;
      }

      // Publish the point clouds
      {
        sensor_msgs::msg::PointCloud2 msg;
        
        pcl::toROSMsg(*out.original_cloud, msg);
        msg.header.frame_id = kinect_tf_name;
        original_publisher->publish(msg);
        
        pcl::toROSMsg(*out.masked_point_cloud, msg);
        msg.header.frame_id = kinect_tf_name;
        masked_publisher->publish(msg);
        
        pcl::toROSMsg(*out.downsampled_cloud, msg);
        msg.header.frame_id = kinect_tf_name;
        downsampled_publisher->publish(msg);
        
        pcl::toROSMsg(*out.cpd_output, msg);
        msg.header.frame_id = kinect_tf_name;
        template_publisher->publish(msg);
        
        pcl::toROSMsg(*out.optimized_output, msg);
        msg.header.frame_id = kinect_tf_name;
        output_publisher->publish(msg);
      }

      // Publish markers indication the order of the points
      {
        auto rope_marker_fn = [this](PointCloud::ConstPtr cloud, std::string const& ns) {
          vm::Marker order;
          order.header.frame_id = kinect_tf_name;
          order.header.stamp = this->now();
          order.ns = ns;
          order.type = vm::Marker::LINE_STRIP;
          order.action = vm::Marker::ADD;
          order.pose.orientation.w = 1.0;
          order.id = 1;
          order.scale.x = 0.01;
          order.color.r = 1.0;
          order.color.a = 1.0;

          for (auto pc_iter : *cloud) {
            gm::Point p;
            p.x = pc_iter.x;
            p.y = pc_iter.y;
            p.z = pc_iter.z;
            order.points.push_back(p);
          }
          return order;
        };

        auto const rope_marker = rope_marker_fn(out.optimized_output, "line_order");
        order_pub->publish(rope_marker);
      }

      auto const t1 = this->now();
      auto const dt = (t1 - t0).seconds();
      RCLCPP_DEBUG_STREAM(this->get_logger(), "dt = " << dt << "s");
    };

    auto const options = KinectSub::SubscriptionOptions(kinect_name + "/" + kinect_channel);
    // wait a second so the TF buffer can fill
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    KinectSub sub(callback, options);

    RCLCPP_INFO(this->get_logger(), "Spinning...");
    rclcpp::spin(shared_from_this());
  }

  ObstacleConstraints find_nearest_points_and_normals(planning_scene_monitor::LockedPlanningSceneRW planning_scene,
                                                      Eigen::Isometry3d const& cdcpd_to_moveit) {
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.distance = true;
    req.max_contacts_per_pair = 1;
    collision_detection::CollisionResult res;
    planning_scene->checkCollisionUnpadded(req, res);

    vm::MarkerArray contact_markers;
    ObstacleConstraints obstacle_constraints;
    auto contact_idx = 0u;
    for (auto const& [contact_names, contacts] : res.contacts) {
      if (contacts.empty()) {
        continue;
      }

      auto const contact = contacts[0];
      auto add_interaction_constraint = [&](int contact_idx, int body_idx, std::string body_name,
                                            Eigen::Vector3d const& tracked_point_moveit_frame,
                                            Eigen::Vector3d const& object_point_moveit_frame) {
        auto const normal_dir = contact.depth > 0.0 ? 1.0 : -1.0;
        Eigen::Vector3d const object_point_cdcpd_frame = cdcpd_to_moveit.inverse() * object_point_moveit_frame;
        Eigen::Vector3d const tracked_point_cdcpd_frame = cdcpd_to_moveit.inverse() * tracked_point_moveit_frame;
        Eigen::Vector3d const normal_cdcpd_frame =
            ((tracked_point_cdcpd_frame - object_point_cdcpd_frame) * normal_dir).normalized();
        auto get_point_idx = [&]() {
          unsigned int point_idx;
          sscanf(body_name.c_str(), (collision_body_prefix + "%u").c_str(), &point_idx);
          return point_idx;
        };
        auto const point_idx = get_point_idx();
        obstacle_constraints.emplace_back(
            ObstacleConstraint{point_idx, object_point_cdcpd_frame.cast<float>(), normal_cdcpd_frame.cast<float>()});

        // debug & visualize
        {
          RCLCPP_DEBUG_STREAM(
              this->get_logger(), "nearest point: " << contact.nearest_points[0].x() << ", " << contact.nearest_points[0].y()
                                         << ", " << contact.nearest_points[0].z() << " on " << contact.body_name_1
                                         << " and " << contact.nearest_points[1].x() << ", "
                                         << contact.nearest_points[1].y() << ", " << contact.nearest_points[1].z()
                                         << " on " << contact.body_name_2 << " depth " << contact.depth
                                         << " (in moveit frame)");

          vm::Marker arrow;
          arrow.id = 100 * contact_idx + 0;
          arrow.action = vm::Marker::ADD;
          arrow.type = vm::Marker::ARROW;
          arrow.ns = "arrow";
          arrow.header.frame_id = moveit_frame;
          arrow.header.stamp = this->now();
          arrow.color.r = 1.0;
          arrow.color.g = 0.0;
          arrow.color.b = 1.0;
          arrow.color.a = 0.2;
          arrow.scale.x = 0.001;
          arrow.scale.y = 0.002;
          arrow.scale.z = 0.002;
          arrow.pose.orientation.w = 1;
          arrow.points.push_back(ConvertTo<gm::Point>(object_point_moveit_frame));
          arrow.points.push_back(ConvertTo<gm::Point>(tracked_point_moveit_frame));

          vm::Marker normal;
          normal.id = 100 * contact_idx + 0;
          normal.action = vm::Marker::ADD;
          normal.type = vm::Marker::ARROW;
          normal.ns = "normal";
          normal.header.frame_id = kinect_tf_name;
          normal.header.stamp = this->now();
          normal.color.r = 0.4;
          normal.color.g = 1.0;
          normal.color.b = 0.7;
          normal.color.a = 0.6;
          normal.scale.x = 0.0015;
          normal.scale.y = 0.0025;
          normal.scale.z = 0.0025;
          normal.pose.orientation.w = 1;
          normal.points.push_back(ConvertTo<gm::Point>(object_point_cdcpd_frame));
          Eigen::Vector3d const normal_end_point_cdcpd_frame = object_point_cdcpd_frame + normal_cdcpd_frame * 0.02;
          normal.points.push_back(ConvertTo<gm::Point>(normal_end_point_cdcpd_frame));

          contact_markers.markers.push_back(arrow);
          contact_markers.markers.push_back(normal);
        }
      };

      if (contact.depth > min_distance_threshold) {
        continue;
      }
      if (contact.body_name_1.find(collision_body_prefix) != std::string::npos) {
        add_interaction_constraint(contact_idx, 0, contact.body_name_1, contact.nearest_points[0],
                                   contact.nearest_points[1]);
      } else if (contact.body_name_2.find(collision_body_prefix) != std::string::npos) {
        add_interaction_constraint(contact_idx, 1, contact.body_name_2, contact.nearest_points[1],
                                   contact.nearest_points[0]);
      } else {
        continue;
      }

      ++contact_idx;
    }

    vm::MarkerArray clear_array;
    vm::Marker clear_marker;
    clear_marker.action = vm::Marker::DELETEALL;
    clear_array.markers.push_back(clear_marker);
    contact_marker_pub->publish(clear_array);
    contact_marker_pub->publish(contact_markers);
    return obstacle_constraints;
  }

  ObstacleConstraints get_moveit_obstacle_constriants(PointCloud::ConstPtr tracked_points) {
    Eigen::Isometry3d cdcpd_to_moveit;
    try {
      auto const cdcpd_to_moveit_msg = tf_buffer_->lookupTransform(moveit_frame, kinect_tf_name, tf2::TimePointZero);
      cdcpd_to_moveit = ehc::GeometryTransformToEigenIsometry3d(cdcpd_to_moveit_msg.transform);
    } catch (tf2::TransformException const& ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 10000,
          "Unable to lookup transform from %s to %s: %s", kinect_tf_name.c_str(), moveit_frame.c_str(), ex.what());
      return {};
    }

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(scene_monitor_);

    // customize by excluding some objects
    auto& world = planning_scene->getWorldNonConst();
    std::vector<std::string> objects_to_ignore{
        "collision_sphere.link_1",
        "ground_plane.link",
    };
    for (auto const& object_to_ignore : objects_to_ignore) {
      if (world->hasObject(object_to_ignore)) {
        auto success = world->removeObject(object_to_ignore);
        if (success) {
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Successfully removed " << object_to_ignore);
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to remove " << object_to_ignore);
        }
      }
    }

    auto& robot_state = planning_scene->getCurrentStateNonConst();

    // remove the attached "tool boxes"
    std::vector<std::string> objects_to_detach{"left_tool_box", "right_tool_box"};
    for (auto const& object_to_detach : objects_to_detach) {
      if (not robot_state.hasAttachedBody(object_to_detach)) {
        continue;
      }
      auto success = robot_state.clearAttachedBody(object_to_detach);
      if (not success) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to detach " << object_to_detach);
      }
    }

    // Note: Bullet collision detector may not be available in ROS2 Humble MoveIt
    // planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    // attach to the robot base link, sort of hacky but MoveIt only has API for checking robot vs self/world,
    // so we have to make the tracked points part of the robot, hence "attached collision objects"
    for (auto const& [tracked_point_idx, point] : enumerate(*tracked_points)) {
      Eigen::Vector3d const tracked_point_cdcpd_frame = point.getVector3fMap().cast<double>();
      Eigen::Vector3d const tracked_point_moveit_frame = cdcpd_to_moveit * tracked_point_cdcpd_frame;
      Eigen::Isometry3d tracked_point_pose_moveit_frame = Eigen::Isometry3d::Identity();
      tracked_point_pose_moveit_frame.translation() = tracked_point_moveit_frame;

      std::stringstream collision_body_name_stream;
      collision_body_name_stream << collision_body_prefix << tracked_point_idx;
      auto const collision_body_name = collision_body_name_stream.str();

      auto sphere = std::make_shared<shapes::Box>(0.01, 0.01, 0.01);

      robot_state.attachBody(collision_body_name, Eigen::Isometry3d::Identity(), {sphere},
                             {tracked_point_pose_moveit_frame}, std::vector<std::string>{}, "base");
    }

    // visualize
    visual_tools_->publishRobotState(robot_state, rviz_visual_tools::CYAN);

    return find_nearest_points_and_normals(planning_scene, cdcpd_to_moveit);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto cmn = std::make_shared<CDCPD_Moveit_Node>("hdt_michigan");
  
  // Initialize after shared_ptr is created
  cmn->init();

  return EXIT_SUCCESS;
}
