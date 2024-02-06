#include <example_behaviors/setup_mtc_pick_from_pose.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/yaml_parsing_tools.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
const auto kLogger = rclcpp::get_logger("SetupMTCPickFromPose");
using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
using moveit_studio::behaviors::parseParameter;

// Port names for input and output ports.
constexpr auto kPortIDTask = "task";
constexpr auto kPortIDObjectiveParameters = "parameters";
constexpr auto kPortIDGraspPose = "grasp_pose";

// Parameter names for the behavior parameters.
constexpr auto kWorldFrameNameParameter = "world_frame_name";
constexpr auto kArmGroupNameParameter = "arm_group_name";
constexpr auto kEndEffectorGroupNameParameter = "end_effector_group_name";
constexpr auto kEndEffectorNameParameter = "end_effector_name";
constexpr auto kHandFrameNameParameter = "hand_frame_name";
constexpr auto kHandOpenNameParameter = "end_effector_opened_pose_name";
constexpr auto kHandCloseNameParameter = "end_effector_closed_pose_name";
constexpr auto kApproachDistanceParameter = "approach_distance";
constexpr auto kLiftDistanceParameter = "lift_distance";

// behavior constants
constexpr auto kPropertyNameTrajectoryExecutionInfo = "trajectory_execution_info";
constexpr double kIKTimeoutSeconds = 1.0;
constexpr int kMaxIKSolutions = 20;
constexpr auto kSceneObjectNameOctomap = "<octomap>";
constexpr auto kSceneObjectName = "object";
}  // namespace

namespace example_behaviors
{
SetupMtcPickFromPose::SetupMtcPickFromPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SetupMtcPickFromPose::providedPorts()
{
  return {
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortIDTask),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose),
    BT::InputPort<YAML::Node>(kPortIDObjectiveParameters),
  };
}

BT::NodeStatus SetupMtcPickFromPose::tick()
{
  using namespace moveit_studio::behaviors;

  // ----------------------------------------
  // Load data from the behavior input ports.
  // ----------------------------------------
  const auto grasp_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDGraspPose);
  const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIDTask);
  const auto objective_parameters = getInput<YAML::Node>(kPortIDObjectiveParameters);

  // Check that all required input data ports were set
  if (const auto error = maybe_error(grasp_pose, task, objective_parameters); error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  // ---------------------------------------------------------------------------
  // Load behavior specific parameters defined in a separate configuration file.
  // ---------------------------------------------------------------------------
  const auto behavior_parameters = parseParameter<YAML::Node>(objective_parameters.value(), name());
  if (!behavior_parameters)
  {
    shared_resources_->logger->publishFailureMessage(name(),
                                                     "Could not find behavior specific parameters in the configuration "
                                                     "file: " +
                                                         behavior_parameters.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto world_frame_name = parseParameter<std::string>(behavior_parameters.value(), kWorldFrameNameParameter);
  const auto arm_group_name = parseParameter<std::string>(behavior_parameters.value(), kArmGroupNameParameter);
  const auto end_effector_group_name =
      parseParameter<std::string>(behavior_parameters.value(), kEndEffectorGroupNameParameter);
  const auto end_effector_name = parseParameter<std::string>(behavior_parameters.value(), kEndEffectorNameParameter);
  const auto hand_frame_name = parseParameter<std::string>(behavior_parameters.value(), kHandFrameNameParameter);
  const auto hand_opened_name = parseParameter<std::string>(behavior_parameters.value(), kHandOpenNameParameter);
  const auto hand_closed_name = parseParameter<std::string>(behavior_parameters.value(), kHandCloseNameParameter);
  const auto approach_distance = parseParameter<double>(behavior_parameters.value(), kApproachDistanceParameter);
  const auto lift_distance = parseParameter<double>(behavior_parameters.value(), kLiftDistanceParameter);

  if (const auto error =
          moveit_studio::behaviors::maybe_error(world_frame_name, arm_group_name, end_effector_group_name,
                                                end_effector_name, hand_frame_name, approach_distance, lift_distance);
      error)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Parsing behavior parameters failed: " + *error);
    return BT::NodeStatus::FAILURE;
  }

  // Create planners
  const auto mtc_pipeline_planner =
      std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(shared_resources_->node);
  const auto mtc_joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

  // Spawn object
  // This stage spawns a collision object for picking
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Spawn object");
    moveit_msgs::msg::CollisionObject object;
    object.id = kSceneObjectName;
    object.header.frame_id = "world";
    object.pose = grasp_pose.value().pose;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { 0.05, 0.05, 0.05 };
    // object.pose.position.z += 2;
    stage->addObject(object);
    task.value()->add(std::move(stage));
  }

  /** Open Hand **/
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveTo>("Open hand", mtc_joint_interpolation_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                          { kPropertyNameTrajectoryExecutionInfo });
    stage->setGroup(end_effector_group_name.value());
    stage->setGoal(hand_opened_name.value());
    task.value()->add(std::move(stage));
  }

  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Pick From Pose");
  container->properties().set(kPropertyNameTrajectoryExecutionInfo,
                              boost::any_cast<moveit::task_constructor::TrajectoryExecutionInfo>(
                                  task.value()->properties().get(kPropertyNameTrajectoryExecutionInfo)));
  container->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);

  container->setProperty("group", arm_group_name.value());
  container->setProperty("hand", end_effector_group_name.value());
  container->setProperty("eef", end_effector_name.value());
  container->setProperty("ik_frame", hand_frame_name.value());

  /** Move To Pre-Grasp Pose **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::Connect>(
        "Move to Pre-Approach Pose", moveit::task_constructor::stages::Connect::GroupPlannerVector{
                                         { arm_group_name.value(), mtc_pipeline_planner } });
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                          { kPropertyNameTrajectoryExecutionInfo });
    stage->setTimeout(10);
    container->add(std::move(stage));
  }

  // Set Allowed Collisions
  // This stage forbids collisions between the gripper and the octomap before the stage (during the Move To Pre-Grasp
  // Pose stage, so the gripper doesn't collide with objects while moving into position), and allows them after this
  // stage.
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Allow collision 1 (hand,object)");

    stage->allowCollisions(kSceneObjectName,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    container->add(std::move(stage));
  }

  const Eigen::Vector3d approach_vector{ 0.0, 0.0, approach_distance.value() };

  /** Approach Grasp **/
  {
    // Send relative move to MTC stage
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Approach", mtc_cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                          { kPropertyNameTrajectoryExecutionInfo });
    stage->restrictDirection(moveit::task_constructor::stages::MoveRelative::BACKWARD);
    stage->setGroup(arm_group_name.value());
    stage->setIKFrame(hand_frame_name.value());

    geometry_msgs::msg::Vector3Stamped approach_vector_msg;
    tf2::toMsg(approach_vector, approach_vector_msg.vector);
    approach_vector_msg.header.frame_id = hand_frame_name.value();

    stage->setDirection(approach_vector_msg);
    stage->setTimeout(10);
    container->add(std::move(stage));
  }

  // Set Allowed Collisions
  // This stage allows collisions between the gripper and the octomap before the stage (during the Approach Grasp
  // stage, so the gripper can move into the octomap), and forbids them after this stage.
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Allow collision 2 (hand,object)");
    stage->allowCollisions(kSceneObjectName,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    container->insert(std::move(stage));
  }

  // Generate the Inverse Kinematic (IK) solutions to move to the pose specified in the "grasp_pose" input port.
  // This will generate up to kMaxIKSolutions IK solution candidates to sample from, unless the timeout specified in
  // kIKTimeoutSeconds is reached first.
  // Collision checking is ignored for IK pose generation. Solutions that result in forbidden collisions will be
  // eliminated by failures in the stages before and after this one.
  {
    // Specify pose to generate for
    auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePose>("Generate pose");
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_frame");
    stage->setPose(grasp_pose.value());
    stage->setMonitoredStage(task.value()->stages()->findChild("Open hand"));  // Hook into current state

    // Compute IK
    auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("Pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(kMaxIKSolutions);
    wrapper->setTimeout(kIKTimeoutSeconds);
    wrapper->setIKFrame(hand_frame_name.value());
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
    wrapper->setIgnoreCollisions(true);
    container->add(std::move(wrapper));
  }

  // Allow Collision
  // This stage allows collisions between the gripper and object for stages after this one (during the Close Hand,
  // Lift, and Retreat stages).
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Allow collision 3 (hand,object)");
    stage->allowCollisions(kSceneObjectName,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           true);
    container->insert(std::move(stage));
  }

  /** Close Hand **/
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveTo>("Close hand", mtc_joint_interpolation_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                          { kPropertyNameTrajectoryExecutionInfo });
    stage->setGroup(end_effector_group_name.value());
    stage->setGoal(hand_closed_name.value());
    container->add(std::move(stage));
  }

  /** Attach Object **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Attach");

    stage->attachObject(kSceneObjectName, kHandFrameNameParameter);
    container->add(std::move(stage));
  }

  /** Lift Object **/
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Lift", mtc_cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                          { kPropertyNameTrajectoryExecutionInfo });
    stage->setGroup(arm_group_name.value());
    stage->setIKFrame(hand_frame_name.value());

    geometry_msgs::msg::Vector3Stamped lift_vector_msg;
    lift_vector_msg.header.frame_id = world_frame_name.value();
    lift_vector_msg.vector.z = lift_distance.value();

    stage->setDirection(lift_vector_msg);
    container->add(std::move(stage));
  }

  /** Retreat **/
  {
    // Send relative move to MTC stage
    auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Retreat", mtc_cartesian_planner);
    stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                          { kPropertyNameTrajectoryExecutionInfo });
    stage->setGroup(arm_group_name.value());
    stage->setIKFrame(hand_frame_name.value());

    geometry_msgs::msg::Vector3Stamped retreat_vector_msg;
    tf2::toMsg(approach_vector * -1, retreat_vector_msg.vector);
    retreat_vector_msg.header.frame_id = hand_frame_name.value();

    stage->setDirection(retreat_vector_msg);
    container->add(std::move(stage));
  }

  // Forbid Collision
  // This stage forbids collisions between the gripper and the object for subsequent stages.
  {
    auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("Forbid collision (hand,object)");
    stage->allowCollisions(kSceneObjectName,
                           task.value()
                               ->getRobotModel()
                               ->getJointModelGroup(end_effector_group_name.value())
                               ->getLinkModelNamesWithCollisionGeometry(),
                           false);
    container->add(std::move(stage));
  }

  task.value()->add(std::move(container));

  return BT::NodeStatus::SUCCESS;
}
}  // namespace example_behaviors
