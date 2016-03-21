#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.h>

using aikido::util::CatkinResourceRetriever;
using boost::format;
using boost::str;
using dart::common::Uri;
using dart::dynamics::Inertia;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNode;
using dart::dynamics::WeldJoint;
using dart::dynamics::PlanarJoint;
using dart::dynamics::FreeJoint;
using dart::dynamics::Joint;
using dart::dynamics::ShapePtr;
using dart::simulation::World;
using std::chrono::duration;
using std::chrono::duration_cast;
using Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Clock = std::chrono::steady_clock;

namespace po = boost::program_options;

namespace {

BodyNode* getBodyNode(const SkeletonPtr& _skeleton, const std::string& _name)
{
  if (!_skeleton)
    throw std::runtime_error("Skeleton is nullptr.");

  if (BodyNode* const bodynode = _skeleton->getBodyNode(_name))
    return bodynode;
  else 
    throw std::runtime_error(str(
      format("Skeleton '%s' has no BodyNode with name '%s'.")
      % _skeleton->getName() % _name));
}

PlanarJoint* convertFreeJointToPlanarJoint(FreeJoint* free_joint)
{
  if (!free_joint)
    return nullptr;

  const FreeJoint::Properties& free_properties
    = free_joint->getFreeJointProperties();

  PlanarJoint::Properties planar_properties;
  static_cast<Joint::Properties&>(planar_properties) = free_properties;
  planar_properties.setXYPlane();

  BodyNode* const child_bodynode = free_joint->getChildBodyNode();
  return child_bodynode->changeParentJointType<PlanarJoint>(planar_properties);
}

void computeInertiaFromGeometry(BodyNode* bodynode, double density)
{
  if (bodynode->getNumCollisionShapes() > 1)
    throw std::runtime_error(str(
      format("BodyNode '%s' on Skeleton '%s' has %d collision shapes; only"
             " one is supported for computing an inertia tensor.")
      % bodynode->getName() % bodynode->getSkeleton()->getName()
      % bodynode->getNumCollisionShapes()));
  else if (bodynode->getNumCollisionShapes() == 0)
    return;

  const ShapePtr shape = bodynode->getCollisionShape(0);
  const double mass = density * shape->getVolume();
  const Inertia inertia(
    mass, shape->getOffset(), shape->computeInertia(mass));

  std::cout << "BodyNode '" << bodynode->getName() << "'\n"
            << "  origin = " << inertia.getLocalCOM().transpose() << "\n"
            << "  mass = " << inertia.getMass() << "\n"
            << "  inertia = "
            << inertia.getMoment().diagonal().transpose() << "\n"
            << std::endl;

  bodynode->setInertia(inertia);
}

void replaceRenderGeometryWithCollision(BodyNode* bodynode)
{
  bodynode->removeAllVisualizationShapes();

  for (size_t icol = 0; icol < bodynode->getNumCollisionShapes(); ++icol)
    bodynode->addVisualizationShape(bodynode->getCollisionShape(icol));
}

} // namespace

int main(int argc, char **argv)
{
  static constexpr char urdf_path[] = "package://dart_benchmark/models/block_pushing.urdf";
  static constexpr double viewer_refresh_rate = 30.;
  static constexpr double floor_friction = 1.;
  static constexpr double object_friction = 0.25;
  static constexpr double position_gain = 500.;
  static constexpr double orientation_gain = 500.;

  std::string controller_type, collision_type, joint_type;
  double timestep, total_duration, desired_forward_velocity;
  bool has_viewer;

  {
    po::options_description description("Options");
    description.add_options()
      ("velocity",
       po::value<double>(&desired_forward_velocity)->default_value(0.01),
        "desired pushing velocity, in m/s")
      ("duration", po::value<double>(&total_duration)->default_value(50.),
        "simulation duration, in seconds")
      ("timestep", po::value<double>(&timestep)->default_value(0.01),
        "simulation timestep, in seconds")
      ("collision",
        po::value<std::string>(&collision_type)->default_value("fcl"),
        "collision detector type")
      ("controller",
        po::value<std::string>(&controller_type)->default_value("pid"),
        "controller type")
      ("joint",
        po::value<std::string>(&joint_type)->default_value("free"),
        "joint type")
      ("view", po::bool_switch(&has_viewer)->default_value(false),
        "open a viewer")
      ("help", "display help text")
      ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      std::cout << description << std::endl;
      return 1;
    }
  }

  const size_t num_substeps = std::max<size_t>(1,
    static_cast<size_t>(1. / (viewer_refresh_rate * timestep) + 0.5));
  const size_t num_timesteps = static_cast<size_t>(
    total_duration / timestep + 0.5);

  std::cout << "Creating CollisionDetector." << std::endl;
  std::unique_ptr<dart::collision::CollisionDetector> collision_detector;
  if (collision_type == "bullet")
    collision_detector.reset(new dart::collision::BulletCollisionDetector);
  else if (collision_type == "fcl")
    collision_detector.reset(new dart::collision::FCLCollisionDetector);
  else if (collision_type == "fcl_mesh")
    collision_detector.reset(new dart::collision::FCLMeshCollisionDetector);
  else
    throw std::runtime_error(str(
      format("Unknown type of collision detector '%s'.") % collision_type));

  std::cout << "Loading URDF from: " << urdf_path << std::endl;
  dart::utils::DartLoader loader;
  const auto retriever = std::make_shared<CatkinResourceRetriever>();
  const SkeletonPtr skeleton = loader.parseSkeleton(urdf_path, retriever);
  skeleton->enableSelfCollision(true);

  std::cout << "Setting Joint and BodyNode properties." << std::endl;
  BodyNode* const floor = getBodyNode(skeleton, "map");
  floor->changeParentJointType<WeldJoint>();
  floor->setFrictionCoeff(floor_friction);
  floor->setRestitutionCoeff(0.);

  BodyNode* const pushee = getBodyNode(skeleton, "pushee");
  Joint* pushee_joint = pushee->getParentJoint();
  pushee_joint->setActuatorType(Joint::PASSIVE);
  pushee->setFrictionCoeff(object_friction);
  pushee->setRestitutionCoeff(0.);

  BodyNode* const pusher = getBodyNode(skeleton, "pusher");
  Joint* pusher_joint = pusher->getParentJoint();
  pusher->setFrictionCoeff(object_friction);
  pusher->setRestitutionCoeff(0.);

  if (joint_type == "free")
    ; // do nothing
  else if (joint_type == "planar")
  {
    pushee_joint = convertFreeJointToPlanarJoint(
      dynamic_cast<FreeJoint*>(pushee_joint));
    pusher_joint = convertFreeJointToPlanarJoint(
      dynamic_cast<FreeJoint*>(pusher_joint));
  }
  else
    throw std::runtime_error(str(
      format("Unknown joint type '%s'.") % joint_type));

  if (controller_type == "servo")
    pusher_joint->setActuatorType(Joint::SERVO);
  else if (controller_type == "pid")
    pusher_joint->setActuatorType(Joint::FORCE);
  else
    throw std::runtime_error(str(
      format("Unknown control type '%s'.") % controller_type));

  std::cout << "Initializing simulation." << std::endl;
  World world;
  world.setTimeStep(timestep);
  world.addSkeleton(skeleton);
  world.getConstraintSolver()->setCollisionDetector(
    std::move(collision_detector));

  std::cout << "Running simulation for " << num_timesteps << " timesteps."
            << std::endl;

  Eigen::VectorXd desired_velocity, feedforward_force;
  std::vector<size_t> position_indices, orientation_indices;

  if (joint_type == "free")
  {
    world.setGravity(Eigen::Vector3d(0., 0., -9.81));

    orientation_indices = {0, 1, 2};
    position_indices = {3, 4, 5};

    desired_velocity = Vector6d::Zero();
    desired_velocity[3] = desired_forward_velocity;

    feedforward_force = Vector6d::Zero();
    feedforward_force[3] = (pusher->getMass() + pushee->getMass())
      * world.getGravity().norm() * std::sqrt(floor_friction * object_friction);
  }
  else if (joint_type == "planar")
  {
    position_indices = {0, 1};
    orientation_indices = {2};

    // Disable gravity and collision with the floor, since the planar joint
    // handles this constraint.
    world.setGravity(Eigen::Vector3d::Zero());
    floor->setCollidable(false);

    desired_velocity = Eigen::Vector3d::Zero();
    desired_velocity[0] = desired_forward_velocity;

    feedforward_force = Eigen::Vector3d::Zero();
    feedforward_force[0] = (pusher->getMass() + pushee->getMass())
      * world.getGravity().norm() * std::sqrt(floor_friction * object_friction);
  }
  else
    assert(false);

  // Create a viewer.
  std::unique_ptr<aikido::rviz::InteractiveMarkerViewer> viewer;
  std::unique_ptr<ros::NodeHandle> nh;
  Clock::time_point time_render;
  Clock::duration period_render;

  if (has_viewer)
  {
    std::cout << "Starting ROS node." << std::endl;
    ros::init(argc, argv, "dart_urdf_demo");
    nh.reset(new ros::NodeHandle);

    std::cout << "Creating viewer that renders every "
      << num_substeps << " timesteps ("
      << (1. / (num_substeps * timestep)) << " Hz)." << std::endl;

    viewer.reset(new aikido::rviz::InteractiveMarkerViewer("dart_markers"));
    viewer->addSkeleton(skeleton);
    viewer->update();

    period_render = duration_cast<Clock::duration>(
      duration<double>(num_substeps * timestep));
    time_render = Clock::now() + period_render;
  }

  // Run the simulation.
  std::cout << "Entering simulation loop." << std::endl;

  const Clock::time_point time_before = Clock::now();
  for (size_t istep = 0; istep < num_timesteps; ++istep)
  {
    if (controller_type == "servo")
    {
      pusher_joint->setCommands(desired_velocity);
    }
    else if (controller_type == "pid")
    {
      for (const size_t idof : orientation_indices)
        pusher_joint->setCommand(idof, feedforward_force[idof]
          - orientation_gain * pusher_joint->getPosition(idof));

      for (const size_t idof : position_indices)
        pusher_joint->setCommand(idof, feedforward_force[idof] + position_gain
          * (desired_velocity[idof] - pusher_joint->getVelocity(idof)));
    }
    else
      assert(false);

#ifndef NDEBUG
    std::cout
      << "current_velocity = " << pusher_joint->getVelocities().transpose() << "\n"
      << "desired_velocity = " << desired_velocity.transpose() << "\n"
      << "current_position = " << pusher_joint->getPositions().transpose() << "\n"
      << "feedforward_force = " << feedforward_force.transpose() << "\n"
      << "command = " << pusher_joint->getCommands().transpose() << "\n"
      << "---\n"
      << std::flush;
#endif

    // Simulate.
    world.step();

    // Periodically update the viewer.
    if (viewer && istep % num_substeps == 0)
    {
      viewer->update();

      const Clock::time_point now = Clock::now();
      if (now > time_render)
        std::cerr << "Warning: Overran viewer update by "
          << duration_cast<duration<double>>(now - time_render).count()
          << " s." << std::endl;

      std::this_thread::sleep_until(time_render);
      time_render += period_render;
    }
  }

  const Clock::time_point time_after = Clock::now();
  const double sim_elapsed = num_timesteps * timestep;
  const double wall_elapsed = duration_cast<duration<double>>(
    time_after - time_before).count();

  std::cout << "Exiting simulation loop.\n\n"
    << "Results:\n"
    << "I | Timestep:           " << timestep << "\n"
    << "I | Number of Timestep: " << num_timesteps << "\n"
    << "I | Actuator Type:      " << controller_type << "\n"
    << "I | Collision Detector: " << collision_type << "\n"
    << "O | Wall Time:          " << wall_elapsed << " s\n"
    << "O | Simulation Time:    " << sim_elapsed << " s\n"
    << "O | Realtime Ratio:     " << sim_elapsed / wall_elapsed << "\n" 
    << std::flush;

  if (viewer)
    ros::shutdown();

  return 0;
}
