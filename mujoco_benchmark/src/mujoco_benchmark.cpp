#include <array>
#include <chrono>
#include <iostream>
#include <boost/program_options.hpp>
#include <MuJoCo/glfw3.h>
#include <MuJoCo/mujoco.h>
#include <Eigen/Dense>

namespace po = boost::program_options;
using std::chrono::duration;
using std::chrono::duration_cast;

int main(int argc, char **argv)
{
  static const char pusher_name[] = "pusher";
  static const char pushee_name[] = "pushee";
  static const char model_path[] = "models/block_pushing.mjcf";
  static const char key_path[] = "mjkey.txt";
  static constexpr double viewer_refresh_rate = 30.;
  static constexpr int font_scale(150);
  static constexpr int num_objects(1000);
  static constexpr double position_gain = 500.;
  static constexpr double orientation_gain = 250.;

  double desired_forward_velocity;
  double total_duration;
  double timestep;
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
  static const mjtNum desired_velocity[6] {
    desired_forward_velocity, 0., 0., 0., 0., 0. };

  std::cout << "Reading MuJoCo key from: " << key_path << std::endl;
	mj_activate(key_path);

  // Create the hand.
  std::array<char, 1000> error = {"could not load model"};
  mjModel* model = mj_loadXML(model_path, error.data());
  if (!model)
  {
    std::cerr << "Failed loading model '" << model_path << "': "
              << error.data() << std::endl;
    return 0;
  }

  model->opt.timestep = timestep;

  mjData* hand_data = mj_makeData(model);
  mj_forward(model, hand_data);

  std::cout << "Printing model to: " << "MUJOCO_OUTPUT.txt" << std::endl;
  mj_printModel(model, "MUJOCO_OUTPUT.txt");

  // Viewer.
  mjvObjects objects;
  mjvCamera cam;
  mjvOption vopt;
  mjrOption ropt;
  mjrContext con;
  GLFWwindow* window = nullptr;

  if (has_viewer)
  {
    std::cout << "Initializing abstract visualization." << std::endl;
    mjv_makeObjects(&objects, num_objects);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);

    // Initialize MuJoCo OpenGL rendering.
    std::cout << "Initializing OpenGL rendering." << std::endl;
    if (!glfwInit())
    {
      std::cerr << "Failed initializing GLFW." << std::endl;
      return 1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);

    window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    if (!window)
    {
      glfwTerminate();
      std::cerr << "Failed creating GLFW window." << std::endl;
      return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSetWindowTitle(window, "muul with MuJoCo");

    mjr_defaultOption(&ropt);
    mjr_defaultContext(&con);
    mjr_makeContext(model, &con, font_scale);
  }

  // Get references into the MuJoCo arrays for the pusher.
  const int hand_body_index = mj_name2id(model, mjOBJ_BODY, pusher_name);
  if (hand_body_index < 0)
    throw std::runtime_error("There is no MuJoCo body with this name.");

  const int hand_joint_index = model->body_jntadr[hand_body_index];
  if (hand_joint_index < 0)
    throw std::runtime_error("This MuJoCo body does have any joints.");
  if (model->jnt_type[hand_joint_index] != mjJNT_FREE)
    throw std::runtime_error("MuCoJo joint is not of type JNT_FREE.");

  const int hand_qpos_index = model->jnt_qposadr[hand_joint_index];
  if (hand_qpos_index < 0)
    throw std::runtime_error("This MuJoCo body does any position values.");

  const int hand_dof_index = model->jnt_dofadr[hand_joint_index];
  if (hand_dof_index < 0)
    throw std::runtime_error("This MuJoCo joint has no DOFs.");

  // Get references into the MuJoCo arrays for the pushee.
  const int pushee_body_index = mj_name2id(model, mjOBJ_BODY, pushee_name);
  if (pushee_body_index < 0)
    throw std::runtime_error("There is no MuJoCo body with this name.");

  const int pushee_joint_index = model->body_jntadr[pushee_body_index];
  if (pushee_joint_index < 0)
    throw std::runtime_error("This MuJoCo body does have any joints.");
  if (model->jnt_type[pushee_joint_index] != mjJNT_FREE)
    throw std::runtime_error("MuCoJo joint is not of type JNT_FREE.");

  const int pushee_qpos_index = model->jnt_qposadr[pushee_joint_index];
  if (pushee_qpos_index < 0)
    throw std::runtime_error("This MuJoCo body does any position values.");

  const int pushee_dof_index = model->jnt_dofadr[pushee_joint_index];
  if (pushee_dof_index < 0)
    throw std::runtime_error("This MuJoCo joint has no DOFs.");

  // Set the initial state of the simulation.

  // Main loop.
  std::cout << "Entering main loop." << std::endl;

  const auto time_before = std::chrono::steady_clock::now();

  for (int istep = 0; istep < num_timesteps; ++istep)
  {
    const Eigen::AngleAxis<mjtNum> orientation(Eigen::Quaternion<mjtNum>(
      hand_data->xquat[4 * hand_body_index + 0],
      hand_data->xquat[4 * hand_body_index + 1],
      hand_data->xquat[4 * hand_body_index + 2],
      hand_data->xquat[4 * hand_body_index + 3]));
    const Eigen::Matrix<mjtNum, 3, 1> orientation_twist(
      orientation.angle() * orientation.axis());

    Eigen::Map<Eigen::Matrix<mjtNum, 6, 1>> qfrc_applied(
      hand_data->qfrc_applied + hand_dof_index);
    Eigen::Map<const Eigen::Matrix<mjtNum, 6, 1>> qpos(
      hand_data->qpos + hand_qpos_index);
    Eigen::Map<const Eigen::Matrix<mjtNum, 6, 1>> qvel(
      hand_data->qvel + hand_dof_index);

    // Run a P controller on velocity.
    for (size_t idof = 0; idof < 3; ++idof)
      qfrc_applied[idof] = position_gain * (desired_velocity[idof] - qvel[idof]);

    for (size_t idof = 0; idof < 3; ++idof)
      qfrc_applied[idof + 3] = orientation_gain * -orientation_twist[idof];

    // Add feed-forward for the push.
    // TODO: Is there a better way of doing this?
    qfrc_applied[0] += 88.29 * 1.5;
    qfrc_applied[4] += -0.1 * qfrc_applied[0];

		mj_step(model, hand_data);

    if (window)
    {
      if (glfwWindowShouldClose(window))
        break;

      if (istep % num_substeps == 0)
      {
        mjrRect rect = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &rect.width, &rect.height);

        mjv_makeGeoms(model, hand_data, &objects, &vopt, mjCAT_ALL, 0,
          nullptr, nullptr, nullptr);
        mjv_makeLights(model, hand_data, &objects);
        mjv_setCamera(model, hand_data, &cam);
        mjv_updateCameraPose(&cam, (mjtNum)rect.width/(mjtNum)rect.height);
        mjr_render(0, rect, &objects, &ropt, &cam.pose, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
      }
    }
  }

  const auto time_after = std::chrono::steady_clock::now();
  const double wall_elapsed = duration_cast<duration<double>>(
    time_after - time_before).count();
  const double sim_elapsed = num_timesteps * timestep;

  std::cout << "Exiting simulation loop.\n\n"
    << "Results:\n"
    << "I | Timestep:           " << timestep << "\n"
    << "I | Number of Timestep: " << num_timesteps << "\n"
    << "O | Wall Time:          " << wall_elapsed << " s\n"
    << "O | Simulation Time:    " << sim_elapsed << " s\n"
    << "O | Realtime Ratio:     " << sim_elapsed / wall_elapsed << "\n" 
    << std::flush;

  mj_deleteData(hand_data);
  mj_deleteModel(model);

  if (window)
  {
    mjr_freeContext(&con);
    mjv_freeObjects(&objects);
  }

  glfwTerminate();
  mj_deactivate();

  std::cout << "Done." << std::endl;

  return 0;
}
