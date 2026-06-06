#include <benchmark/benchmark.h>
#include <aerial_robot_dynamics/robot_model.h>
#include <aerial_robot_dynamics/math_utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  bool verbose = false;
  bool is_floating_base = true;
  if (argc > 1)
  {
    std::string arg = argv[1];
    if (arg != "0")
      verbose = true;
  }
  if (argc > 2)
  {
    std::string arg = argv[2];
    if (arg == "0")
      is_floating_base = false;
  }

  aerial_robot_dynamics::PinocchioRobotModel pinocchio_robot_model(is_floating_base);

  // Test the robot model
  std::cout << "verbose: " << verbose << std::endl;
  std::cout << "is_floating_base: " << is_floating_base << std::endl;

  const int DATA_SIZE = 4096;
  std::vector<Eigen::VectorXd> q_vec(DATA_SIZE);
  std::vector<Eigen::VectorXd> thrust_vec(DATA_SIZE);

  for (int i = 0; i < DATA_SIZE; i++)
  {
    q_vec[i] = pinocchio::randomConfiguration(*(pinocchio_robot_model.getModel()));
    thrust_vec[i] = Eigen::VectorXd::Ones(pinocchio_robot_model.getRotorNum());
    aerial_robot_dynamics::addNoise(thrust_vec[i], 0.1);
  }

  benchmark::RegisterBenchmark("BM_tauext_partial_q_rnea", [&](benchmark::State& state) {
    size_t idx = 0;

    for (auto _ : state)
    {
      const auto& q = q_vec[idx & (DATA_SIZE - 1)];
      const auto& thrust = thrust_vec[idx & (DATA_SIZE - 1)];

      Eigen::MatrixXd tauext_by_thrust_q_derivative_rnea =
          pinocchio_robot_model.computeTauExtByThrustQDerivative(q, thrust);

      benchmark::DoNotOptimize(tauext_by_thrust_q_derivative_rnea);

      idx++;
    }
  });

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();

  return 0;
}
