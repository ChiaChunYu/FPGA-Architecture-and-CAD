#pragma once
#include <chrono>
#include <random>
#include <vector>

#include "../design/design.hpp"

struct Config {
  double time_limit_seconds = 220.0;
  double initial_region_prob = 0.3;
  double region_prob_step = 0.02;
  double max_region_prob = 0.9;
  double min_temperature = 1e-5;
  int initial_moves = 50000;
};

struct State {
  double total_hpwl = 0.0;
  long long sum_usage = 0;
  long long sum_squared_usage = 0;
};

class Placer {
 public:
  Placer(Design& design, const Config& config, std::chrono::steady_clock::time_point start_time);
  void Run();

 private:
  Design& design_;
  Config config_;
  std::chrono::steady_clock::time_point start_time_;

  State state_;
  std::vector<std::vector<int>> usage_map_;
  std::vector<std::vector<LogicBlock*>> grid_graph_;
  std::mt19937 rng_;

  void InitPlace();
  void InitState();
  double EstimateInitTemperature();

  double CongestionCoefficient() const;
  double CalcCost() const;

  void UpdateState(const std::vector<Net*>& nets, int val);
  void SwapPosition(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y);
  void UpdateParameters(double& temperature, double& region_prob, int& moves_per_temperature, const double& acceptance_rate);
};