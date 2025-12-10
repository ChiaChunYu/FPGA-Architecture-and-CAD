#pragma once
#include <chrono>
#include <random>
#include <vector>

#include "../design/design.hpp"

struct Config {
  // ---------- time ----------
  double time_limit_seconds = 220.0;

  // ---------- temperature ----------
  double min_temperature = 1e-5;

  // ---------- optimal region probability ----------
  double initial_region_prob = 0.5;
  double min_region_prob = 0.5;
  double max_region_prob = 0.9;
  double region_prob_step_up = 0.02;
  double region_prob_step_down = 0.02;

  // ---------- moves per temperature ----------
  int initial_moves_per_temp = 60000;
  int min_moves_per_temp = 40000;
  int max_moves_per_temp = 80000;
  double moves_scale_up = 1.2;
  double moves_scale_down = 0.8;

  // ---------- range limiter (Previously window_scale) ----------
  double initial_range_limiter = 1.0;
  double min_range_limiter = 0.01;
  double max_range_limiter = 1.0;

  // ---------- Strategies ----------
  // true: Use CalcOptimalRegion (median/quantile method)
  // false: Use CalcCenter (mean method + search radius)
  bool use_optimal_region_calc = false;

  // ---------- random seed ----------
  // 0 represnt use random_device
  unsigned int random_seed = 7777;
};

struct State {
  double total_hpwl = 0.0;
  long long sum_usage = 0;
  long long sum_squared_usage = 0;
};

struct AffectedNetInfo {
  Net* net;
  bool has_b1;
  bool has_b2;
};

class Placer {
 public:
  Placer(Design& design, const Config& config, std::chrono::steady_clock::time_point start_time);
  void InitPlace();
  void Run();

 private:
  Design& design_;
  Config config_;
  std::chrono::steady_clock::time_point start_time_;

  State state_;
  std::vector<int> usage_map_;
  std::vector<LogicBlock*> grid_graph_;
  std::mt19937 rng_;

  void InitState();
  double EstimateInitTemperature();

  double CongestionCoefficient() const;
  double CalcCost() const;

  void UpdateState(const std::vector<Net*>& nets, int val);
  void SwapPosition(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y);

  void UpdateParameters(double& temperature, double& region_prob, int& moves_per_temperature, double& range_limiter, double initial_temperature,
                        const double& acceptance_rate);
};