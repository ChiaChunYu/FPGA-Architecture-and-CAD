#pragma once
#include <random>
#include <vector>

class Design;
class LogicBlock;
class Net;

struct SAData  {
  double total_hpwl;
  long long sum_usage;
  long long sum_squared_usage;
};

class Placer {
 public:
  static void RunSA(Design& design);

 private:
  static void InitPlace(Design& design);

  static double CalculateCongestionCoefficient(const Design& design, const SAData& sa_data);
  static double ComputeCost(const Design& design, const SAData& sa_data);

  static void InitializeData(const Design& design, const std::vector<std::vector<int>>& usage_map, SAData& sa_data);
  static void UpdateData(const Design& design, std::vector<std::vector<int>>& usage_map, SAData& sa_data, const std::vector<Net*>& nets, int val);
  static void SwapPosition(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y, Design& design,
                           std::vector<std::vector<int>>& usage_map, SAData& sa_data, std::vector<std::vector<LogicBlock*>>& grid_graph);

  static double EstimateInitialTemperature(Design& design, std::vector<std::vector<int>>& usage_map, SAData& sa_data,
                                           std::vector<std::vector<LogicBlock*>>& grid_graph, std::mt19937& rng);
};