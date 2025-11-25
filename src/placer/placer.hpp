#pragma once
#include <vector>

class Design;
class LogicBlock;
class Net;
struct SwapResult;

class Placer {
 public:
  static double CalculateHPWL(const Design& design);
  static double CalculateCongestionCoefficient(const Design& design, std::vector<std::vector<int>>& usage_map);
  static double UpdateMapAndCalcHPWL(std::vector<std::vector<int>>& usage_map, const std::vector<Net*>& nets, const Design& design, int val);
  static SwapResult CalculateSwapDelta(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y, Design& design,
                                       std::vector<std::vector<int>>& usage_map, double current_total_hpwl, double current_cost);

  // static double CalculateHPWL(const Design& design);
  // static double CalculateCongestionCoefficient(const Design& design, std::vector<std::vector<int>>& usage_map);
  static void InitPlace(Design& design);
  static void RunSA(Design& design);
};

