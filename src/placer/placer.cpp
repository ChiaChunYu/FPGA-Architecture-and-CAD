#include "placer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "../design/design.hpp"

const double TIME_LIMIT_SECONDS = 200.0;

struct SwapResult {
  double delta_cost;
  double new_total_hpwl;
  double new_congestion;
  double new_cost;
};

double Placer::CalculateHPWL(const Design& design) {
  return design.CalculateTotalHPWL();
}

double Placer::CalculateCongestionCoefficient(const Design& design, std::vector<std::vector<int>>& usage_map) {
  long long num_sites = (long long)design.chip_width() * design.chip_height();

  double sum_usage = 0.0;
  double sum_squared_usage = 0.0;

  for (int x = 0; x < design.chip_width(); ++x) {
    for (int y = 0; y < design.chip_height(); ++y) {
      double val = (double)usage_map[x][y];
      sum_usage += val;
      sum_squared_usage += (val * val);
    }
  }

  double mean_squared_usage = sum_squared_usage / (double)num_sites;
  double mean_usage = sum_usage / (double)num_sites;
  double mean_usage_squared = mean_usage * mean_usage;

  if (mean_usage_squared == 0.0) return 1.0;

  return mean_squared_usage / mean_usage_squared;
}

double Placer::UpdateMapAndCalcHPWL(std::vector<std::vector<int>>& usage_map, const std::vector<Net*>& nets, const Design& design, int val) {
  double total_partial_hpwl = 0.0;
  int chip_w = design.chip_width();
  int chip_h = design.chip_height();

  for (auto net : nets) {
    BoundingBox bb = net->ComputeBoundingBox(nullptr);
    if (!bb.is_valid) continue;

    total_partial_hpwl += (bb.upper_x - bb.lower_x) + (bb.upper_y - bb.lower_y);
    int start_x = std::max(0, static_cast<int>(std::floor(bb.lower_x)));
    int end_x = std::min(chip_w, static_cast<int>(std::ceil(bb.upper_x)));
    int start_y = std::max(0, static_cast<int>(std::floor(bb.lower_y)));
    int end_y = std::min(chip_h, static_cast<int>(std::ceil(bb.upper_y)));

    for (int x = start_x; x < end_x; ++x) {
      for (int y = start_y; y < end_y; ++y) {
        usage_map[x][y] += val;
      }
    }
  }

  return total_partial_hpwl;
}

SwapResult Placer::CalculateSwapDelta(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y, Design& design,
                                      std::vector<std::vector<int>>& usage_map, double current_total_hpwl, double current_cost) {
  std::vector<Net*> affected_nets;
  size_t reserve_size = block1->nets().size();
  if (block2) reserve_size += block2->nets().size();
  affected_nets.reserve(reserve_size);

  affected_nets.insert(affected_nets.end(), block1->nets().begin(), block1->nets().end());
  if (block2) {
    affected_nets.insert(affected_nets.end(), block2->nets().begin(), block2->nets().end());
  }
  std::sort(affected_nets.begin(), affected_nets.end());
  affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()), affected_nets.end());

  double orig_partial_hpwl = UpdateMapAndCalcHPWL(usage_map, affected_nets, design, -1);

  int x1 = block1->x();
  int y1 = block1->y();
  block1->set_x(target_x);
  block1->set_y(target_y);
  if (block2) {
    block2->set_x(x1);
    block2->set_y(y1);
  }

  double new_partial_hpwl = UpdateMapAndCalcHPWL(usage_map, affected_nets, design, 1);
  double delta_hpwl = new_partial_hpwl - orig_partial_hpwl;
  double new_total_hpwl = current_total_hpwl + delta_hpwl;
  double new_congestion = Placer::CalculateCongestionCoefficient(design, usage_map);
  double new_cost = new_total_hpwl * new_congestion;

  return {new_cost - current_cost, new_total_hpwl, new_congestion, new_cost};
}

void Placer::InitPlace(Design& design) {
  int current_x = 0;
  int current_y = 0;
  int chip_width = design.chip_width();
  int chip_height = design.chip_height();

  for (auto block : design.logic_blocks()) {
    block->set_x(current_x);
    block->set_y(current_y);
    current_x++;
    if (current_x >= chip_width) {
      current_x = 0;
      current_y++;
    }
  }
  std::cout << "Initial placement completed." << std::endl;
}

void Placer::RunSA(Design& design) {
  std::cout << "[SA] Starting Simple SA (No Cache, Direct Calculation)..." << std::endl;

  Placer::InitPlace(design);
  std::srand(static_cast<unsigned>(std::time(nullptr)));
  auto sa_start_time = std::chrono::steady_clock::now();

  int chip_width = design.chip_width();
  int chip_height = design.chip_height();

  std::vector<std::vector<LogicBlock*>> grid(chip_width, std::vector<LogicBlock*>(chip_height, nullptr));
  for (auto block : design.logic_blocks()) {
    if (block->x() < chip_width && block->y() < chip_height) {
      grid[block->x()][block->y()] = block;
    }
  }

  std::vector<std::vector<int>> usage_map = design.CalculateUsageMap();
  double current_total_hpwl = design.CalculateTotalHPWL();
  double current_congestion_coefficient = CalculateCongestionCoefficient(design, usage_map);
  double current_cost = current_total_hpwl * current_congestion_coefficient;
  int round_count = 0;
  std::cout << "[SA] Init Cost: " << current_cost << " | HPWL: " << current_total_hpwl << " | CC: " << current_congestion_coefficient << std::endl;

  // SA Parameters
  double temperature = 5000;
  double min_temperature = 1e-5;
  // int moves_per_temperature = 10 * design.logic_blocks().size();
  int moves_per_temperature = 15 * std::pow(design.logic_blocks().size(), 1.2);
  // if (moves_per_temperature < 3000) moves_per_temperature = 3000;
  // if (moves_per_temperature > 400000) moves_per_temperature = 400000;

  while (temperature > min_temperature) {
    // auto current_time = std::chrono::steady_clock::now();
    // std::chrono::duration<double> elapsed_seconds = current_time - sa_start_time;
    // if (elapsed_seconds.count() > TIME_LIMIT_SECONDS) {
    //   std::cout << "\n[SA] Time Limit Reached. Stopping." << std::endl;
    //   break;
    // }

    int accepted_moves = 0;

    for (int i = 0; i < moves_per_temperature; ++i) {
      const auto& blocks = design.logic_blocks();
      LogicBlock* block1 = blocks[std::rand() % blocks.size()];
      int x1 = block1->x();
      int y1 = block1->y();
      int x2, y2;

      double random_val = static_cast<double>(std::rand()) / RAND_MAX;
      if (random_val < 0.7) {
        OptimalRegion region = block1->GetOptimalRegion(chip_width, chip_height);
        int width = region.upper_x - region.lower_x + 1;
        int height = region.upper_y - region.lower_y + 1;
        x2 = region.lower_x + (std::rand() % width);
        y2 = region.lower_y + (std::rand() % height);
      } else {
        x2 = std::rand() % chip_width;
        y2 = std::rand() % chip_height;
      }

      if (x1 == x2 && y1 == y2) continue;

      LogicBlock* block2 = grid[x2][y2];
      SwapResult result = Placer::CalculateSwapDelta(block1, block2, x2, y2, design, usage_map, current_total_hpwl, current_cost);
      grid[x1][y1] = block2;
      grid[x2][y2] = block1;

      bool accept = false;
      if (result.delta_cost < 0.7) {
        accept = true;
      } else {
        double r = static_cast<double>(std::rand()) / RAND_MAX;
        if (r < std::exp(-result.delta_cost / temperature)) {
          accept = true;
        }
      }
      if (accept) {
        current_cost = result.new_cost;
        current_total_hpwl = result.new_total_hpwl;
        current_congestion_coefficient = result.new_congestion;
        accepted_moves++;
      } else {
        Placer::CalculateSwapDelta(block1, block2, x1, y1, design, usage_map, 0, 0);
        grid[x1][y1] = block1;
        grid[x2][y2] = block2;
      }
    }

    // Update temperature
    double acceptance_rate = (double)accepted_moves / moves_per_temperature;
    double alpha = 0.8;
    if (acceptance_rate > 0.96)
      alpha = 0.5;
    else if (acceptance_rate > 0.8)
      alpha = 0.92;
    else if (acceptance_rate > 0.15)
      alpha = 0.95;
    temperature *= alpha;

    round_count++;
    std::cout << std::fixed << std::setprecision(4) << "Round " << std::setw(3) << round_count << " | T: " << std::scientific << std::setprecision(2)
              << temperature << " | Acc: " << std::fixed << std::setw(4) << (int)(acceptance_rate * 100) << "%"
              << " | Cost: " << std::scientific << std::setprecision(3) << current_cost << " | HPWL: " << std::fixed << std::setprecision(1)
              << current_total_hpwl << " | CC: " << std::setprecision(4) << current_congestion_coefficient << std::endl;
  }

  std::cout << "[SA] Final Cost: " << current_cost << " (HPWL: " << current_total_hpwl << ", CC: " << current_congestion_coefficient << ")"
            << std::endl;
}
