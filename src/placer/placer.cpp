#include "placer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "../design/design.hpp"

const double TIME_LIMIT_SECONDS = 220.0;

void Placer::InitPlace(Design& design) {
  int current_x = 0;
  int current_y = 0;
  int chip_width = design.chip_width();

  for (auto block : design.logic_blocks()) {
    block->set_x(current_x);
    block->set_y(current_y);
    current_x++;
    if (current_x >= chip_width) {
      current_x = 0;
      current_y++;
    }
  }
}

double Placer::CalculateCongestionCoefficient(const Design& design, const SAData& sa_data) {
  long long num_sites = static_cast<long long>(design.chip_width()) * static_cast<long long>(design.chip_height());
  double mean_usage = sa_data.sum_usage / static_cast<double>(num_sites);
  double mean_sq = sa_data.sum_squared_usage / static_cast<double>(num_sites);

  if (mean_usage == 0.0) {
    return 1.0;
  }

  return mean_sq / (mean_usage * mean_usage);
}

double Placer::ComputeCost(const Design& design, const SAData& sa_data) {
  double congestion_coefficient = CalculateCongestionCoefficient(design, sa_data);
  return sa_data.total_hpwl * congestion_coefficient;
}

void Placer::InitializeData(const Design& design, const std::vector<std::vector<int>>& usage_map, SAData& sa_data) {
  int chip_width = design.chip_width();
  int chip_height = design.chip_height();
  long long sum_usage = 0;
  long long sum_squared_usage = 0;

  for (int x = 0; x < chip_width; ++x) {
    for (int y = 0; y < chip_height; ++y) {
      long long val = usage_map[x][y];
      sum_usage += val;
      sum_squared_usage += val * val;
    }
  }

  sa_data.total_hpwl = design.GetTotalHPWL();
  sa_data.sum_usage = sum_usage;
  sa_data.sum_squared_usage = sum_squared_usage;
}

void Placer::UpdateData(const Design& design, std::vector<std::vector<int>>& usage_map, SAData& sa_data, const std::vector<Net*>& nets, int val) {
  int chip_width = design.chip_width();
  int chip_height = design.chip_height();

  for (auto net : nets) {
    BoundingBox boundingbox = net->ComputeBoundingBox();
    if (!boundingbox.is_valid) continue;
    double hpwl = (boundingbox.upper_x - boundingbox.lower_x) + (boundingbox.upper_y - boundingbox.lower_y);
    sa_data.total_hpwl += val * hpwl;
    int start_x = std::max(0, static_cast<int>(std::floor(boundingbox.lower_x)));
    int end_x = std::min(chip_width, static_cast<int>(std::ceil(boundingbox.upper_x)));
    int start_y = std::max(0, static_cast<int>(std::floor(boundingbox.lower_y)));
    int end_y = std::min(chip_height, static_cast<int>(std::ceil(boundingbox.upper_y)));

    for (int x = start_x; x < end_x; ++x) {
      for (int y = start_y; y < end_y; ++y) {
        long long old_val = usage_map[x][y];
        long long new_val = old_val + val;

        sa_data.sum_usage += (new_val - old_val);

        sa_data.sum_squared_usage += (new_val * new_val - old_val * old_val);

        usage_map[x][y] = new_val;
      }
    }
  }
}

void Placer::SwapPosition(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y, Design& design,
                          std::vector<std::vector<int>>& usage_map, SAData& sa_data, std::vector<std::vector<LogicBlock*>>& grid_graph) {
  std::vector<Net*> affected_nets;
  affected_nets.reserve(block1->nets().size() + (block2 ? block2->nets().size() : 0));
  affected_nets.insert(affected_nets.end(), block1->nets().begin(), block1->nets().end());
  if (block2) {
    affected_nets.insert(affected_nets.end(), block2->nets().begin(), block2->nets().end());
  }

  std::sort(affected_nets.begin(), affected_nets.end());
  affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()), affected_nets.end());

  UpdateData(design, usage_map, sa_data, affected_nets, -1);
  int x1 = block1->x();
  int y1 = block1->y();

  int x2 = target_x;
  int y2 = target_y;

  block1->set_x(x2);
  block1->set_y(y2);

  if (block2) {
    block2->set_x(x1);
    block2->set_y(y1);
  }
  grid_graph[x1][y1] = block2;
  grid_graph[x2][y2] = block1;
  UpdateData(design, usage_map, sa_data, affected_nets, +1);
}

double Placer::EstimateInitialTemperature(Design& design, std::vector<std::vector<int>>& usage_map, SAData& sa_data,
                                          std::vector<std::vector<LogicBlock*>>& grid_graph, std::mt19937& rng) {
  int chip_width = design.chip_width();
  int chip_height = design.chip_height();
  const auto& blocks = design.logic_blocks();
  double base_cost = ComputeCost(design, sa_data);
  double sum_pos_delta = 0.0;
  int sample_moves = std::min(200, (int)blocks.size() * 2);
  int count_pos = 0;

  std::uniform_int_distribution<int> dist_block(0, blocks.size() - 1);
  std::uniform_int_distribution<int> dist_w(0, chip_width - 1);
  std::uniform_int_distribution<int> dist_h(0, chip_height - 1);

  for (int i = 0; i < sample_moves; ++i) {
    LogicBlock* block1 = blocks[dist_block(rng)];
    int x1 = block1->x();
    int y1 = block1->y();
    int x2;
    int y2;
    do {
      x2 = dist_w(rng);
      y2 = dist_h(rng);
    } while (x1 == x2 && y1 == y2);

    LogicBlock* block2 = grid_graph[x2][y2];

    SwapPosition(block1, block2, x2, y2, design, usage_map, sa_data, grid_graph);
    double new_cost = ComputeCost(design, sa_data);
    double delta = new_cost - base_cost;

    if (delta > 0) {
      sum_pos_delta += delta;
      count_pos++;
    }

    SwapPosition(block1, block2, x1, y1, design, usage_map, sa_data, grid_graph);
  }

  if (count_pos == 0) return 1.0;

  double avg_pos_delta = sum_pos_delta / count_pos;
  double p0 = 0.8;
  double T0 = -avg_pos_delta / std::log(p0);

  if (T0 <= 0.0) T0 = 1.0;
  return T0;
}

void Placer::RunSA(Design& design) {
  auto sa_start_time = std::chrono::steady_clock::now();
  std::cout << "[SA] Starting SA" << std::endl;
  Placer::InitPlace(design);
  std::random_device rd;
  std::mt19937 rng(rd());

  // SA data
  int round_count = 0;
  int chip_width = design.chip_width();
  int chip_height = design.chip_height();
  std::vector<std::vector<int>> usage_map = design.GetUsageMap();
  std::vector<std::vector<LogicBlock*>> grid_graph = design.GetGridGraph();
  const auto& blocks = design.logic_blocks();
  SAData sa_data;
  InitializeData(design, usage_map, sa_data);
  double current_cost = ComputeCost(design, sa_data);

  // SA Parameters
  double temperature = EstimateInitialTemperature(design, usage_map, sa_data, grid_graph, rng);
  const double min_temperature = 1e-5;
  const double max_region_prob = 0.9;
  const double prob_step = 0.02;
  double current_region_prob = 0.3;
  double alpha = 0.8;
  int moves_per_temperature = 50000;
  std::cout << "[SA] Estimated Initial Temperature: " << temperature << std::endl;
  std::cout << "[SA] Init Cost: " << current_cost << " | HPWL: " << sa_data.total_hpwl << " | CC: " << CalculateCongestionCoefficient(design, sa_data)
            << std::endl;

  std::uniform_int_distribution<int> dist_w(0, chip_width - 1);
  std::uniform_int_distribution<int> dist_h(0, chip_height - 1);
  std::uniform_real_distribution<double> dist_prob(0.0, 1.0);
  std::uniform_int_distribution<int> dist_block_idx(0, blocks.size() - 1);

  while (temperature > min_temperature) {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - sa_start_time;
    if (elapsed_seconds.count() > TIME_LIMIT_SECONDS) {
      std::cout << "\n[SA] Time Limit Reached. Stopping." << std::endl;
      break;
    }

    int accepted_moves = 0;

    for (int i = 0; i < moves_per_temperature; ++i) {
      LogicBlock* block1 = blocks[dist_block_idx(rng)];
      int x1 = block1->x();
      int y1 = block1->y();
      int x2;
      int y2;
      bool use_region = (dist_prob(rng) < current_region_prob);

      OptimalRegion region;
      if (use_region) {
        region = block1->GetOptimalRegion(chip_width, chip_height);
        if (region.upper_x == region.lower_x && region.upper_y == region.lower_y && region.lower_x == x1 && region.lower_y == y1) {
          use_region = false;
        }
      }

      do {
        if (use_region) {
          int width = region.upper_x - region.lower_x + 1;
          int height = region.upper_y - region.lower_y + 1;
          std::uniform_int_distribution<int> dist_region_w(0, width - 1);
          std::uniform_int_distribution<int> dist_region_h(0, height - 1);
          x2 = region.lower_x + dist_region_w(rng);
          y2 = region.lower_y + dist_region_h(rng);
        } else {
          x2 = dist_w(rng);
          y2 = dist_h(rng);
        }
      } while (x1 == x2 && y1 == y2);

      LogicBlock* block2 = grid_graph[x2][y2];

      SwapPosition(block1, block2, x2, y2, design, usage_map, sa_data, grid_graph);

      double new_cost = ComputeCost(design, sa_data);
      double delta_cost = new_cost - current_cost;

      bool accept = false;
      if (delta_cost < 0.0) {
        accept = true;
      } else {
        double r = dist_prob(rng);
        if (r < std::exp(-delta_cost / temperature)) {
          accept = true;
        }
      }

      if (accept) {
        current_cost = new_cost;
        accepted_moves++;
      } else {
        SwapPosition(block1, block2, x1, y1, design, usage_map, sa_data, grid_graph);
      }
    }

    if (current_region_prob < max_region_prob) {
      current_region_prob += prob_step;
    }

    double acceptance_rate = static_cast<double>(accepted_moves) / moves_per_temperature;
    if (acceptance_rate > 0.85)
      alpha = 0.5;
    else if (acceptance_rate > 0.75)
      alpha = 0.92;
    else if (acceptance_rate > 0.15)
      alpha = 0.95;
    else 
      alpha = 0.8;
    temperature *= alpha;

    round_count++;
    std::cout << std::fixed << std::setprecision(4) << "Round " << std::setw(3) << round_count << " | T: " << std::fixed << std::setprecision(5)
              << temperature << " | Acc: " << std::fixed << std::setw(1) << static_cast<int>(acceptance_rate * 100) << "%"
              << " | Cost: " << std::fixed << std::setprecision(7) << current_cost << " | HPWL: " << std::fixed << std::setprecision(6)
              << sa_data.total_hpwl << " | CC: " << std::setprecision(7) << CalculateCongestionCoefficient(design, sa_data) << std::endl;
  }
  std::cout << "[SA] Final Cost: " << current_cost << " (HPWL: " << sa_data.total_hpwl << ", CC: " << CalculateCongestionCoefficient(design, sa_data)
            << ")" << std::endl;
}
