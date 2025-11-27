#include "placer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "../design/design.hpp"

Placer::Placer(Design& design, const Config& config, std::chrono::steady_clock::time_point start_time)
    : design_(design), config_(config), start_time_(start_time) {
  std::random_device rd;
  rng_.seed(rd());
}

void Placer::InitPlace() {
  int current_x = 0;
  int current_y = 0;
  int chip_width = design_.chip_width();

  for (auto block : design_.logic_blocks()) {
    block->set_x(current_x);
    block->set_y(current_y);
    current_x++;
    if (current_x >= chip_width) {
      current_x = 0;
      current_y++;
    }
  }
}

// void Placer::InitPlace() {
//   auto& blocks = design_.logic_blocks();
//   auto& nets = design_.nets();

//   int B = blocks.size();
//   int E = nets.size();
//   int W = design_.chip_width();
//   int H = design_.chip_height();

//   if (B == 0) return;

//   for (auto* b : blocks) {
//     b->set_x(rng_() % W);
//     b->set_y(rng_() % H);
//   }

//   std::vector<double> cx(E, 0.0);
//   std::vector<double> cy(E, 0.0);

//   const int ITER = 5;

//   for (int it = 0; it < ITER; it++) {
//     for (int e = 0; e < E; e++) {
//       Net* net = nets[e];

//       double sx = 0, sy = 0;
//       int count = 0;

//       for (LogicBlock* blk : net->blocks()) {
//         sx += blk->x();
//         sy += blk->y();
//         count++;
//       }

//       for (IOPin* pin : net->pins()) {
//         sx += pin->x();
//         sy += pin->y();
//         count++;
//       }

//       if (count == 0) count = 1;
//       cx[e] = sx / count;
//       cy[e] = sy / count;
//     }

//     for (LogicBlock* blk : blocks) {
//       double sx = 0, sy = 0;
//       int deg = blk->nets().size();

//       if (deg == 0) continue;

//       for (Net* net : blk->nets()) {
//         int idx = 0;
//         for (; idx < E; idx++)
//           if (nets[idx] == net) break;

//         sx += cx[idx];
//         sy += cy[idx];
//       }

//       blk->set_x(sx / deg);
//       blk->set_y(sy / deg);
//     }
//   }

//   std::vector<LogicBlock*> sorted = blocks;
//   std::sort(sorted.begin(), sorted.end(), [](LogicBlock* a, LogicBlock* b) {
//       return (a->x() + a->y()) < (b->x() + b->y());
//   });

//   int idx = 0;
//   for (int y = 0; y < H; y++) {
//     for (int x = 0; x < W; x++) {
//       if (idx >= B) break;
//       sorted[idx]->set_x(x);
//       sorted[idx]->set_y(y);
//       idx++;
//     }
//   }

//   std::cout << "[InitPlace] Analytical placement finished\n";
// }

void Placer::InitState() {
  usage_map_ = design_.GetUsageMap();
  grid_graph_ = design_.GetGridGraph();

  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();
  long long sum_usage = 0;
  long long sum_squared_usage = 0;

  for (int x = 0; x < chip_width; ++x) {
    for (int y = 0; y < chip_height; ++y) {
      long long val = usage_map_[x][y];
      sum_usage += val;
      sum_squared_usage += val * val;
    }
  }

  state_.total_hpwl = design_.CalcTotalHPWL();
  state_.sum_usage = sum_usage;
  state_.sum_squared_usage = sum_squared_usage;
}

double Placer::EstimateInitTemperature() {
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();
  const auto& blocks = design_.logic_blocks();
  double base_cost = CalcCost();
  double sum_pos_delta = 0.0;
  int sample_moves = 100;
  int count_pos = 0;

  std::uniform_int_distribution<int> dist_block(0, blocks.size() - 1);
  std::uniform_int_distribution<int> dist_w(0, chip_width - 1);
  std::uniform_int_distribution<int> dist_h(0, chip_height - 1);

  for (int i = 0; i < sample_moves; ++i) {
    LogicBlock* block1 = blocks[dist_block(rng_)];
    int x1 = block1->x();
    int y1 = block1->y();
    int x2;
    int y2;
    do {
      x2 = dist_w(rng_);
      y2 = dist_h(rng_);
    } while (x1 == x2 && y1 == y2);

    LogicBlock* block2 = grid_graph_[x2][y2];

    SwapPosition(block1, block2, x2, y2);
    double new_cost = CalcCost();
    double delta = new_cost - base_cost;

    if (delta > 0) {
      sum_pos_delta += delta;
      count_pos++;
    }

    SwapPosition(block1, block2, x1, y1);
  }

  if (count_pos == 0) return 1.0;

  double avg_pos_delta = sum_pos_delta / count_pos;
  double p0 = 0.8;
  double T0 = -avg_pos_delta / std::log(p0);

  if (T0 <= 0.0) T0 = 1.0;
  return T0;
}

double Placer::CongestionCoefficient() const {
  long long num_sites = static_cast<long long>(design_.chip_width()) * static_cast<long long>(design_.chip_height());
  double mean_usage = state_.sum_usage / static_cast<double>(num_sites);
  double mean_sq = state_.sum_squared_usage / static_cast<double>(num_sites);

  if (mean_usage == 0.0) {
    return 1.0;
  }

  return mean_sq / (mean_usage * mean_usage);
}

double Placer::CalcCost() const {
  double congestion_coefficient = CongestionCoefficient();
  return state_.total_hpwl * congestion_coefficient;
}

void Placer::UpdateState(const std::vector<Net*>& nets, int val) {
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();

  for (auto net : nets) {
    BoundingBox boundingbox = net->CalcBoundingBox(nullptr);
    if (!boundingbox.is_valid) continue;

    double hpwl = (boundingbox.upper_x - boundingbox.lower_x) + (boundingbox.upper_y - boundingbox.lower_y);
    state_.total_hpwl += val * hpwl;

    int start_x = std::max(0, static_cast<int>(std::floor(boundingbox.lower_x)));
    int end_x = std::min(chip_width, static_cast<int>(std::ceil(boundingbox.upper_x)));
    int start_y = std::max(0, static_cast<int>(std::floor(boundingbox.lower_y)));
    int end_y = std::min(chip_height, static_cast<int>(std::ceil(boundingbox.upper_y)));

    for (int x = start_x; x < end_x; ++x) {
      for (int y = start_y; y < end_y; ++y) {
        long long old_val = usage_map_[x][y];
        long long new_val = old_val + val;

        state_.sum_usage += (new_val - old_val);
        state_.sum_squared_usage += (new_val * new_val - old_val * old_val);

        usage_map_[x][y] = new_val;
      }
    }
  }
}

void Placer::SwapPosition(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y) {
  std::vector<Net*> affected_nets;
  affected_nets.reserve(block1->nets().size() + (block2 ? block2->nets().size() : 0));
  affected_nets.insert(affected_nets.end(), block1->nets().begin(), block1->nets().end());
  if (block2) {
    affected_nets.insert(affected_nets.end(), block2->nets().begin(), block2->nets().end());
  }
  std::sort(affected_nets.begin(), affected_nets.end());
  affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()), affected_nets.end());

  UpdateState(affected_nets, -1);

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

  grid_graph_[x1][y1] = block2;
  grid_graph_[x2][y2] = block1;

  UpdateState(affected_nets, +1);
}

void Placer::UpdateParameters(double& temperature, double& region_prob, int& moves_per_temperature, const double& acceptance_rate) {
  double alpha;
  if (acceptance_rate > 0.85)
    alpha = 0.5;
  else if (acceptance_rate > 0.75)
    alpha = 0.92;
  else if (acceptance_rate > 0.15)
    alpha = 0.95;
  else
    alpha = 0.8;

  temperature *= alpha;

  if (region_prob < config_.max_region_prob) {
    region_prob += config_.region_prob_step;
  }
}

void Placer::Run() {
  std::cout << "[SA] Starting SA" << std::endl;
  InitPlace();
  InitState();

  int round_count = 0;
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();
  const auto& blocks = design_.logic_blocks();

  // Simulated Annealing Parameters
  double current_cost = CalcCost();
  double temperature = EstimateInitTemperature();
  double current_region_prob = config_.initial_region_prob;
  int moves_per_temperature = config_.initial_moves;

  std::uniform_int_distribution<int> dist_w(0, chip_width - 1);
  std::uniform_int_distribution<int> dist_h(0, chip_height - 1);
  std::uniform_real_distribution<double> dist_prob(0.0, 1.0);
  std::uniform_int_distribution<int> dist_block_idx(0, blocks.size() - 1);

  std::cout << "[SA] Estimated Initial Temperature: " << temperature << std::endl;
  std::cout << "[SA] Init Cost: " << current_cost << " | HPWL: " << state_.total_hpwl << " | CC: " << CongestionCoefficient() << std::endl;

  while (temperature > config_.min_temperature) {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - start_time_;
    if (elapsed_seconds.count() > config_.time_limit_seconds) {
      std::cout << "\n[SA] Time Limit Reached (" << elapsed_seconds.count() << "s). Stopping." << std::endl;
      break;
    }

    int accepted_moves = 0;

    for (int i = 0; i < moves_per_temperature; ++i) {
      LogicBlock* block1 = blocks[dist_block_idx(rng_)];
      int x1 = block1->x();
      int y1 = block1->y();
      int x2;
      int y2;
      bool use_region = (dist_prob(rng_) < current_region_prob);

      OptimalRegion region;
      if (use_region) {
        region = block1->CalcOptimalRegion(chip_width, chip_height);
        if (region.upper_x == region.lower_x && region.upper_y == region.lower_y && region.lower_x == x1 && region.lower_y == y1) {
          use_region = false;
        }
      }

      do {
        if (use_region) {
          // int width = region.upper_x - region.lower_x + 1;
          // int height = region.upper_y - region.lower_y + 1;
          // x2 = region.lower_x + (rng_() % width);
          // y2 = region.lower_y + (rng_() % height);
          int width = region.upper_x - region.lower_x + 1;
          int height = region.upper_y - region.lower_y + 1;
          std::uniform_int_distribution<int> dist_region_w(0, width - 1);
          std::uniform_int_distribution<int> dist_region_h(0, height - 1);
          x2 = region.lower_x + dist_region_w(rng_);
          y2 = region.lower_y + dist_region_h(rng_);
        } else {
          x2 = dist_w(rng_);
          y2 = dist_h(rng_);
        }
      } while (x1 == x2 && y1 == y2);

      LogicBlock* block2 = grid_graph_[x2][y2];

      SwapPosition(block1, block2, x2, y2);

      double new_cost = CalcCost();
      double delta_cost = new_cost - current_cost;

      bool accept = false;
      if (delta_cost <= 0.0) {
        accept = true;
      } else {
        double r = dist_prob(rng_);
        if (r < std::exp(-delta_cost / temperature)) {
          accept = true;
        }
      }

      if (accept) {
        current_cost = new_cost;
        accepted_moves++;
      } else {
        SwapPosition(block1, block2, x1, y1);
      }
    }

    double acceptance_rate = static_cast<double>(accepted_moves) / static_cast<double>(moves_per_temperature);
    UpdateParameters(temperature, current_region_prob, moves_per_temperature, acceptance_rate);
    round_count++;

    std::cout << std::fixed << std::setprecision(4) << "Round " << std::setw(3) << round_count << " | T: " << std::fixed << std::setprecision(5)
              << temperature << " | Acc: " << std::fixed << std::setw(1) << static_cast<int>(acceptance_rate * 100) << "%"
              << " | Cost: " << std::fixed << std::setprecision(7) << current_cost << " | HPWL: " << std::fixed << std::setprecision(6)
              << state_.total_hpwl << " | CC: " << std::setprecision(7) << CongestionCoefficient() << std::endl;
  }
  std::cout << "[SA] Final Cost: " << current_cost << " (HPWL: " << state_.total_hpwl << ", CC: " << CongestionCoefficient() << ")" << std::endl;
}