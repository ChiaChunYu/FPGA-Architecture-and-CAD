#include "placer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <unordered_map>
#include <vector>

#include "../design/design.hpp"

Placer::Placer(Design& design, const Config& config, std::chrono::steady_clock::time_point start_time)
    : design_(design), config_(config), start_time_(start_time) {
  if (config_.random_seed == 0) {
    std::random_device rd;
    rng_.seed(rd());
  } else {
    rng_.seed(config_.random_seed);
  }
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

void Placer::InitState() {
  usage_map_ = design_.GetUsageMap();
  grid_graph_ = design_.GetGridGraph();

  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();

  long long sum_usage = 0;
  long long sum_squared_usage = 0;

  for (int x = 0; x < chip_width; ++x) {
    for (int y = 0; y < chip_height; ++y) {
      long long v = usage_map_[x][y];
      sum_usage += v;
      sum_squared_usage += v * v;
    }
  }

  state_.total_hpwl = design_.CalcTotalHPWL();
  state_.sum_usage = sum_usage;
  state_.sum_squared_usage = sum_squared_usage;

  for (Net* net : design_.nets()) {
    net->CalcCachedBoundingBox();
  }
}

double Placer::EstimateInitTemperature() {
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();
  const auto& blocks = design_.logic_blocks();

  double base_cost = CalcCost();
  double sum_pos_delta = 0.0;
  int sample_moves = 1000;
  int count_pos = 0;

  if (blocks.empty()) return 1.0;

  std::uniform_int_distribution<int> dist_block(0, blocks.size() - 1);
  std::uniform_int_distribution<int> dist_w(0, chip_width - 1);
  std::uniform_int_distribution<int> dist_h(0, chip_height - 1);

  for (int i = 0; i < sample_moves; ++i) {
    LogicBlock* block1 = blocks[dist_block(rng_)];
    int x1 = block1->x();
    int y1 = block1->y();

    int x2, y2;
    do {
      x2 = dist_w(rng_);
      y2 = dist_h(rng_);
    } while (x1 == x2 && y1 == y2);

    LogicBlock* block2 = grid_graph_[x2][y2];

    SwapPosition(block1, block2, x2, y2);
    double new_cost = CalcCost();
    double delta = new_cost - base_cost;
    if (delta > 0.0) {
      sum_pos_delta += delta;
      count_pos++;
    }
    SwapPosition(block1, block2, x1, y1);
  }

  if (count_pos == 0) return 1.0;

  double avg_pos_delta = sum_pos_delta / static_cast<double>(count_pos);
  double p0 = 0.8;
  double T0 = -avg_pos_delta / std::log(p0);
  if (T0 <= 0.0) T0 = 1.0;
  return T0;
}

double Placer::CongestionCoefficient() const {
  long long num_sites = static_cast<long long>(design_.chip_width()) * static_cast<long long>(design_.chip_height());
  if (num_sites == 0) return 1.0;

  double mean_usage = static_cast<double>(state_.sum_usage) / static_cast<double>(num_sites);
  double mean_sq = static_cast<double>(state_.sum_squared_usage) / static_cast<double>(num_sites);

  if (mean_usage == 0.0) return 1.0;
  return mean_sq / (mean_usage * mean_usage);
}

double Placer::CalcCost() const {
  double congestion_coefficient = CongestionCoefficient();

  const double min_cc = 1.0;
  const double max_cc = 1.6;
  const double min_exp = 1.0;
  const double max_exp = 3.0;

  double t = (congestion_coefficient - min_cc) / (max_cc - min_cc);
  double exponent = min_exp + t * (max_exp - min_exp);
  // if (congestion_coefficient < 1.05) exponent = min_exp;

  return state_.total_hpwl * std::pow(congestion_coefficient, exponent);
}

void Placer::UpdateState(const std::vector<Net*>& nets, int val) {
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();

  for (Net* net : nets) {
    const BoundingBox& bbox = net->cached_bbox();
    if (!bbox.is_valid) continue;

    double hpwl = (bbox.upper_x - bbox.lower_x) + (bbox.upper_y - bbox.lower_y);
    state_.total_hpwl += static_cast<double>(val) * hpwl;

    int start_x = std::max(0, static_cast<int>(std::floor(bbox.lower_x)));
    int end_x = std::min(chip_width, static_cast<int>(std::ceil(bbox.upper_x)));
    int start_y = std::max(0, static_cast<int>(std::floor(bbox.lower_y)));
    int end_y = std::min(chip_height, static_cast<int>(std::ceil(bbox.upper_y)));

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

  int x1_old = block1->x();
  int y1_old = block1->y();
  int x2_old = -1;
  int y2_old = -1;
  if (block2) {
    x2_old = block2->x();
    y2_old = block2->y();
  }

  block1->set_x(target_x);
  block1->set_y(target_y);
  if (block2) {
    block2->set_x(x1_old);
    block2->set_y(y1_old);
  }

  for (Net* net : affected_nets) {
    bool has_b1 = false;
    bool has_b2 = false;
    for (LogicBlock* b : net->blocks()) {
      if (b == block1)
        has_b1 = true;
      else if (block2 && b == block2)
        has_b2 = true;
    }

    if (has_b1 && has_b2) {
      net->CalcCachedBoundingBox();
    } else if (has_b1) {
      net->UpdateCachedBoundingBox(block1, x1_old, y1_old);
    } else if (block2 && has_b2) {
      net->UpdateCachedBoundingBox(block2, x2_old, y2_old);
    }
  }

  grid_graph_[x1_old][y1_old] = block2;
  grid_graph_[target_x][target_y] = block1;

  UpdateState(affected_nets, +1);
}

void Placer::UpdateParameters(double& temperature, double& region_prob, int& moves_per_temperature, double& range_limiter,
                              const double& acceptance_rate) {
  // ---------- temperature ----------
  if (acceptance_rate > 0.95) {
    temperature *= 0.5;
  } else if (acceptance_rate > 0.75) {
    temperature *= 0.92;
  } else if (acceptance_rate > 0.15) {
    temperature *= 0.95;
  } else {
    temperature *= 0.8;
  }
  // ---------- optimal region probability ----------
  if (acceptance_rate > 0.85) {
    region_prob -= config_.region_prob_step_down;
  } else if (acceptance_rate > 0.75) {
    region_prob += config_.region_prob_step_up;
  } else if (acceptance_rate > 0.15) {
    region_prob += config_.region_prob_step_up;
  } else {
    region_prob -= config_.region_prob_step_down;
  }

  if (region_prob < config_.min_region_prob) region_prob = config_.min_region_prob;
  if (region_prob > config_.max_region_prob) region_prob = config_.max_region_prob;

  // ---------- moves per temperature ----------
  if (acceptance_rate > 0.9) {
    int new_moves = static_cast<int>(moves_per_temperature * config_.moves_scale_down);
    if (new_moves < config_.min_moves_per_temp) new_moves = config_.min_moves_per_temp;
    moves_per_temperature = new_moves;
  } else if (acceptance_rate > 0.6) {
    int new_moves = static_cast<int>(moves_per_temperature * config_.moves_scale_up);
    if (new_moves > config_.max_moves_per_temp) new_moves = config_.max_moves_per_temp;
    moves_per_temperature = new_moves;
  } else if (acceptance_rate < 0.15) {
    int new_moves = static_cast<int>(moves_per_temperature * config_.moves_scale_down);
    if (new_moves < config_.min_moves_per_temp) new_moves = config_.min_moves_per_temp;
    moves_per_temperature = new_moves;
  }

  // ---------- range limiter ----------
  if (acceptance_rate > 0.0) {
    double ratio = acceptance_rate / config_.target_acceptance;
    if (ratio > 1.2) ratio = 1.2;
    if (ratio < 0.8) ratio = 0.8;
    range_limiter *= ratio;
  } else {
    range_limiter *= 0.9;
  }

  if (range_limiter < config_.min_range_limiter) range_limiter = config_.min_range_limiter;
  if (range_limiter > config_.max_range_limiter) range_limiter = config_.max_range_limiter;
}

void Placer::Run() {
  InitState();
  int round_count = 0;
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();
  const auto& blocks = design_.logic_blocks();

  double current_cost = CalcCost();
  double temperature = EstimateInitTemperature();
  double current_region_prob = config_.initial_region_prob;
  int moves_per_temperature = config_.initial_moves_per_temp;
  double range_limiter = config_.initial_range_limiter;

  int max_search_radius_x = chip_width - 1;
  int max_search_radius_y = chip_height - 1;
  int min_search_radius_x = 1;
  int min_search_radius_y = 1;

  double best_cost = current_cost;
  int no_improve_rounds = 0;
  const int max_no_improve_rounds = config_.max_no_improve_rounds;
  const double min_improve = config_.min_improve;

  std::uniform_real_distribution<double> dist_prob(0.0, 1.0);
  std::uniform_int_distribution<int> dist_block_idx(0, static_cast<int>(blocks.size()) - 1);

  std::cout << "[SA] Estimated Initial Temperature: " << temperature << std::endl;
  std::cout << "[SA] Init Cost: " << current_cost << " | HPWL: " << state_.total_hpwl << " | CC: " << CongestionCoefficient() << std::endl;

  while (temperature > config_.min_temperature) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;
    if (elapsed.count() > config_.time_limit_seconds) {
      std::cout << "\n[SA] Time Limit Reached (" << elapsed.count() << "s). Stopping." << std::endl;
      break;
    }

    double round_best_cost = current_cost;

    int search_radius_x = std::max(min_search_radius_x, static_cast<int>(max_search_radius_x * range_limiter));
    int search_radius_y = std::max(min_search_radius_y, static_cast<int>(max_search_radius_y * range_limiter));

    if (search_radius_x < min_search_radius_x) search_radius_x = min_search_radius_x;
    if (search_radius_y < min_search_radius_y) search_radius_y = min_search_radius_y;

    int accepted_moves = 0;

    for (int i = 0; i < moves_per_temperature; ++i) {
      LogicBlock* block1 = blocks[dist_block_idx(rng_)];
      int x1 = block1->x();
      int y1 = block1->y();
      int x2, y2;

      bool use_region = (dist_prob(rng_) < current_region_prob);
      OptimalRegion region;

      if (use_region) {
        region = block1->CalcOptimalRegion(chip_width, chip_height);
        if (region.lower_x == region.upper_x && region.lower_y == region.upper_y && region.lower_x == x1 && region.lower_y == y1) {
          use_region = false;
        }
      }

      do {
        int left, right, bottom, top;
        if (use_region) {
          left = region.lower_x;
          right = region.upper_x;
          bottom = region.lower_y;
          top = region.upper_y;
        } else {
          left = std::max(0, x1 - search_radius_x);
          right = std::min(chip_width - 1, x1 + search_radius_x);
          bottom = std::max(0, y1 - search_radius_y);
          top = std::min(chip_height - 1, y1 + search_radius_y);
        }

        std::uniform_int_distribution<int> dist_x(left, right);
        std::uniform_int_distribution<int> dist_y(bottom, top);
        x2 = dist_x(rng_);
        y2 = dist_y(rng_);
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
        if (current_cost < round_best_cost) {
          round_best_cost = current_cost;
        }
      } else {
        SwapPosition(block1, block2, x1, y1);
      }
    }

    double acceptance_rate = static_cast<double>(accepted_moves) / static_cast<double>(moves_per_temperature);
    UpdateParameters(temperature, current_region_prob, moves_per_temperature, range_limiter, acceptance_rate);
    round_count++;

    std::cout << std::fixed << std::setprecision(4) << "Round " << std::setw(3) << round_count << " | T: " << std::setprecision(5) << temperature
              << " | Acc: " << static_cast<int>(acceptance_rate * 100) << "%"
              << " | range: " << std::setprecision(2) << range_limiter << " | Cost: " << std::setprecision(6) << current_cost
              << " | HPWL: " << std::setprecision(6) << state_.total_hpwl << " | CC: " << std::setprecision(6) << CongestionCoefficient()
              << " | moves: " << moves_per_temperature << std::endl;

    double improvement = best_cost - round_best_cost;

    if (abs(improvement) > min_improve) {
      best_cost = round_best_cost;
      no_improve_rounds = 0;
    } else {
      ++no_improve_rounds;
      if (no_improve_rounds >= max_no_improve_rounds) {
        std::cout << "[SA] Early stopping: no significant cost improvement in " << max_no_improve_rounds << " rounds." << std::endl;
        break;
      }
    }
  }
  std::cout << "[SA] Final Cost: " << current_cost << " (HPWL: " << state_.total_hpwl << ", CC: " << CongestionCoefficient() << ")" << std::endl;
}
