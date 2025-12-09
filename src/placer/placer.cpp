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
  if (config_.random_seed == 0) {
    std::random_device rd;
    rng_.seed(rd());
  } else {
    rng_.seed(config_.random_seed);
  }
}

// Initial random placement of logic blocks
void Placer::InitPlace() {
  auto& logic_blocks = design_.logic_blocks();
  int size = static_cast<int>(logic_blocks.size());

  int width = design_.chip_width();
  int height = design_.chip_height();
  int capacity = width * height;

  if (size > capacity) {
    throw std::runtime_error("Error: Not enough chip area for all logic blocks.");
  }

  std::vector<std::pair<int, int>> positions;
  positions.reserve(capacity);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      positions.emplace_back(x, y);
    }
  }

  std::shuffle(positions.begin(), positions.end(), rng_);

  for (int i = 0; i < size; ++i) {
    int x = positions[i].first;
    int y = positions[i].second;
    logic_blocks[i]->set_x(x);
    logic_blocks[i]->set_y(y);
  }
}

void Placer::Run() {
  InitState();

  int round_count = 0;
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();
  const auto& blocks = design_.logic_blocks();

  double current_cost = CalcCost();
  double temperature = EstimateInitTemperature();
  double initial_temperature = temperature;

  double current_region_prob = config_.initial_region_prob;
  int moves_per_temperature = config_.initial_moves_per_temp;
  double current_range_limiter = config_.initial_range_limiter;

  std::uniform_real_distribution<double> dist_prob(0.0, 1.0);
  std::uniform_int_distribution<int> dist_block_idx(0, static_cast<int>(blocks.size()) - 1);

  std::cout << "[SA] Estimated Initial Temperature: " << temperature << std::endl;
  std::cout << "[SA] Init Cost: " << current_cost << " | HPWL: " << state_.total_hpwl << " | CC: " << CongestionCoefficient() << std::endl;

  while (temperature > config_.min_temperature) {
    auto round_start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;

    if (elapsed.count() > config_.time_limit_seconds) {
      std::cout << "[SA] Time Limit Reached (" << elapsed.count() << "s). Stopping." << std::endl;
      break;
    }

    double elapsed_sec = elapsed.count();
    double time_ratio = elapsed_sec / config_.time_limit_seconds;

    int search_radius_x = std::max(1, static_cast<int>(std::round(current_range_limiter * chip_width)));
    int search_radius_y = std::max(1, static_cast<int>(std::round(current_range_limiter * chip_height)));
    int accepted_moves = 0;

    for (int i = 0; i < moves_per_temperature; ++i) {
      LogicBlock* block1 = blocks[dist_block_idx(rng_)];
      int x1 = block1->x();
      int y1 = block1->y();

      int x2, y2;

      bool use_region = (dist_prob(rng_) < current_region_prob);
      int region_left, region_right, region_bottom, region_top;

      if (use_region) {
        if (config_.use_optimal_region_calc) {
          OptimalRegion region = block1->CalcOptimalRegion(chip_width, chip_height);
          region_left = region.lower_x;
          region_right = region.upper_x;
          region_bottom = region.lower_y;
          region_top = region.upper_y;
        } else {
          std::pair<int, int> center = block1->CalcCenter(chip_width, chip_height);
          region_left = std::max(0, center.first - search_radius_x);
          region_right = std::min(chip_width - 1, center.first + search_radius_x);
          region_bottom = std::max(0, center.second - search_radius_y);
          region_top = std::min(chip_height - 1, center.second + search_radius_y);
        }
        if (region_left == region_right && region_bottom == region_top && region_left == x1 && region_bottom == y1) {
          use_region = false;
        }
      }

      do {
        int left, right, bottom, top;
        if (use_region) {
          left = region_left;
          right = region_right;
          bottom = region_bottom;
          top = region_top;
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

      LogicBlock* block2 = grid_graph_[y2 * chip_width + x2];

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
        ++accepted_moves;
      } else {
        SwapPosition(block1, block2, x1, y1);
      }
    }

    double acceptance_rate = (moves_per_temperature > 0) ? static_cast<double>(accepted_moves) / static_cast<double>(moves_per_temperature) : 0.0;
    ++round_count;
    auto round_end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> round_diff = round_end_time - round_start_time;
    std::cout << std::fixed << std::setprecision(2) << "[SA] Round " << round_count << " completed in " << round_diff.count() << " seconds."
              << std::endl;

    std::cout << std::fixed << "Round " << round_count << " | T: " << std::setprecision(5) << temperature
              << " | Acc: " << static_cast<int>(acceptance_rate * 100) << "%"
              << " | Range: " << std::setprecision(3) << current_range_limiter << " | Cost: " << std::setprecision(6) << current_cost
              << " | HPWL: " << std::setprecision(6) << state_.total_hpwl << " | CC: " << std::setprecision(6) << CongestionCoefficient()
              << " | moves: " << moves_per_temperature << std::endl;

    UpdateParameters(temperature, current_region_prob, moves_per_temperature, current_range_limiter, initial_temperature, acceptance_rate);
  }
  std::cout << "[SA] Final Cost: " << current_cost << " (HPWL: " << state_.total_hpwl << ", CC: " << CongestionCoefficient() << ")" << std::endl;
}

void Placer::InitState() {
  usage_map_ = design_.GetUsageMap();
  grid_graph_ = design_.GetGridGraph();

  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();

  long long sum_usage = 0;
  long long sum_squared_usage = 0;

  int total_cells = chip_width * chip_height;
  for (int i = 0; i < total_cells; ++i) {
    long long v = usage_map_[i];
    sum_usage += v;
    sum_squared_usage += v * v;
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

    LogicBlock* block2 = grid_graph_[y2 * chip_width + x2];

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
  double p0 = 0.9;
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

  return state_.total_hpwl * congestion_coefficient;
}

void Placer::UpdateState(const std::vector<Net*>& nets, int val) {
  int chip_width = design_.chip_width();
  int chip_height = design_.chip_height();

  for (Net* net : nets) {
    const BoundingBox& boundingbox = net->cached_bbox();
    if (!boundingbox.is_valid) continue;

    double hpwl = (boundingbox.upper_x - boundingbox.lower_x) + (boundingbox.upper_y - boundingbox.lower_y);
    state_.total_hpwl += static_cast<double>(val) * hpwl;

    int start_x = std::max(0, static_cast<int>(std::floor(boundingbox.lower_x)));
    int end_x = std::min(chip_width, static_cast<int>(std::ceil(boundingbox.upper_x)));
    int start_y = std::max(0, static_cast<int>(std::floor(boundingbox.lower_y)));
    int end_y = std::min(chip_height, static_cast<int>(std::ceil(boundingbox.upper_y)));

    for (int y = start_y; y < end_y; ++y) {
      int row_offset = y * chip_width;
      for (int x = start_x; x < end_x; ++x) {
        int idx = row_offset + x;

        long long old_val = usage_map_[idx];
        long long new_val = old_val + val;

        state_.sum_usage += (new_val - old_val);
        state_.sum_squared_usage += (new_val * new_val - old_val * old_val);

        usage_map_[idx] = new_val;
      }
    }
  }
}

void Placer::SwapPosition(LogicBlock* block1, LogicBlock* block2, int target_x, int target_y) {
  const auto& nets1 = block1->nets();
  const std::vector<Net*>& nets2 = block2 ? block2->nets() : std::vector<Net*>{};
  std::vector<AffectedNetInfo> net_infos;
  net_infos.reserve(nets1.size() + nets2.size());

  for (Net* net : nets1) {
    net_infos.push_back({net, true, false});
  }

  if (block2) {
    for (Net* net : nets2) {
      bool found = false;
      for (auto& info : net_infos) {
        if (info.net == net) {
          info.has_b2 = true;
          found = true;
          break;
        }
      }
      if (!found) {
        net_infos.push_back({net, false, true});
      }
    }
  }

  std::vector<Net*> affected_nets;
  affected_nets.reserve(net_infos.size());
  for (const auto& info : net_infos) {
    affected_nets.push_back(info.net);
  }

  UpdateState(affected_nets, -1);

  int x1_old = block1->x();
  int y1_old = block1->y();
  int chip_width = design_.chip_width();

  block1->set_x(target_x);
  block1->set_y(target_y);
  if (block2) {
    block2->set_x(x1_old);
    block2->set_y(y1_old);
  }

  for (auto& info : net_infos) {
    Net* net = info.net;

    if (info.has_b1 && info.has_b2) {
      net->CalcCachedBoundingBox();
    } else if (info.has_b1) {
      net->UpdateCachedBoundingBox(block1, x1_old, y1_old);
    } else if (block2 && info.has_b2) {
      net->UpdateCachedBoundingBox(block2, target_x, target_y);
    }
  }

  grid_graph_[y1_old * chip_width + x1_old] = block2;
  grid_graph_[target_y * chip_width + target_x] = block1;

  UpdateState(affected_nets, +1);
}

void Placer::UpdateParameters(double& temperature, double& region_prob, int& moves_per_temperature, double& range_limiter, double initial_temperature,
                              const double& acceptance_rate) {
  // ------------- Moves Per Temperature -------------
  if (acceptance_rate > 0.85 || acceptance_rate < 0.05) {
    int new_moves = static_cast<int>(moves_per_temperature * config_.moves_scale_down);
    moves_per_temperature = std::max(new_moves, config_.min_moves_per_temp);
  } else {
    int new_moves = static_cast<int>(moves_per_temperature * config_.moves_scale_up);
    moves_per_temperature = std::min(new_moves, config_.max_moves_per_temp);
  }

  // ------------- Optimal Region Probability -------------
  if (acceptance_rate > 0.85 || acceptance_rate < 0.10) {
    region_prob -= config_.region_prob_step_down;
  } else {
    region_prob += config_.region_prob_step_down;
  }

  region_prob = std::max(config_.min_region_prob, std::min(region_prob, config_.max_region_prob));

  // ------------- Temperature -------------
  double alpha;
  if (acceptance_rate > 0.96) {
    alpha = 0.5;
  } else if (acceptance_rate > 0.8) {
    alpha = 0.85;
  } else if (acceptance_rate > 0.15) {
    alpha = 0.9;
  } else {
    alpha = 0.85;
  }

  temperature *= alpha;

  //  ------------- Range Limiter -------------
  // double scale_factor = temperature / initial_temperature;
  // double new_range_limiter = std::sqrt(std::max(scale_factor, 1e-8));
  // range_limiter = std::max(0.01, std::min(new_range_limiter, 1.0));
  const double beta = 0.35;
  double scale_factor = temperature / initial_temperature;
  if (scale_factor < 1e-8) scale_factor = 1e-8;

  double new_range_limiter = std::pow(scale_factor, beta);

  if (new_range_limiter < config_.min_range_limiter) new_range_limiter = config_.min_range_limiter;
  if (new_range_limiter > config_.max_range_limiter) new_range_limiter = config_.max_range_limiter;
  range_limiter = new_range_limiter;
}
