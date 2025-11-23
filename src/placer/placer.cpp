#include "placer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <vector>

#include "../design/design.hpp"

const double TIME_LIMIT_SECONDS = 200.0;

double Placer::CalculateHPWL(const Design& design) {
  return design.CalculateTotalHPWL();
}

double Placer::CalculateCongestionCoefficient(const Design& design) {
  return design.CalculateCongestionCoefficient();
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

  int round_count = 0;
  double current_total_hpwl = design.CalculateTotalHPWL();
  double current_congestion_coefficient = design.CalculateCongestionCoefficient();
  double current_cost = current_total_hpwl * current_congestion_coefficient;
  std::cout << "[SA] Init Cost: " << current_cost << " | HPWL: " << current_total_hpwl << " | CC: " << current_congestion_coefficient << std::endl;

  // SA Parameters
  double temperature = current_cost / 20.0;
  double min_temperature = 1e-5;
  int moves_per_temperature = 10 * design.logic_blocks().size();
  // int moves_per_temperature = 15 * std::pow(design.logic_blocks().size(), 1.33);
  // if (moves_per_temperature < 3000) moves_per_temperature = 3000;
  // if (moves_per_temperature > 400000) moves_per_temperature = 400000;

  while (temperature > min_temperature) {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time - sa_start_time;
    if (elapsed_seconds.count() > TIME_LIMIT_SECONDS) {
      std::cout << "\n[SA] Time Limit Reached. Stopping." << std::endl;
      break;
    }

    int accepted_moves = 0;

    for (int i = 0; i < moves_per_temperature; ++i) {
      const auto& blocks = design.logic_blocks();
      LogicBlock* block1 = blocks[std::rand() % blocks.size()];
      int x1 = block1->x();
      int y1 = block1->y();
      int x2, y2;

      double random_val = static_cast<double>(std::rand()) / RAND_MAX;

      if (random_val < 0.5) {
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

      block1->set_x(x2);
      block1->set_y(y2);
      if (block2) {
        block2->set_x(x1);
        block2->set_y(y1);
      }
      grid[x1][y1] = block2;
      grid[x2][y2] = block1;

      double new_total_hpwl = design.CalculateTotalHPWL();
      double new_congestion_coefficient = design.CalculateCongestionCoefficient();
      double new_cost = new_total_hpwl * new_congestion_coefficient;
      double cost_delta = new_cost - current_cost;

      bool accept = false;
      if (cost_delta < 0) {
        accept = true;
      } else {
        double r = static_cast<double>(std::rand()) / RAND_MAX;
        if (r < std::exp(-cost_delta / temperature)) {
          accept = true;
        }
      }

      if (accept) {
        current_cost = new_cost;
        current_total_hpwl = new_total_hpwl;
        current_congestion_coefficient = new_congestion_coefficient;
        accepted_moves++;
      } else {
        // Reject：Restore Previous State
        block1->set_x(x1);
        block1->set_y(y1);
        if (block2) {
          block2->set_x(x2);
          block2->set_y(y2);
        }
        grid[x1][y1] = block1;
        grid[x2][y2] = block2;
      }
    }

    // Update temperature
    double acceptance_rate = (double)accepted_moves / moves_per_temperature;
    double alpha = 0.8;
    if (acceptance_rate > 0.96)
      alpha = 0.7;
    else if (acceptance_rate > 0.8)
      alpha = 0.9;
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