#include "placer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "../design/design.hpp"

void Placer::InitPlace(Design& design) {
  int current_x = 0;
  int current_y = 0;
  int chip_w = design.chip_width();
  int chip_h = design.chip_height();

  for (auto block : design.logic_blocks()) {
    block->set_x(current_x);
    block->set_y(current_y);

    current_x++;
    if (current_x >= chip_w) {
      current_x = 0;
      current_y++;
    }

    if (current_y >= chip_h) {
      std::cerr << "Error: Not enough space on FPGA to place all blocks!"
                << std::endl;
      break;
    }
  }
  std::cout << "Initial placement completed." << std::endl;
}

double Placer::CalculateHPWL(const Design& design) {
  double total_hpwl = 0;

  for (const auto& net : design.nets()) {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    bool has_terminals = false;

    for (auto block : net->blocks()) {
      has_terminals = true;
      double bx = static_cast<double>(block->x());
      double by = static_cast<double>(block->y());

      if (bx < min_x) min_x = bx;
      if ((bx + 1.0) > max_x) max_x = bx + 1.0;
      if (by < min_y) min_y = by;
      if ((by + 1.0) > max_y) max_y = by + 1.0;
    }

    for (auto pin : net->pins()) {
      has_terminals = true;
      double px = pin->x();
      double py = pin->y();

      if (px < min_x) min_x = px;
      if (px > max_x) max_x = px;
      if (py < min_y) min_y = py;
      if (py > max_y) max_y = py;
    }

    if (!has_terminals) continue;

    double hpwl = (max_x - min_x) + (max_y - min_y);
    total_hpwl += hpwl;
  }
  return total_hpwl;
}

double Placer::CalculateCongestionCoefficient(const Design& design) {
  int rows = design.chip_height();
  int cols = design.chip_width();
  long long n_sites = (long long)rows * cols;

  if (n_sites == 0) return 0.0;

  std::vector<std::vector<int>> usage(cols, std::vector<int>(rows, 0));

  for (const auto& net : design.nets()) {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    bool has_terminals = false;
    for (auto block : net->blocks()) {
      has_terminals = true;
      double bx = static_cast<double>(block->x());
      double by = static_cast<double>(block->y());

      if (bx < min_x) min_x = bx;
      if ((bx + 1.0) > max_x) max_x = bx + 1.0;
      if (by < min_y) min_y = by;
      if ((by + 1.0) > max_y) max_y = by + 1.0;
    }

    for (auto pin : net->pins()) {
      has_terminals = true;
      double px = pin->x();
      double py = pin->y();

      if (px < min_x) min_x = px;
      if (px > max_x) max_x = px;
      if (py < min_y) min_y = py;
      if (py > max_y) max_y = py;
    }

    if (!has_terminals) continue;

    int start_x = std::max(0, (int)std::ceil(min_x));
    int start_y = std::max(0, (int)std::ceil(min_y));

    for (int x = start_x; x < max_x && x < cols; ++x) {
      for (int y = start_y; y < max_y && y < rows; ++y) {
        usage[x][y]++;
      }
    }
  }

  double sum_u = 0.0;
  double sum_sq_u = 0.0;

  for (int x = 0; x < cols; ++x) {
    for (int y = 0; y < rows; ++y) {
      double val = (double)usage[x][y];
      sum_u += val;
      sum_sq_u += (val * val);
    }
  }

  double term1 = sum_sq_u / (double)n_sites;
  double mean_u = sum_u / (double)n_sites;
  double term2 = mean_u * mean_u;

  if (term2 == 0.0) return 1.0;

  return term1 / term2;
}

void Placer::Run(Design& design) {
  InitPlace(design);
  double hpwl = CalculateHPWL(design);
  double cc = CalculateCongestionCoefficient(design);
  std::cout << "Initial HPWL: " << hpwl << std::endl;
  std::cout << "Initial Congestion: " << cc << std::endl;
}