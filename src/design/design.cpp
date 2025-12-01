#include "design.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// ------------------------------ LogicBlock Implementation------------------------

LogicBlock::LogicBlock(const std::string& name) : name_(name), x_(0), y_(0) {}

void LogicBlock::AddNet(Net* net) {
  nets_.push_back(net);
}

OptimalRegion LogicBlock::CalcOptimalRegion(int chip_width, int chip_height) const {
  if (nets_.empty()) {
    return {0, 0, chip_width - 1, chip_height - 1};
  }

  std::vector<double> buf_x;
  std::vector<double> buf_y;
  buf_x.reserve(nets_.size() * 2);
  buf_y.reserve(nets_.size() * 2);

  for (auto net : nets_) {
    BoundingBox bounding_box = net->CalcBoundingBox(this);

    if (bounding_box.is_valid) {
      buf_x.push_back(bounding_box.lower_x);
      buf_x.push_back(bounding_box.upper_x);
      buf_y.push_back(bounding_box.lower_y);
      buf_y.push_back(bounding_box.upper_y);
    }
  }

  if (buf_x.empty()) {
    return {0, 0, chip_width - 1, chip_height - 1};
  }

  size_t n = buf_x.size();
  size_t mid_lower = n / 2 - 1;
  size_t mid_upper = n / 2;

  std::nth_element(buf_x.begin(), buf_x.begin() + mid_upper, buf_x.end());
  double raw_ux = buf_x[mid_upper];
  // std::nth_element(buf_x.begin(), buf_x.begin() + mid_lower, buf_x.begin() + mid_upper);
  std::nth_element(buf_x.begin(), buf_x.begin() + mid_lower, buf_x.end());
  double raw_lx = buf_x[mid_lower];

  std::nth_element(buf_y.begin(), buf_y.begin() + mid_upper, buf_y.end());
  double raw_uy = buf_y[mid_upper];
  // std::nth_element(buf_y.begin(), buf_y.begin() + mid_lower, buf_y.begin() + mid_upper);
  std::nth_element(buf_y.begin(), buf_y.begin() + mid_lower, buf_y.end());
  double raw_ly = buf_y[mid_lower];

  int lower_x = std::max(0, std::min(chip_width - 1, static_cast<int>(std::floor(raw_lx))));
  int upper_x = std::max(0, std::min(chip_width - 1, static_cast<int>(std::ceil(raw_ux))));
  int lower_y = std::max(0, std::min(chip_height - 1, static_cast<int>(std::floor(raw_ly))));
  int upper_y = std::max(0, std::min(chip_height - 1, static_cast<int>(std::ceil(raw_uy))));

  if (lower_x > upper_x) upper_x = lower_x;
  if (lower_y > upper_y) upper_y = lower_y;

  return {lower_x, lower_y, upper_x, upper_y};
}

// ------------------------------ IOPin Implementation-----------------------------

IOPin::IOPin(const std::string& name, double x, double y) : name_(name), x_(x), y_(y) {}

// ------------------------------ Net Implementation-------------------------------

Net::Net(const std::string& name, int degree) : name_(name), degree_(degree) {
  blocks_.reserve(degree);
  pins_.reserve(degree);
}

void Net::AddBlock(LogicBlock* block) {
  blocks_.push_back(block);
}

void Net::AddPin(IOPin* pin) {
  pins_.push_back(pin);
}

BoundingBox Net::CalcBoundingBox(const LogicBlock* exclude_block) const {
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  bool has_terminals = false;

  for (auto block : blocks_) {
    if (block == exclude_block) continue;

    has_terminals = true;
    double block_x = static_cast<double>(block->x());
    double block_y = static_cast<double>(block->y());

    if (block_x < min_x) min_x = block_x;
    if (block_y < min_y) min_y = block_y;
    if ((block_x + 1.0) > max_x) max_x = block_x + 1.0;
    if ((block_y + 1.0) > max_y) max_y = block_y + 1.0;
  }

  for (auto pin : pins_) {
    has_terminals = true;
    double pin_x = pin->x();
    double pin_y = pin->y();

    if (pin_x < min_x) min_x = pin_x;
    if (pin_x > max_x) max_x = pin_x;
    if (pin_y < min_y) min_y = pin_y;
    if (pin_y > max_y) max_y = pin_y;
  }

  if (!has_terminals) return {0.0, 0.0, 0.0, 0.0, false};
  return {min_x, min_y, max_x, max_y, true};
}

double Net::CalcHPWL() const {
  BoundingBox bounding_box = CalcBoundingBox(nullptr);

  if (!bounding_box.is_valid) return 0.0;

  return (bounding_box.upper_x - bounding_box.lower_x) + (bounding_box.upper_y - bounding_box.lower_y);
}

void Net::CalcCachedBoundingBox() {
  BoundingBox boundingbox = CalcBoundingBox(nullptr);
  cached_bbox_ = boundingbox;

  if (!boundingbox.is_valid) {
    min_x_count_ = max_x_count_ = min_y_count_ = max_y_count_ = 0;
    return;
  }

  double lower_x = boundingbox.lower_x;
  double upper_x = boundingbox.upper_x;
  double lower_y = boundingbox.lower_y;
  double upper_y = boundingbox.upper_y;

  int min_x_cnt = 0, max_x_cnt = 0;
  int min_y_cnt = 0, max_y_cnt = 0;

  for (LogicBlock* block : blocks_) {
    double x_lower = static_cast<double>(block->x());
    double y_lower = static_cast<double>(block->y());
    double x_upper = x_lower + 1.0;
    double y_upper = y_lower + 1.0;

    if (x_lower == lower_x) ++min_x_cnt;
    if (x_upper == upper_x) ++max_x_cnt;
    if (y_lower == lower_y) ++min_y_cnt;
    if (y_upper == upper_y) ++max_y_cnt;
  }

  for (IOPin* pin : pins_) {
    double x = pin->x();
    double y = pin->y();
    if (x == lower_x) ++min_x_cnt;
    if (x == upper_x) ++max_x_cnt;
    if (y == lower_y) ++min_y_cnt;
    if (y == upper_y) ++max_y_cnt;
  }

  min_x_count_ = min_x_cnt;
  max_x_count_ = max_x_cnt;
  min_y_count_ = min_y_cnt;
  max_y_count_ = max_y_cnt;
}

void Net::UpdateCachedBoundingBox(const LogicBlock* block, int old_x, int old_y) {
  if (!cached_bbox_.is_valid) {
    CalcCachedBoundingBox();
    return;
  }

  double lower_x = cached_bbox_.lower_x;
  double upper_x = cached_bbox_.upper_x;
  double lower_y = cached_bbox_.lower_y;
  double upper_y = cached_bbox_.upper_y;

  bool need_recompute = false;

  double old_x_lower = static_cast<double>(old_x);
  double old_y_lower = static_cast<double>(old_y);
  double old_x_upper = old_x_lower + 1.0;
  double old_y_upper = old_y_lower + 1.0;

  if (old_x_lower == lower_x) {
    if (--min_x_count_ == 0) need_recompute = true;
  }
  if (old_x_upper == upper_x) {
    if (--max_x_count_ == 0) need_recompute = true;
  }
  if (old_y_lower == lower_y) {
    if (--min_y_count_ == 0) need_recompute = true;
  }
  if (old_y_upper == upper_y) {
    if (--max_y_count_ == 0) need_recompute = true;
  }

  if (need_recompute) {
    CalcCachedBoundingBox();
    return;
  }

  double new_x_lower = static_cast<double>(block->x());
  double new_y_lower = static_cast<double>(block->y());
  double new_x_upper = new_x_lower + 1.0;
  double new_y_upper = new_y_lower + 1.0;

  if (new_x_lower < cached_bbox_.lower_x) {
    cached_bbox_.lower_x = new_x_lower;
    min_x_count_ = 1;
  } else if (new_x_lower == cached_bbox_.lower_x) {
    ++min_x_count_;
  }

  if (new_x_upper > cached_bbox_.upper_x) {
    cached_bbox_.upper_x = new_x_upper;
    max_x_count_ = 1;
  } else if (new_x_upper == cached_bbox_.upper_x) {
    ++max_x_count_;
  }

  if (new_y_lower < cached_bbox_.lower_y) {
    cached_bbox_.lower_y = new_y_lower;
    min_y_count_ = 1;
  } else if (new_y_lower == cached_bbox_.lower_y) {
    ++min_y_count_;
  }

  if (new_y_upper > cached_bbox_.upper_y) {
    cached_bbox_.upper_y = new_y_upper;
    max_y_count_ = 1;
  } else if (new_y_upper == cached_bbox_.upper_y) {
    ++max_y_count_;
  }
}

// ------------------------------ Design Implementation---------------------------- //

Design::Design() : chip_width_(0), chip_height_(0) {}

void Design::set_chip_dimensions(int width, int height) {
  chip_width_ = width;
  chip_height_ = height;
}

void Design::AddLogicBlock(LogicBlock* block) {
  logic_blocks_.push_back(block);
}

void Design::AddIOPin(IOPin* pin) {
  io_pins_.push_back(pin);
}

void Design::AddNet(Net* net) {
  nets_.push_back(net);
}

double Design::CalcTotalHPWL() const {
  double total_hpwl = 0.0;
  for (const auto& net : nets_) {
    total_hpwl += net->CalcHPWL();
  }
  return total_hpwl;
}

std::vector<std::vector<int>> Design::GetUsageMap() const {
  std::vector<std::vector<int>> usage_map(chip_width_, std::vector<int>(chip_height_, 0));

  for (const auto& net : nets_) {
    BoundingBox boundingbox = net->CalcBoundingBox(nullptr);
    if (!boundingbox.is_valid) continue;

    int start_x = std::max(0, static_cast<int>(std::floor(boundingbox.lower_x)));
    int end_x = std::min(chip_width_, static_cast<int>(std::ceil(boundingbox.upper_x)));
    int start_y = std::max(0, static_cast<int>(std::floor(boundingbox.lower_y)));
    int end_y = std::min(chip_height_, static_cast<int>(std::ceil(boundingbox.upper_y)));

    for (int x = start_x; x < end_x; ++x) {
      for (int y = start_y; y < end_y; ++y) {
        usage_map[x][y]++;
      }
    }
  }

  return usage_map;
}

std::vector<std::vector<LogicBlock*>> Design::GetGridGraph() const {
  std::vector<std::vector<LogicBlock*>> grid_graph(chip_width_, std::vector<LogicBlock*>(chip_height_, nullptr));

  for (size_t i = 0; i < logic_blocks_.size(); ++i) {
    const auto& block = logic_blocks_[i];
    int x = block->x();
    int y = block->y();
    if (x >= 0 && x < chip_width_ && y >= 0 && y < chip_height_) {
      grid_graph[x][y] = block;
    }
  }

  return grid_graph;
}
