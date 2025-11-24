#pragma once
#include <string>
#include <vector>

class LogicBlock;
class IOPin;
class Net;

struct OptimalRegion {
  int lower_x;
  int lower_y;
  int upper_x;
  int upper_y;
};

struct BoundingBox {
  double lower_x;
  double lower_y;
  double upper_x;
  double upper_y;
  bool is_valid;
};

class LogicBlock {
 public:
  explicit LogicBlock(const std::string& name);

  const std::string& name() const { return name_; }
  int x() const { return x_; }
  int y() const { return y_; }
  const std::vector<Net*>& nets() const { return nets_; }

  void set_x(int x) { x_ = x; }
  void set_y(int y) { y_ = y; }
  void AddNet(Net* net);

  OptimalRegion GetOptimalRegion(int chip_width, int chip_height) const;

 private:
  std::string name_;
  int x_;
  int y_;
  std::vector<Net*> nets_;
};

class IOPin {
 public:
  IOPin(const std::string& name, double x, double y);

  const std::string& name() const { return name_; }
  double x() const { return x_; }
  double y() const { return y_; }
  const std::vector<Net*>& nets() const { return nets_; }

  void AddNet(Net* net);

 private:
  std::string name_;
  double x_;
  double y_;
  std::vector<Net*> nets_;
};

class Net {
 public:
  Net(const std::string& name, int degree);

  const std::string& name() const { return name_; }
  const std::vector<LogicBlock*>& blocks() const { return blocks_; }
  const std::vector<IOPin*>& pins() const { return pins_; }

  void AddBlock(LogicBlock* block);
  void AddPin(IOPin* pin);

  BoundingBox ComputeBoundingBox(const LogicBlock* exclude_block = nullptr) const;
  double CalculateHPWL() const;

 private:
  std::string name_;
  int degree_;
  std::vector<LogicBlock*> blocks_;
  std::vector<IOPin*> pins_;
};

class Design {
 public:
  Design();

  int chip_width() const { return chip_width_; }
  int chip_height() const { return chip_height_; }

  void set_chip_dimensions(int width, int height);
  void set_chip_height(int height);
  void set_chip_width(int width);

  void AddLogicBlock(LogicBlock* block);
  void AddIOPin(IOPin* pin);
  void AddNet(Net* net);

  const std::vector<LogicBlock*>& logic_blocks() const { return logic_blocks_; }
  const std::vector<IOPin*>& io_pins() const { return io_pins_; }
  const std::vector<Net*>& nets() const { return nets_; }

  double CalculateTotalHPWL() const;
  std::vector<std::vector<int>> CalculateUsageMap() const;
  double CalculateCongestionCoefficient() const;

 private:
  int chip_width_;
  int chip_height_;

  std::vector<LogicBlock*> logic_blocks_;
  std::vector<IOPin*> io_pins_;
  std::vector<Net*> nets_;
};
