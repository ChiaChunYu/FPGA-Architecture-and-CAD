#pragma once
#include <string>
#include <vector>

class LogicBlock;
class IOPin;
class Net;

struct OptimalRegion {
  int lower_x = 0;
  int lower_y = 0;
  int upper_x = 0;
  int upper_y = 0;
};

struct BoundingBox {
  double lower_x = 0.0;
  double lower_y = 0.0;
  double upper_x = 0.0;
  double upper_y = 0.0;
  bool is_valid = false;
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

  OptimalRegion CalcOptimalRegion(int chip_width, int chip_height) const;

  std::pair<int,int> CalcCenter(int chip_width, int chip_height) const;

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

 private:
  std::string name_;
  double x_;
  double y_;
};

class Net {
 public:
  Net(const std::string& name, int degree);

  const std::string& name() const { return name_; }
  const std::vector<LogicBlock*>& blocks() const { return blocks_; }
  const std::vector<IOPin*>& pins() const { return pins_; }
  const BoundingBox& cached_bbox() const { return cached_bbox_; }

  void AddBlock(LogicBlock* block);
  void AddPin(IOPin* pin);

  BoundingBox CalcBoundingBox(const LogicBlock* exclude_block = nullptr) const;
  double CalcHPWL() const;

  void CalcCachedBoundingBox();
  void UpdateCachedBoundingBox(const LogicBlock* block, int old_x, int old_y);

 private:
  std::string name_;
  int degree_;
  std::vector<LogicBlock*> blocks_;
  std::vector<IOPin*> pins_;

  BoundingBox cached_bbox_;
  int min_x_count_ = 0;
  int max_x_count_ = 0;
  int min_y_count_ = 0;
  int max_y_count_ = 0;
};

class Design {
 public:
  Design();

  int chip_width() const { return chip_width_; }
  int chip_height() const { return chip_height_; }

  void set_chip_dimensions(int width, int height);

  void AddLogicBlock(LogicBlock* block);
  void AddIOPin(IOPin* pin);
  void AddNet(Net* net);

  const std::vector<LogicBlock*>& logic_blocks() const { return logic_blocks_; }
  const std::vector<IOPin*>& io_pins() const { return io_pins_; }
  const std::vector<Net*>& nets() const { return nets_; }

  double CalcTotalHPWL() const;

  std::vector<std::vector<int>> GetUsageMap() const;
  std::vector<std::vector<LogicBlock*>> GetGridGraph() const;

 private:
  int chip_width_;
  int chip_height_;

  std::vector<LogicBlock*> logic_blocks_;
  std::vector<IOPin*> io_pins_;
  std::vector<Net*> nets_;
};
