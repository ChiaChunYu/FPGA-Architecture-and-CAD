#pragma once

class Design;

class Placer {
 public:
  Placer() = default;

  void Run(Design& design);

  void InitPlace(Design& design);

  double CalculateHPWL(const Design& design);

  double CalculateCongestionCoefficient(const Design& design);
};