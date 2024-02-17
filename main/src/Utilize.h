#ifndef Utilize_H
#define Utilize_H

#include <cmath>
#include <algorithm>

class Utilize {
  public:
  static bool AtTargetRange(double, double, double);
  static double WrapRads(double);
  static double WrapDegs(double);
  static double NormalizeRads(double);
  static double NormalizeDegs(double);
};

#endif
