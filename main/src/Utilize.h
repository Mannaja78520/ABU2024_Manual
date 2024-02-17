#ifndef Utilize_H
#define Utilize_H

class Utilize {
  public:
  static boolean AtTargetRange(double, double, double);
  static double WrapRads(double);
  static double WrapDegs(double);
  static double NormalizeRads(double);
  static double NormalizeDegs(double);
  static byte SigNum(double);
}

#endif
