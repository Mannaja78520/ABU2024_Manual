#include "Utilize.h"

bool Utilize::AtTargetRange(double number, double target, double range) {
  return target - range < number && number < target + range;
}

double Utilize::WrapRads(double rads) {
  if (rads >  M_PI) return rads - (2.0 * M_PI);
  if (rads < -M_PI) return rads + (2.0 * M_PI);
  return rads;
}

double Utilize::WrapDegs(double degs) {
  if (degs >  180) return degs - 360;
  return degs;
}

double Utilize::NormalizeRads(double rads) {
  rads = (rads < -2.0 * M_PI) ? rads + (2.0 * M_PI) :
         (rads >  2.0 * M_PI) ? rads - (2.0 * M_PI) : rads;
  return rads;
}

double Utilize::NormalizeDegs(double degs) {
  degs = (degs < -360) ? degs + 360 : 
         (degs >  360) ? degs - 360 : degs;
  return degs;
}
