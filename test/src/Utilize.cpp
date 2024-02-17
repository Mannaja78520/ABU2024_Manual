#include "Utilize.h"
#include "Robot.h"

bool Utilize::AtTargetRange(double number, double target, double range) {
  return target - range < number && number < target + range;
}

double Utilize::WrapRads(double rads) {
  if (rads >  PI) return rads - (2.0 * PI);
  if (rads < -PI) return rads + (2.0 * PI);
  return rads;
}

double Utilize::WrapDegs(double degs) {
  if (degs >  180) return degs - 360;
  return degs;
}

double Utilize::NormalizeRads(double rads) {
  rads = (rads < -TWO_PI) ? rads + TWO_PI :
         (rads >  TWO_PI) ? rads - TWO_PI : rads;
  return rads;
}

double Utilize::NormalizeDegs(double degs) {
  degs = (degs < -360) ? degs + 360 : 
         (degs >  360) ? degs - 360 : degs;
  return degs;
}
