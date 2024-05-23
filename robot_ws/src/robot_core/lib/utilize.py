import math

def AtTargetRange(number, target, range):
    return bool(target - range < number and number < target + range)

def WrapRads(rads):
    if rads >  math.pi :
        return rads - (2 * math.pi)
    if rads < -math.pi:
        return rads + (2 * math.pi)
    return WrapRads(rads)

def WrapDegs(degs):
    if degs >  180 :
        return degs - 360
    if degs < -180:
        return degs + 360
    return WrapDegs(degs)

def NormalizeRads(rads):
    rads = rads % (2 * math.pi)
    if rads == (2 * math.pi) :
        return 0
    return rads

def NormalizeDegs(degs):
    degs = degs % 360
    if degs == 360 :
        return 0
    return degs

def sig_num(number):
    return 0 if number == 0 else (-1 if number < 0 else 1)

def clip(value, min_val, max_val):
    return max(min(value, max_val), min_val)
