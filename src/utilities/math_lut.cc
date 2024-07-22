#include "math_lut.hh"
#include <cmath>

struct Pow2TableRange {
	static constexpr float min = -5.1f;
	static constexpr float max = 5.1f;
};

Mapping::LookupTable_t<64, float> Pow2 =
	Mapping::LookupTable_t<64, float>::generate<Pow2TableRange>([](auto x) { return std::pow(2.f, x); });
