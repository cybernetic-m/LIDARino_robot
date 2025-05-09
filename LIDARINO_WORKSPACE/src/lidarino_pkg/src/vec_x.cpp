#include "vec_x.h"
using namespace std;

ostream& operator<<(ostream& os, const Vec2& src) {
  os << "Vec2 " << (&src) << " [ ";
  for (int i = 0; i < src.Dim; ++i) {
    os << src.values[i] << " ";
  }
  os << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Vec2i& src) {
  os << "Vec2i " << (&src) << " [ ";
  for (int i = 0; i < src.Dim; ++i) {
    os << src.values[i] << " ";
  }
  os << "]";
  return os;
}

ostream& operator<<(ostream& os, const Vec3& src) {
  os << "Vec3 (" << &src << ") [ ";
  for (int i = 0; i < src.Dim; ++i) {
    os << src.values[i] << " ";
  }
  os << "]";
  return os;
}
