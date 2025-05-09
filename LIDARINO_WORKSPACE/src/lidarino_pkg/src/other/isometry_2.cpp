#include "isometry_2.h"

using namespace std;

ostream& operator<<(ostream& os, const Isometry2& src) {
  os << "Iso2 (" << &src << ") " << endl;
  for (int r = 0; r < src.Dim; ++r) {
    for (int c = 0; c < src.Dim; ++c) os << src.rotation[r][c] << " ";
    os << "| " << src.translation[r] << endl;
  }
  return os;
}
