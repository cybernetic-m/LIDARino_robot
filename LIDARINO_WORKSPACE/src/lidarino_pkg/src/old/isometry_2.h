#pragma once
#include <math.h>

#include <iostream>

#include "precision.h"
#include "vec_x.h"

struct Isometry2 {
  static constexpr int Dim = 2;
  Scalar rotation[Dim][Dim];

  Vec2 translation;

  static inline const Isometry2& Identity() {
    static Isometry2 iso;
    iso.setIdentity();
    return iso;
  };

  Isometry2() {}

  Isometry2(const Vec3& src) {
    const Scalar& x = src[0];
    const Scalar& y = src[1];
    const Scalar& theta = src[2];
    const Scalar s = sin(theta);
    const Scalar c = cos(theta);
    rotation[0][0] = c;
    rotation[0][1] = -s;
    rotation[1][0] = s;
    rotation[1][1] = c;
    translation[0] = x;
    translation[1] = y;
  }

  Isometry2(Scalar x, Scalar y, Scalar theta) : Isometry2(Vec3(x, y, theta)) {}

  Vec3 toVector() const {
    Vec3 dest;
    dest[0] = translation[0];
    dest[1] = translation[1];
    dest[2] = atan2(rotation[1][0], rotation[0][0]);
    return dest;
  }

  inline void setIdentity() {
    for (int r = 0; r < Dim; ++r) {
      for (int c = 0; c < Dim; ++c) {
        if (r == c)
          rotation[r][c] = Scalar(1);
        else
          rotation[r][c] = Scalar(0);
      }
    }
    translation.setZero();
  }
  /* inv(R, t) = (R^T, -R^T* T) */
  inline Isometry2 inverse() const {
    Isometry2 dest;
    dest.setIdentity();

    for (int r = 0; r < Dim; ++r) {
      for (int c = 0; c < Dim; ++c) {
        dest.rotation[r][c] = rotation[c][r];
      }
    }
    for (int r = 0; r < Dim; ++r)
      for (int c = 0; c < Dim; ++c)
        dest.translation[r] -= dest.rotation[r][c] * translation[c];
    return dest;
  }

  /* (R1, t1)*(R1, t2) = (R1*R2, t1 + R1*t2) */
  inline Isometry2 operator*(const Isometry2& other) const {
    Isometry2 dest;

    for (int r = 0; r < Dim; ++r) {
      dest.translation[r] = translation[r];
      for (int c = 0; c < Dim; ++c) {
        Scalar accumulator = 0;
        for (int k = 0; k < Dim; ++k) {
          accumulator += rotation[r][k] * other.rotation[k][c];
        }
        dest.rotation[r][c] = accumulator;
        dest.translation[r] += rotation[r][c] * other.translation[c];
      }
    }
    return dest;
  }

  // (R|t)*v = R*v+t;
  inline Vec2 operator*(const Vec2& other) {
    Vec2 ret;
    for (int r = 0; r < Dim; ++r) {
      Scalar acc = 0.0;
      for (int c = 0; c < Dim; ++c) {
        acc += rotation[r][c] * other[c];
      }
      acc += translation[r];
      ret[r] = acc;
    }
    return ret;
  }
};

std::ostream& operator<<(std::ostream& os, const Isometry2& iso);
