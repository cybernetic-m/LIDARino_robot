#pragma once
#include "precision.h"
#include <iostream>

// simple vector2 class
struct Vec2 {
  // static in a class: declares a variable global to all instances of Vec2f
  // constexpr specifies that this variable is evaluated at compile time
  static constexpr int Dim = 2;

  // Array of values
  Scalar values[Dim];

  // inline in a function makes the function to be expanded at compilation
  // time in the positions where it is called.
  // inline functions do not admit recursion, and should be declared
  // in the header
  inline void setZero() {
    for (int i = 0; i < Dim; ++i) values[i] = Scalar(0);
  }

  // we redefine the [] operator, to support the element access
  // in an array like style. We to this in two flavors
  // const and non const

  // this returns a reference to an element of the vector,
  // hence you can change it
  inline Scalar& operator[](int p) { return values[p]; }

  // this returns a CONST reference to an element of the vector,
  // hence you cannot change it, and it is safe to use in const methods
  inline const Scalar& operator[](int p) const { return values[p]; }

  inline Vec2 operator+(const Vec2& other) const {
    Vec2 ret = *this;
    for (int i = 0; i < Dim; ++i) {
      ret[i] += other[i];
    }
    return ret;
  }

  inline Vec2& operator+=(const Vec2& other) {
    for (int i = 0; i < Dim; ++i) {
      values[i] += other[i];
    }
    return *this;
  }

  inline Vec2 operator*(const Scalar& other) const {
    Vec2 ret = *this;
    for (int i = 0; i < Dim; ++i) {
      ret[i] *= other;
    }
    return ret;
  }

  inline Vec2& operator*=(const Vec2& other) {
    for (int i = 0; i < Dim; ++i) {
      values[i] *= other[i];
    }
    return *this;
  }
};

// streaming operator allows to use cout << vec3 to print on a stream
std::ostream& operator<<(std::ostream& os, const Vec2& src);

struct Vec2i {
  static constexpr int Dim = 2;
  int values[Dim];
  int& operator[](int p) { return values[p]; }
  const int& operator[](int p) const { return values[p]; }

  inline Vec2i operator+(const Vec2i& other) const {
    Vec2i ret = *this;
    for (int i = 0; i < Dim; ++i) {
      ret[i] += other[i];
    }
    return ret;
  }

  inline Vec2i& operator+=(const Vec2i& other) {
    for (int i = 0; i < Dim; ++i) {
      values[i] += other[i];
    }
    return *this;
  }

  inline Vec2i operator*(const int& other) const {
    Vec2i ret = *this;
    for (int i = 0; i < Dim; ++i) {
      ret[i] *= other;
    }
    return ret;
  }

  inline Vec2i& operator*=(const Vec2i& other) {
    for (int i = 0; i < Dim; ++i) {
      values[i] *= other[i];
    }
    return *this;
  }
};

std::ostream& operator<<(std::ostream& os, const Vec2i& src);

// same as before but with 3 dimensions
struct Vec3 {
  static constexpr int Dim = 3;
  Scalar values[Dim];

  Scalar& operator[](int p) { return values[p]; }
  const Scalar& operator[](int p) const { return values[p]; }

  // default constructor
  Vec3(){}

  Vec3(Scalar x, Scalar y, Scalar z) {
    values[0]=x;
    values[1]=y;
    values[2]=z;
  }
  
  inline Vec3 operator+(const Vec3& other) const {
    Vec3 ret = *this;
    for (int i = 0; i < Dim; ++i) {
      ret[i] += other[i];
    }
    return ret;
  }
  
  inline Vec3& operator+=(const Vec3& other) {
    for (int i = 0; i < Dim; ++i) {
      values[i] += other[i];
    }
    return *this;
  }

  inline Vec3 operator*(const Scalar& other) const {
    Vec3 ret = *this;
    for (int i = 0; i < Dim; ++i) {
      ret[i] *= other;
    }
    return ret;
  }

  inline Vec3& operator*=(const Vec3& other) {
    for (int i = 0; i < Dim; ++i) {
      values[i] *= other[i];
    }
    return *this;
  }
};

std::ostream& operator<<(std::ostream& os, const Vec3& src);
