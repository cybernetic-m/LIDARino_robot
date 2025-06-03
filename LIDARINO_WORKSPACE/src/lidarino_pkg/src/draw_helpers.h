#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

using Canvas = cv::Mat;

// draws a line from p0 to p1
void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color);
void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color);

int showCanvas(Canvas& canvas, int timeout_ms);
int showScaledCanvas(Canvas& canvas, float scale_factor = 1.f, int timeout_ms = 1);
int showCroppedScaledCanvas(Canvas& canvas, float crop_width=200, float crop_height=200, float scale = 3.0f, int timeout_ms = 1);
int showCanvasMode(Canvas& canvas, int mode = 1, float crop_width=200, float crop_height=200, float scale = 3.0f, int timeout_ms = 1);