#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

using Canvas = cv::Mat;

// draws a line from p0 to p1
void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color);
void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color);
int showCanvas(Canvas& canvas, int timeout_ms);



// Enhanced canvas display functions for better visualization
int showScaledCanvas(Canvas& canvas, float scale_factor = 0.5f, int timeout_ms = 1);
int showFittedCanvas(Canvas& canvas, int max_width = 1200, int max_height = 800, int timeout_ms = 1);

// Utility constants for common screen sizes
constexpr int SCREEN_HD_WIDTH = 1280;
constexpr int SCREEN_HD_HEIGHT = 720;
constexpr int SCREEN_FHD_WIDTH = 1920;
constexpr int SCREEN_FHD_HEIGHT = 1080;