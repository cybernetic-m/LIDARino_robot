#include "draw_helpers.h"

void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color) {
  cv::line(dest, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]),
           cv::Scalar(color, color, color), 1);
}

void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius,
             cv::Scalar(color, color, color), 1);  // Add explicit thickness parameter
}

int showCanvas(Canvas& canvas, int timeout_ms) {
  cv::imshow("canvas", canvas);
  int key = cv::waitKey(timeout_ms);
  if (key == 27)  // exit on ESC
    exit(0);
  // cerr << "key" << key << endl;
  return key;
}




int showScaledCanvas(Canvas& canvas, float scale_factor, int timeout_ms) {
    if (canvas.empty()) {
        std::cerr << "Warning: Attempting to show empty canvas" << std::endl;
        return -1;
    }
    
    Canvas scaled_canvas;
    int new_width = static_cast<int>(canvas.cols * scale_factor);
    int new_height = static_cast<int>(canvas.rows * scale_factor);
    
    // Ensure minimum reasonable size
    new_width = std::max(200, new_width);
    new_height = std::max(200, new_height);
    
    // Ensure maximum reasonable size (for screen fitting)
    new_width = std::min(1920, new_width);
    new_height = std::min(1080, new_height);
    
    cv::resize(canvas, scaled_canvas, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
    
    cv::namedWindow("Scaled Canvas", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Scaled Canvas", 50, 50);
    cv::imshow("Scaled Canvas", scaled_canvas);
    
    int key = cv::waitKey(timeout_ms);
    if (key == 27) { // ESC key
        std::cerr << "ESC pressed - exiting application" << std::endl;
        exit(0);
    }
    
    return key;
}

// Function to automatically fit canvas to screen
int showFittedCanvas(Canvas& canvas, int max_width, int max_height, int timeout_ms) {
    if (canvas.empty()) {
        std::cerr << "Warning: Attempting to show empty canvas" << std::endl;
        return -1;
    }
    
    // Calculate scaling factor to fit within specified dimensions
    float width_scale = static_cast<float>(max_width) / canvas.cols;
    float height_scale = static_cast<float>(max_height) / canvas.rows;
    float scale = std::min(width_scale, height_scale);
    
    // Don't upscale if already small enough
    scale = std::min(scale, 1.0f);
    
    Canvas fitted_canvas;
    int new_width = static_cast<int>(canvas.cols * scale);
    int new_height = static_cast<int>(canvas.rows * scale);
    
    cv::resize(canvas, fitted_canvas, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
    
    cv::namedWindow("Fitted Canvas", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Fitted Canvas", 100, 100);
    cv::imshow("Fitted Canvas", fitted_canvas);
    
    std::cout << "Canvas scaled from " << canvas.cols << "x" << canvas.rows 
              << " to " << new_width << "x" << new_height 
              << " (scale factor: " << scale << ")" << std::endl;
    
    int key = cv::waitKey(timeout_ms);
    if (key == 27) { // ESC key
        std::cerr << "ESC pressed - exiting application" << std::endl;
        exit(0);
    }
    
    return key;
}