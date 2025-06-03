#include "draw_helpers.h"
#include <iostream>


void drawLine(Canvas& dest, const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, uint8_t color) {
  cv::line(dest, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]),
           cv::Scalar(color, color, color), 1);
}

void drawCircle(Canvas& dest, const Eigen::Vector2f& center, int radius, uint8_t color) {
  cv::circle(dest, cv::Point(center[0], center[1]), radius,
             cv::Scalar(color, color, color), 1);  
}

int showCanvas(Canvas& canvas, int timeout_ms) {
    if (canvas.empty()) {
        std::cerr << "Warning: Attempting to show empty canvas" << std::endl;
        return -1;
    }
    
    //cv::namedWindow("canvas", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("canvas", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);

    cv::namedWindow("canvas", cv::WINDOW_NORMAL);
    cv::resizeWindow("canvas", 800, 600);  
    cv::imshow("canvas", canvas);
    
    int key = cv::waitKey(timeout_ms);
    
    if (key == 27) { // ESC key
        std::cerr << "ESC pressed - exiting application" << std::endl;
        exit(0);
    }
    
    return key;
}




int showScaledCanvas(Canvas& canvas, float scale, int timeout_ms) {
    if (canvas.empty()) {
        std::cerr << "Warning: Attempting to show empty canvas" << std::endl;
        return -1;
    }
    
    Canvas scaled_canvas;

    int new_width = static_cast<int>(canvas.cols * scale);
    int new_height = static_cast<int>(canvas.rows * scale);
    
    new_width = std::max(50, new_width);
    new_height = std::max(50, new_height);
    new_width = std::min(2000, new_width);
    new_height = std::min(2000, new_height);
    
    cv::resize(canvas, scaled_canvas, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
    
    cv::namedWindow("Scaled Canvas", cv::WINDOW_NORMAL);
    //cv::namedWindow("Scaled Canvas", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("Scaled Canvas", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Scaled Canvas", new_width, new_height); 
    cv::imshow("Scaled Canvas", scaled_canvas);
    
    int key = cv::waitKey(timeout_ms);
    if (key == 27) {
        std::cerr << "ESC pressed - exiting application" << std::endl;
        exit(0);
    }
    return key;
}



int showCroppedScaledCanvas(Canvas& canvas, float crop_width, float crop_height, float scale, int timeout_ms) {
    if (canvas.empty()) {
        std::cerr << "Warning: Empty canvas for crop+scale" << std::endl;
        return -1;
    }
    
    // CROPPING THE CANVAS  AROUND THE CENTER
    int center_x = canvas.cols / 2;
    int center_y = canvas.rows / 2;
    
    int left = std::max(0, center_x - static_cast<int>(crop_width) / 2);
    int top = std::max(0, center_y - static_cast<int>(crop_height) / 2);
    int right = std::min(canvas.cols, center_x + static_cast<int>(crop_width) / 2);
    int bottom = std::min(canvas.rows, center_y + static_cast<int>(crop_height) / 2);
    
    cv::Rect crop_rect(left, top, right - left, bottom - top);
    Canvas cropped_canvas = canvas(crop_rect).clone();
    

    //RESCALING
    int new_width = static_cast<int>(cropped_canvas.cols * scale);
    int new_height = static_cast<int>(cropped_canvas.rows * scale);
    
    new_width = std::max(50, new_width);
    new_height = std::max(50, new_height);
    new_width = std::min(2000, new_width);
    new_height = std::min(2000, new_height);
        
    Canvas final_canvas;
    cv::resize(cropped_canvas, final_canvas, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
    
    cv::namedWindow("Cropped and Scaled Canvas", cv::WINDOW_NORMAL); //cv::WINDOW_AUTOSIZE
    //cv::namedWindow("canvas", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    //cv::moveWindow("Cropped and Scaled Canvas", 250, 250);
    cv::resizeWindow("Cropped and Scaled Canvas", new_width, new_height);
    cv::imshow("Cropped and Scaled Canvas", final_canvas);

    int key = cv::waitKey(timeout_ms);
    if (key == 27) {
        std::cerr << "ESC pressed - exiting application" << std::endl;
        exit(0);
    }
    return key;
}


int showCanvasMode(Canvas& canvas, int mode, float crop_width, float crop_height, float scale, int timeout_ms) {
    if (canvas.empty()) {
        std::cerr << "Warning: Empty canvas provided to showCanvasMode" << std::endl;
        return -1;
    }
    switch (mode) {
        case 1: // Original size
            return showCanvas(canvas, timeout_ms);
            
        case 2: // Scaled
            return showScaledCanvas(canvas, scale, timeout_ms);
            
        case 3: // Cropped and Scaled
            return showCroppedScaledCanvas(canvas, crop_width, crop_height, scale, timeout_ms); 
    
        default: 
            std::cerr << "Invalid Mode: " << mode << "using scale 3 and crop 100x100" << std::endl;
            return showCroppedScaledCanvas(canvas, 100.0f, 100.0f, 3.0f, timeout_ms); 
        
    }
}