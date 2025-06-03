#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <stdexcept>
using namespace std;

struct MapConfig {

    std::string image_file;
    float resolution;
    Eigen::Vector2f origin;
    
    MapConfig() : image_file("map.pgm"), resolution(0.05f), origin(-51.200024,-51.200024){}
    
    bool loadMapParameters(const std::string& map_yaml_path) {
        ifstream file(map_yaml_path);
        
        if (!file.is_open()) {
            cerr << "Error on Opening" << map_yaml_path << endl;
            return false;
        }
        
        string line;
        while (std::getline(file, line)) {
        
        
            // Skip comments
            if (line.find('#') != string::npos) {
                line = line.substr(0, line.find('#'));
            }
            
            // Parse image
            if (line.find("image:") != string::npos) {
                image_file = line.substr(line.find(':') + 1);
                
                // This Removes spaces
                image_file.erase(0, image_file.find_first_not_of(" \t"));
                image_file.erase(image_file.find_last_not_of(" \t") + 1);
            }
            
            // Parse resolution  
            if (line.find("resolution:") != string::npos) {
                string res_str = line.substr(line.find(':') + 1);
                resolution = stof(res_str);
            }
            
            // Parse origin
            if (line.find("origin:") != string::npos) {
                std::string origin_str = line.substr(line.find('[') + 1);
                origin_str = origin_str.substr(0, origin_str.find(']'));
                
                // Extract x and y 
                size_t comma1 = origin_str.find(',');
                size_t comma2 = origin_str.find(',', comma1 + 1);
                
                origin.x() = stof(origin_str.substr(0, comma1));
                origin.y() = stof(origin_str.substr(comma1 + 1, comma2 - comma1 - 1));
            }
        }
        
        cerr << "Loaded: " << image_file << ", res=" <<  resolution << ", origin=[" << origin.x() << "," << origin.y() << "]" << endl;
        return true;
    }
};
