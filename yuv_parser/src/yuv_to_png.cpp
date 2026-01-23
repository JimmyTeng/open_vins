#include "yuv_parser.h"
#include <iostream>
#include <string>

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <dump_yuv_dir> [width] [height]" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  dump_yuv_dir  : Path to directory containing YUV files and FrameInfo.txt" << std::endl;
    std::cout << "  width         : Image width (default: 640)" << std::endl;
    std::cout << "  height        : Image height (default: 480)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << program_name << " data/yuv_data/20260115_1034/indoor_rich_texture/horizontal_move_indoor_rich_texture/dump_yuv" << std::endl;
    std::cout << "  " << program_name << " data/yuv_data/20260115_1034/indoor_rich_texture/horizontal_move_indoor_rich_texture/dump_yuv 640 480" << std::endl;
}

int main(int argc, char* argv[]) {
    // Check arguments
    if (argc < 2) {
        std::cerr << "Error: Missing required argument: dump_yuv_dir" << std::endl;
        std::cerr << std::endl;
        printUsage(argv[0]);
        return 1;
    }
    
    std::string dump_yuv_dir = argv[1];
    
    // Parse optional width and height arguments
    int width = 640;
    int height = 480;
    
    if (argc >= 3) {
        width = std::stoi(argv[2]);
        if (width <= 0) {
            std::cerr << "Error: Invalid width: " << argv[2] << std::endl;
            return 1;
        }
    }
    
    if (argc >= 4) {
        height = std::stoi(argv[3]);
        if (height <= 0) {
            std::cerr << "Error: Invalid height: " << argv[3] << std::endl;
            return 1;
        }
    }
    
    std::cout << "=== YUV to PNG Converter ===" << std::endl;
    std::cout << "Input directory: " << dump_yuv_dir << std::endl;
    std::cout << "Image size: " << width << "x" << height << std::endl;
    std::cout << std::endl;
    
    // Create parser with specified dimensions
    YUVParser yuv_parser(width, height);
    
    // Export frames to PNG
    int exported_count = yuv_parser.exportFramesToPNG(dump_yuv_dir);
    
    if (exported_count < 0) {
        std::cerr << "Error: Failed to export frames" << std::endl;
        return 1;
    }
    
    std::cout << std::endl;
    std::cout << "=== Conversion Complete ===" << std::endl;
    std::cout << "Successfully exported " << exported_count << " frames" << std::endl;
    
    return 0;
}
