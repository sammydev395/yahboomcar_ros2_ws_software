#include "System.h"
#include <iostream>

int main() {
    try {
        std::cout << "Initializing ORB-SLAM2..." << std::endl;
        
        ORB_SLAM2::System SLAM("Vocabulary/ORBvoc.txt", "Examples/RGB-D/TUM1.yaml", 
                               ORB_SLAM2::System::RGBD, false);
        
        std::cout << "ORB-SLAM2 initialized successfully!" << std::endl;
        
        // Clean shutdown
        SLAM.Shutdown();
        
        std::cout << "Test completed successfully!" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
} 