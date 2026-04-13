#include "cam_sdk_aruco_test/cam_sdk_aruco_test.hpp"
#include <iostream>
#include <string>
#include <memory>

#ifdef __ROS_ENV__
#include <rclcpp/rclcpp.hpp>
#endif

int main(int argc, char* argv[]) {
#ifdef __ROS_ENV__
    // 初始化ROS
    rclcpp::init(argc, argv);
#endif
    
    // 检查命令行参数
    std::string config_file = "../../config/aruco_show_config.yaml";
    if (argc > 1) {
        config_file = argv[1];
        std::cout << "Using custom config file: " << config_file << std::endl;
    } else {
        std::cout << "Using default config file: " << config_file << std::endl;
    }
    
    // 创建测试对象
#ifdef __ROS_ENV__
    auto test_app = std::make_shared<CamSdkArucoTest>();
#else
    CamSdkArucoTest test_app;
#endif
    
    // 初始化
    std::cout << "Initializing application..." << std::endl;
#ifdef __ROS_ENV__
    if (!test_app->initialize(config_file)) {
        std::cerr << "Failed to initialize the application" << std::endl;
        return -1;
    }
#else
    if (!test_app.initialize(config_file)) {
        std::cerr << "Failed to initialize the application" << std::endl;
        return -1;
    }
#endif
    
    std::cout << "Application initialized successfully" << std::endl;
    
    // 运行主循环
#ifdef __ROS_ENV__
    if (!test_app->run()) {
        std::cerr << "Failed to run the application" << std::endl;
        return -1;
    }
    std::cout << "run the application successfully" << std::endl;
#else
    if (!test_app.run()) {
        std::cerr << "Failed to run the application" << std::endl;
        return -1;
    }
    std::cout << "run the application successfully" << std::endl;
#endif
    
    std::cout << "Application finished successfully" << std::endl;
        // 在主线程中处理ROS回调
#ifdef __ROS_ENV__
    rclcpp::spin(test_app);
#endif
#ifdef __ROS_ENV__
    // 关闭ROS
    rclcpp::shutdown();
#endif
    
    return 0;
}