#include <gtest/gtest.h>
#include "bas_control/system_mgr.hpp"
#include "bas_control/module_info.hpp"

using namespace bas_control;

class SystemMgrTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code here
    }

    void TearDown() override {
        // Cleanup code here
    }
};

// 测试系统管理器初始化
TEST_F(SystemMgrTest, InitializationTest) {
    SystemMgr manager;
    
    // 测试初始化状态
    EXPECT_FALSE(manager.isRunning());
    EXPECT_EQ(manager.getSystemStatus().state, SystemState::STOPPED);
}

// 测试模块注册
TEST_F(SystemMgrTest, ModuleRegistrationTest) {
    SystemMgr manager;
    
    // 注册测试模块
    ModuleInfo module1;
    module1.name = "test_module_1";
    module1.type = ModuleType::DETECTION;
    module1.launch_file = "/test/path/module1.launch.py";
    
    ModuleInfo module2;
    module2.name = "test_module_2";
    module2.type = ModuleType::RECOGNITION;
    module2.launch_file = "/test/path/module2.launch.py";
    
    EXPECT_TRUE(manager.registerModule(module1));
    EXPECT_TRUE(manager.registerModule(module2));
    
    //测试重复注册
    EXPECT_FALSE(manager.registerModule(module1));
}

// 测试系统启动
TEST_F(SystemMgrTest, SystemStartupTest) {
    SystemMgr manager;
    
    // 注册测试模块
    ModuleInfo module;
    module.name = "test_module";
    module.type = ModuleType::DETECTION;
    module.launch_file = "/test/path/module.launch.py";
    manager.registerModule(module);
    
    //启动系统
    EXPECT_TRUE(manager.start());
    EXPECT_TRUE(manager.isRunning());
    
    //停系统
    EXPECT_TRUE(manager.stop());
    EXPECT_FALSE(manager.isRunning());
}

// 测试模块状态查询
TEST_F(SystemMgrTest, ModuleStatusTest) {
    SystemMgr manager;
    
    ModuleInfo module;
    module.name = "test_module";
    module.type = ModuleType::DETECTION;
    module.launch_file = "/test/path/module.launch.py";
    manager.registerModule(module);
    
    // 获取模块状态
    auto status = manager.getModuleStatus("test_module");
    EXPECT_EQ(status.state, ModuleState::STOPPED);
    
    //测试不存在的模块
    auto invalid_status = manager.getModuleStatus("nonexistent_module");
    EXPECT_EQ(invalid_status.state, ModuleState::UNKNOWN);
}

// 测试系统状态查询
TEST_F(SystemMgrTest, SystemStatusTest) {
    SystemMgr manager;
    
    // 获取系统状态
    auto status = manager.getSystemStatus();
    EXPECT_EQ(status.state, SystemState::STOPPED);
    EXPECT_EQ(status.running_modules, 0);
    EXPECT_EQ(status.total_modules, 0);
}

// 测试模块卸载
TEST_F(SystemMgrTest, ModuleUnregisterTest) {
    SystemMgr manager;
    
    ModuleInfo module;
    module.name = "test_module";
    module.type = ModuleType::DETECTION;
    module.launch_file = "/test/path/module.launch.py";
    manager.registerModule(module);
    
    //卸模块
    EXPECT_TRUE(manager.unregisterModule("test_module"));
    EXPECT_FALSE(manager.unregisterModule("test_module")); // 重复卸载应该失败
}

// 测试配置更新
TEST_F(SystemMgrTest, ConfigUpdateTest) {
    SystemMgr manager;
    
    SystemConfig new_config;
    new_config.auto_restart = true;
    new_config.max_restart_attempts = 5;
    new_config.restart_interval = 2.0;
    new_config.health_check_interval = 1.0;
    
    EXPECT_TRUE(manager.updateConfig(new_config));
    
    //验证配置更新
    auto status = manager.getSystemStatus();
    EXPECT_EQ(status.config.auto_restart, true);
    EXPECT_EQ(status.config.max_restart_attempts, 5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}