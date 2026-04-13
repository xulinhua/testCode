# 系统配置使用说明

## 概述

本项目采用统一的系统配置管理方式，所有网络通信相关的配置参数都集中存储在`sys_config/comm_network.yaml`文件中。这样可以确保各个项目使用一致的配置，避免配置不一致导致的问题。

## 配置文件结构

```
comm_network.yaml
├── network          # 网络地址配置
│   ├── server_ip    # 服务器IP地址
│   ├── server_port_dds  # DDS服务器端口
│   ├── server_port_udp  # UDP服务器端口
│   ├── client_ip    # 客户端IP地址
│   ├── client_port_dds  # DDS客户端端口
│   └── client_port_udp  # UDP客户端端口
├── dds_config       # DDS通信配置
│   ├── domain_id    # DDS域ID
│   ├── participant_name  # 参与者名称
│   ├── qos_profile  # QoS配置文件
│   ├── topics       # 主题配置
│   └── timeout      # 超时配置
└── udp_config       # UDP通信配置
    ├── buffer_size  # 缓冲区大小
    ├── max_packet_size  # 最大包大小
    ├── heartbeat    # 心跳配置
    └── retransmission  # 重传配置
```

## 在项目中使用配置

### C++项目中读取配置

```cpp
#include <yaml-cpp/yaml.h>

// 读取网络配置
YAML::Node config = YAML::LoadFile("sys_config/comm_network.yaml");
std::string server_ip = config["network"]["server_ip"].as<std::string>();
int server_port_dds = config["network"]["server_port_dds"].as<int>();
```

### Python项目中读取配置

```python
import yaml

# 读取网络配置
with open('sys_config/comm_network.yaml', 'r') as f:
    config = yaml.safe_load(f)
    
server_ip = config['network']['server_ip']
server_port_dds = config['network']['server_port_dds']
```

## 配置更新流程

1. 修改`comm_network.yaml`文件中的相应参数
2. 重新启动相关项目以使配置生效
3. 验证配置更新是否成功

## 注意事项

1. 所有项目都应从统一配置文件中读取网络参数
2. 不要在各个项目的本地配置文件中重复定义网络参数
3. 更新配置后需要重启相关服务