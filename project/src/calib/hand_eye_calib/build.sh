#!/bin/bash

# 手眼标定项目构建脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查依赖项
check_dependencies() {
    print_info "检查依赖项..."
    
    # 检查CMake
    if ! command -v cmake &> /dev/null; then
        print_error "未找到CMake，请先安装CMake"
        return 1
    fi
    
    # 检查编译器
    if ! command -v g++ &> /dev/null; then
        print_error "未找到g++编译器，请先安装g++"
        return 1
    fi
    
    # 检查OpenCV
    if ! pkg-config --exists opencv4; then
        print_error "未找到OpenCV4，请先安装OpenCV4开发包"
        return 1
    fi
    
    # 检查Eigen3
    if ! pkg-config --exists eigen3; then
        print_error "未找到Eigen3，请先安装Eigen3开发包"
        return 1
    fi
    
    # 检查nlohmann_json
    if ! pkg-config --exists nlohmann_json; then
        print_error "未找到nlohmann_json，请先安装nlohmann_json开发包"
        return 1
    fi
    
    print_info "所有依赖项检查通过"
    return 0
}

# 清理构建目录
clean_build() {
    print_info "清理构建目录..."
    if [ -d "build" ]; then
        rm -rf build
        print_info "已删除旧的构建目录"
    fi
    
    if [ -d "install" ]; then
        rm -rf install
        print_info "已删除旧的安装目录"
    fi
}

# 创建构建目录并配置项目
configure_project() {
    print_info "配置项目..."
    
    mkdir -p build
    cd build
    
    # 配置项目
    if ! cmake .. -DCMAKE_BUILD_TYPE=Release; then
        print_error "项目配置失败"
        cd ..
        return 1
    fi
    
    cd ..
    print_info "项目配置完成"
    return 0
}

# 编译项目
build_project() {
    print_info "编译项目..."
    
    cd build
    
    # 编译项目
    if ! make -j$(nproc); then
        print_error "项目编译失败"
        cd ..
        return 1
    fi
    
    cd ..
    print_info "项目编译完成"
    return 0
}

# 安装项目
install_project() {
    print_info "安装项目..."
    
    cd build
    
    # 安装项目
    if ! make install; then
        print_error "项目安装失败"
        cd ..
        return 1
    fi
    
    cd ..
    print_info "项目安装完成"
    return 0
}

# 运行测试
run_tests() {
    print_info "运行测试..."
    
    cd build
    
    # 运行测试
    if ! make test; then
        print_warning "部分测试未通过"
        cd ..
        return 1
    fi
    
    cd ..
    print_info "所有测试通过"
    return 0
}

# 显示帮助信息
show_help() {
    echo "手眼标定项目构建脚本"
    echo "用法: $0 [选项]"
    echo "选项:"
    echo "  clean     清理构建目录"
    echo "  configure 配置项目"
    echo "  build     编译项目"
    echo "  install   安装项目"
    echo "  test      运行测试"
    echo "  all       执行完整构建流程 (默认)"
    echo "  help      显示此帮助信息"
}

# 主函数
main() {
    # 如果没有参数，则执行完整构建流程
    if [ $# -eq 0 ]; then
        set -- "all"
    fi
    
    # 处理命令行参数
    case "$1" in
        clean)
            clean_build
            ;;
        configure)
            check_dependencies || return 1
            configure_project || return 1
            ;;
        build)
            build_project || return 1
            ;;
        install)
            install_project || return 1
            ;;
        test)
            run_tests || return 1
            ;;
        all)
            check_dependencies || return 1
            clean_build
            configure_project || return 1
            build_project || return 1
            install_project || return 1
            run_tests
            ;;
        help)
            show_help
            ;;
        *)
            print_error "未知选项: $1"
            show_help
            return 1
            ;;
    esac
    
    return 0
}

# 执行主函数
main "$@"