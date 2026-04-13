#include "yolo_det_node.h"

// ---------- 工具：Mat -> PNG -> base64 ----------
static std::string mat2png_base64(const cv::Mat& img)
{
  std::vector<uchar> buf;
  cv::imencode(".png", img, buf);   // PNG 压缩
  static const char* tbl =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(buf.size() * 4 / 3 + 4);
  for (size_t i = 0; i < buf.size(); ) {
    uint32_t b = (buf[i++] & 0xFF) << 16;
    if (i < buf.size()) b |= (buf[i++] & 0xFF) << 8;
    if (i < buf.size()) b |= (buf[i++] & 0xFF);
    out.push_back(tbl[(b >> 18) & 0x3F]);
    out.push_back(tbl[(b >> 12) & 0x3F]);
    out.push_back(i > buf.size()     ? '=' : tbl[(b >> 6) & 0x3F]);
    out.push_back(i > buf.size() + 1 ? '=' : tbl[b & 0x3F]);
  }
  return out;
}

Yolo_Det_Node::Yolo_Det_Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("Yolo_Detection", options)
{
  // 新增相机类型参数
  camera_type_ = this->declare_parameter("camera_type", "Realsense");
  
  // 根据相机类型设置默认话题
  if (camera_type_ == "Gemini") 
  {
      color_image_topic_ = "/camera/color/image_raw";
      depth_image_topic_ = "/camera/depth/image_raw";
      camera_info_topic_ = "/camera/color/camera_info";
  } 
  else 
  {
      color_image_topic_ = "/camera/camera/color/image_raw";
      depth_image_topic_ = "/camera/camera/aligned_depth_to_color/image_raw";
      camera_info_topic_ = "/camera/camera/color/camera_info";
  }

  // 初始化相机内参状态
  camera_intrinsics_initialized_ = false;
  fx_ = 0.0f;
  fy_ = 0.0f;
  cx_ = 0.0f;
  cy_ = 0.0f;

  // 初始化新增功能 - 解耦合设计
  setup_camera_intrinsics();  // 设置相机内参
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
//   publish_static_tf();        // 发布静态TF

  yolo_det_.load_engine(engine_name_);
  color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(color_image_topic_, 10, std::bind(&Yolo_Det_Node::Color_Callback, this, std::placeholders::_1)); 
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(depth_image_topic_, 10, std::bind(&Yolo_Det_Node::Depth_Callback, this, std::placeholders::_1)); 
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10, std::bind(&Yolo_Det_Node::CameraInfo_Callback, this, std::placeholders::_1));
  box_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/det_image", 1);
  res_image_pub_ = this->create_publisher<std_msgs::msg::String>("/det_image_png", 1);
  det_res_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/det_res", 1);
}

void Yolo_Det_Node::Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  
  try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::vector<Detection> detections;
        detections.clear();
        detections = yolo_det_.infer(frame);
        Intrinsics intrinsics;
        intrinsics.fx = fx_;
        intrinsics.fy = fy_;
        intrinsics.cx = cx_;
        intrinsics.cy = cy_;
        publish(detections, msg->header);

        yolo_det_.draw_results(frame, detections, depth_frame_, intrinsics, class_names);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        box_image_pub_->publish(*msg);
        cv::Size target(160, 120); 
        cv::Mat resized;
        cv::resize(frame, resized, target, 0, 0, cv::INTER_LINEAR);    
        auto png_base64 = mat2png_base64(resized);
        auto out = std_msgs::msg::String();
        out.data = std::move(png_base64);
        res_image_pub_->publish(out);        
      } 
      catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "trans image error: %s", e.what());
      }
}
  
void Yolo_Det_Node::Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try 
  {
    depth_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image; 
  } 
  catch (const cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR(this->get_logger(), "trans depth image error: %s", e.what());
  }
}

// 设置相机内参 - 通过API获取
void Yolo_Det_Node::setup_camera_intrinsics()
{
    // 等待相机信息话题发布内参数据
    // 这里不设置硬编码值，而是等待CameraInfo回调函数获取真实的内参数据
    RCLCPP_INFO(this->get_logger(), "等待相机内参数据...");
    RCLCPP_INFO(this->get_logger(), "订阅相机信息话题: %s", camera_info_topic_.c_str());
}

// 相机信息回调函数 - 通过API获取真实的内参数据
void Yolo_Det_Node::CameraInfo_Callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (camera_intrinsics_initialized_) 
    {
        return;  // 已经初始化过，避免重复处理
    }
    
    // 从CameraInfo消息中提取内参
    if (msg->k.size() >= 9) 
    {
        // 内参矩阵 K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        fx_ = msg->k[0];  // 焦距x
        fy_ = msg->k[4];  // 焦距y
        cx_ = msg->k[2];  // 主点x
        cy_ = msg->k[5];  // 主点y
        
        camera_intrinsics_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), "成功获取相机内参:");
        RCLCPP_INFO(this->get_logger(), "  fx=%.2f, fy=%.2f", fx_, fy_);
        RCLCPP_INFO(this->get_logger(), "  cx=%.2f, cy=%.2f", cx_, cy_);
        RCLCPP_INFO(this->get_logger(), "  图像尺寸: %dx%d", msg->width, msg->height);
        
        // 检查内参是否合理
        if (fx_ <= 0 || fy_ <= 0) 
        {
            RCLCPP_WARN(this->get_logger(), "相机内参异常，使用默认值");
            // 使用合理的默认值
            if (camera_type_ == "Gemini") 
            {
                fx_ = 610.0f;
                fy_ = 610.0f;
                cx_ = msg->width / 2.0f;
                cy_ = msg->height / 2.0f;
            }
            else 
            {
                fx_ = 615.0f;
                fy_ = 615.0f;
                cx_ = msg->width / 2.0f;
                cy_ = msg->height / 2.0f;
            }
        }
    }
    else 
    {
        RCLCPP_WARN(this->get_logger(), "CameraInfo消息格式错误，使用默认内参");
        // 使用默认内参
        if (camera_type_ == "Gemini") 
        {
            fx_ = 610.0f;
            fy_ = 610.0f;
            cx_ = 320.0f;
            cy_ = 240.0f;
        }
        else 
        {
            fx_ = 615.0f;
            fy_ = 615.0f;
            cx_ = 320.0f;
            cy_ = 240.0f;
        }
        camera_intrinsics_initialized_ = true;
    }
}

// 发布相机到base_link的静态TF
void Yolo_Det_Node::publish_static_tf()
{
    camera_to_base_tf_.header.stamp = this->now();
    camera_to_base_tf_.header.frame_id = "base_link";
    camera_to_base_tf_.child_frame_id = "camera_link";
    
    // 从参数服务器获取相机位置（支持动态配置）
    double camera_x = this->declare_parameter("cam_x", 0.1);   // x方向偏移
    double camera_y = this->declare_parameter("cam_y", 0.0);   // y方向偏移
    double camera_z = this->declare_parameter("cam_z", 1.1);   // z方向偏移
    
    // 从参数服务器获取相机旋转角度
    double camera_roll = this->declare_parameter("cam_roll", 0.0);   // roll角度
    double camera_pitch = this->declare_parameter("cam_pitch", 0.23); // pitch角度
    double camera_yaw = this->declare_parameter("cam_yaw", 0.0);     // yaw角度
    
    // 设置平移
    camera_to_base_tf_.transform.translation.x = camera_x;
    camera_to_base_tf_.transform.translation.y = camera_y;
    camera_to_base_tf_.transform.translation.z = camera_z;
    
    // 设置旋转（欧拉角：roll, pitch, yaw）
    tf2::Quaternion q;
    q.setRPY(camera_roll, camera_pitch, camera_yaw);
    camera_to_base_tf_.transform.rotation = tf2::toMsg(q);
    
    static_tf_broadcaster_->sendTransform(camera_to_base_tf_);
    RCLCPP_INFO(this->get_logger(), "发布静态TF: base_link -> camera_link, 位置: (%.2f, %.2f, %.2f), 旋转: (%.2f, %.2f, %.2f)", 
                camera_x, camera_y, camera_z, camera_roll, camera_pitch, camera_yaw);
}

// 计算camera_link下的3D坐标
geometry_msgs::msg::Point Yolo_Det_Node::calculate_3d_position(const Detection& det)
{
    geometry_msgs::msg::Point camera_point;
    
    // 检查相机内参是否已初始化
    if (!camera_intrinsics_initialized_) 
    {
        RCLCPP_WARN(this->get_logger(), "相机内参未初始化，无法计算3D坐标");
        return camera_point;
    }
    
    // 检查内参是否有效
    if (fx_ <= 0 || fy_ <= 0) 
    {
        RCLCPP_WARN(this->get_logger(), "相机内参无效，无法计算3D坐标");
        return camera_point;
    }
    
    // 获取边界框中心点
    float center_x = det.bbox[0];  // 边界框中心x
    float center_y = det.bbox[1];  // 边界框中心y
    
    // 确保深度图像有效
    if (depth_frame_.empty()) 
    {
        RCLCPP_WARN(this->get_logger(), "深度图像为空，无法计算3D坐标");
        return camera_point;
    }
    
    // 检查像素坐标是否在图像范围内
    int pixel_x = static_cast<int>(center_x);
    int pixel_y = static_cast<int>(center_y);
    
    if (pixel_x < 0 || pixel_x >= depth_frame_.cols || 
        pixel_y < 0 || pixel_y >= depth_frame_.rows) 
    {
        RCLCPP_WARN(this->get_logger(), "像素坐标超出图像范围: (%d, %d)", pixel_x, pixel_y);
        return camera_point;
    }
    
    // 获取深度值
    uint16_t depth_value = depth_frame_.at<uint16_t>(pixel_y, pixel_x);
    
    if (depth_value == 0) 
    {
        RCLCPP_WARN(this->get_logger(), "深度值为0，无效深度");
        return camera_point;
    }
    
    // 转换为米
    float depth_m = depth_value / 1000.0f;
    
    // 计算3D坐标（相机坐标系）
    // 注意：图像坐标系Y轴向下为正，相机坐标系Y轴向上为正，需要取反
    camera_point.x = (center_x - cx_) * depth_m / fx_; // X坐标
    camera_point.y = (center_y - cy_) * depth_m / fy_; // Y坐标
    camera_point.z = depth_m;                          // Z坐标（深度）
    
    return camera_point;
}

// 将camera_link坐标转换为base_link坐标
geometry_msgs::msg::Point Yolo_Det_Node::transform_to_base_link(const geometry_msgs::msg::Point& camera_point)
{
    geometry_msgs::msg::Point base_point;
    
    // 使用TF变换将camera_link坐标转换到base_link
    tf2::Vector3 camera_vec(camera_point.x, camera_point.y, camera_point.z);
    tf2::Transform base_to_camera;
    
    // 从TransformStamped中提取变换（注意：camera_to_base_tf_表示从base_link到camera_link的变换）
    tf2::fromMsg(camera_to_base_tf_.transform, base_to_camera);
    
    // 获取从camera_link到base_link的逆变换
    tf2::Transform camera_to_base = base_to_camera.inverse();
    
    // 应用变换：camera_link -> base_link
    tf2::Vector3 base_vec = camera_to_base * camera_vec;
    
    base_point.x = base_vec.x();
    base_point.y = base_vec.y();
    base_point.z = base_vec.z();
    
    return base_point;
}

void Yolo_Det_Node::publish(const std::vector<Detection>& dets,
                const std_msgs::msg::Header& header)   // 把图像时间戳/坐标系带过来
{
    vision_msgs::msg::Detection2DArray arr;
    arr.header = header;

    for (const auto& d : dets)
    {
        vision_msgs::msg::Detection2D det;
        det.bbox.center.position.x = d.bbox[0];
        det.bbox.center.position.y = d.bbox[1];
        det.bbox.size_x              = d.bbox[2];
        det.bbox.size_y              = d.bbox[3];

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = class_names[d.class_id];
        hyp.hypothesis.score = d.conf;
        
        // 新增：使用深度图像计算3D坐标并转换到base_link坐标系
        if (!depth_frame_.empty()) 
        {
            // 计算camera_link下的3D坐标
            geometry_msgs::msg::Point camera_point = calculate_3d_position(d);
            // std::cout << "camera_point:" << camera_point.x << "," << camera_point.y << "," << camera_point.z << std::endl;
            // 转换为base_link坐标系
            // geometry_msgs::msg::Point base_point = transform_to_base_link(camera_point);
            geometry_msgs::msg::Point base_point = camera_point;
            // std::cout << "base_point:" << base_point.x << "," << base_point.y << "," << base_point.z << std::endl;
            // 填充到hyp.pose.pose.position
            hyp.pose.pose.position = base_point;
            
            // 设置姿态（默认朝向）
            hyp.pose.pose.orientation.x = 0.0;
            hyp.pose.pose.orientation.y = 0.0;
            hyp.pose.pose.orientation.z = 0.0;
            hyp.pose.pose.orientation.w = 1.0;
            
            RCLCPP_DEBUG(this->get_logger(), "检测目标3D坐标 - camera_link: (%.3f, %.3f, %.3f), base_link: (%.3f, %.3f, %.3f)",
                        camera_point.x, camera_point.y, camera_point.z,
                        base_point.x, base_point.y, base_point.z);
        }
        else
        {
            // 如果没有深度图像，使用原有的3D坐标
            hyp.pose.pose.position.x = d.x3d;                  // 单位：米
            hyp.pose.pose.position.y = d.y3d;
            hyp.pose.pose.position.z = d.z3d;
            hyp.pose.pose.orientation.x = 0.0;
            hyp.pose.pose.orientation.y = 0.0;
            hyp.pose.pose.orientation.z = 0.0;
            hyp.pose.pose.orientation.w = 1.0;
        }
        
        det.results.push_back(hyp);

        arr.detections.push_back(det);
    }
    det_res_pub_->publish(arr);
}

void test()
{
    const std::vector<std::string> class_names = 
    {
    "standard_button", "alarm", "close", "down", "open", "stop", "up", "updown"
    };
  std::string engine_path = "install/elevator_recognition/models/det_rt10.engine";
  std::string image_path = "src/elevator_rec/src/elevator_recognition/test_imgs/02.jpg";
  std::string output_path = "output.jpg";
  // 加载检测器
  YoloDet detector;
  detector.load_engine(engine_path);

  // 读取图像
  cv::Mat img = cv::imread(image_path);
  if (img.empty()) 
  {
      throw std::runtime_error("Failed to load image: " + image_path);
  }

  // 执行检测
  auto start = std::chrono::high_resolution_clock::now();
  int nCount = 100;
  std::vector<Detection> detections;
  for(int i =0; i< nCount; i++)
  {
      detections = detector.infer(img);
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  double ave_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()/(float)nCount;
  // 输出结果
  // printf("Detected %d objects in %.2f ms\n",detections.size(), ave_time);
  std::cout << "Detected " << detections.size() << " objects in "
            << ave_time << "ms" << std::endl;
  if (!detections.empty()) 
  {
      printf("==== 检测结果 ====\n");
      for (const auto& d : detections)
          printf("Box: [%.2f,%.2f,%.2f,%.2f] Conf=%.3f class=%s\n",d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3], d.conf, class_names[d.class_id].c_str());
  } 
  else 
      std::cout << "未检测到目标\n";
  // 绘制结果
  detector.draw_results(img, detections, class_names);
  
  // 保存/显示结果
  cv::imwrite(output_path, img);
  std::cout << "结果已保存到 output.jpg" << std::endl;
}

int main(int argc, char **argv)
{
  if(1)
  {
    test();
  }
  else
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Yolo_Det_Node>());
    rclcpp::shutdown();
  }
  return 0;
}
