/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file ros1_serial_msckf.cpp
 * @brief ROS1串行处理节点：从ROS bag文件中读取IMU和相机数据并串行处理
 * 
 * 该节点的主要功能：
 * 1. 从ROS bag文件中读取IMU和相机数据
 * 2. 按照时间顺序串行处理这些数据
 * 3. 支持单目和双目相机配置
 * 4. 可选地使用真值数据进行系统初始化
 * 5. 处理完成后进行最终的可视化输出
 * 
 * 使用方式：
 * - 通过命令行参数或ROS参数服务器指定配置文件路径
 * - 通过ROS参数服务器配置IMU话题、相机话题、bag文件路径等
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <memory>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS1Visualizer.h"
#include "utils/dataset_reader.h"

using namespace ov_msckf;

// VIO系统管理器，负责视觉惯性里程计的核心处理
std::shared_ptr<VioManager> sys;
// ROS1可视化器，负责发布可视化消息和结果
std::shared_ptr<ROS1Visualizer> viz;

/**
 * 主函数：从ROS bag文件中串行读取IMU和相机数据并处理
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 */
int main(int argc, char **argv) {

  // 确保我们有配置文件路径，如果用户通过命令行传入则使用它
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  // 初始化ROS节点
  ros::init(argc, argv, "ros1_serial_msckf");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  // 从ROS参数服务器获取配置文件路径（如果存在）
  nh->param<std::string>("config_path", config_path, config_path);

  // 加载配置文件
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node_handler(nh);

  // 设置日志详细程度
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // 创建VIO系统
  VioManagerOptions params;
  params.print_and_load(parser);
  // params.num_opencv_threads = 0; // 如果需要可重复性，取消注释此行
  // params.use_multi_threading_pubs = 0; // 如果需要可重复性，取消注释此行
  params.use_multi_threading_subs = false; // 禁用订阅的多线程处理
  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<ROS1Visualizer>(nh, sys);

  // 确保所有必需的参数都已成功读取
  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  // 参数配置部分
  //===================================================================================

  // 获取IMU话题名称
  std::string topic_imu;
  nh->param<std::string>("topic_imu", topic_imu, "/imu0");
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  PRINT_DEBUG("[SERIAL]: imu: %s\n", topic_imu.c_str());

  // 获取所有相机话题名称
  std::vector<std::string> topic_cameras;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic;
    nh->param<std::string>("topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
    PRINT_DEBUG("[SERIAL]: cam: %s\n", cam_topic.c_str());
  }

  // 获取要读取的ROS bag文件路径
  std::string path_to_bag;
  nh->param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/eth/V1_01_easy.bag");
  PRINT_DEBUG("[SERIAL]: ros bag path is: %s\n", path_to_bag.c_str());

  // 如果提供了真值文件，则加载它
  // 注意：需要是CSV格式的ASL格式文件
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  if (nh->hasParam("path_gt")) {
    std::string path_to_gt;
    nh->param<std::string>("path_gt", path_to_gt, "");
    if (!path_to_gt.empty()) {
      ov_core::DatasetReader::load_gt_file(path_to_gt, gt_states);
      PRINT_DEBUG("[SERIAL]: gt file path is: %s\n", path_to_gt.c_str());
    }
  }

  // 获取bag文件的起始位置和处理时长
  // 如果bag_durr < 0，则处理到bag文件末尾
  double bag_start, bag_durr;
  nh->param<double>("bag_start", bag_start, 0);
  nh->param<double>("bag_durr", bag_durr, -1);
  PRINT_DEBUG("[SERIAL]: bag start: %.1f\n", bag_start);
  PRINT_DEBUG("[SERIAL]: bag duration: %.1f\n", bag_durr);

  //===================================================================================
  // ROS bag文件加载和消息提取部分
  //===================================================================================

  // 打开并加载ROS bag文件，查找可以播放的消息
  rosbag::Bag bag;
  bag.open(path_to_bag, rosbag::bagmode::Read);

  // 将bag文件加载为视图
  // 这里我们从bag文件的开头到结尾创建视图
  rosbag::View view_full;
  rosbag::View view;

  // 从完整视图时间开始几秒后开始
  // 如果持续时间为负数，则使用完整的bag长度
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start); // 添加起始偏移
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
  PRINT_DEBUG("time start = %.6f\n", time_init.toSec());
  PRINT_DEBUG("time end   = %.6f\n", time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);

  // 检查确保有数据可以播放
  if (view.size() == 0) {
    PRINT_ERROR(RED "[SERIAL]: No messages to play on specified topics.  Exiting.\n" RESET);
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // 遍历并收集所有消息到列表中
  // 这样做是为了能够访问bag文件中的任意点
  // 注意：如果在这里实例化消息，需要读取整个bag文件
  // 注意：因此我们只检查话题，这样可以快速遍历索引
  // 注意：参见此PR https://github.com/ros/ros_comm/issues/117
  PRINT_INFO("[SERIAL]: 正在读取 bag 文件并收集消息，这可能需要一些时间...\n");
  PRINT_INFO("[SERIAL]: 请耐心等待，程序正在扫描 bag 文件中的所有消息...\n");
  double max_camera_time = -1; // 记录相机消息的最大时间戳
  std::vector<rosbag::MessageInstance> msgs;
  size_t msg_count = 0;
  
  // 先尝试获取总消息数（这可能需要一些时间，但可以显示进度）
  PRINT_INFO("[SERIAL]: 正在计算总消息数...\n");
  size_t total_msgs = view.size(); // 获取总消息数（可能需要一些时间）
  PRINT_INFO("[SERIAL]: 找到 %zu 条消息，开始收集相关消息...\n", total_msgs);
  
  for (const rosbag::MessageInstance &msg : view) {
    msg_count++;
    // 每处理 10000 条消息打印一次进度
    if (msg_count % 10000 == 0) {
      PRINT_INFO("[SERIAL]: 已扫描 %zu / %zu 条消息 (%.1f%%)\n", 
                 msg_count, total_msgs, 100.0 * msg_count / total_msgs);
    }
    if (!ros::ok())
      break;
    // 收集IMU消息
    if (msg.getTopic() == topic_imu) {
      // if (msg.instantiate<sensor_msgs::Imu>() == nullptr) {
      //   PRINT_ERROR(RED "[SERIAL]: IMU topic has unmatched message types!!\n" RESET);
      //   PRINT_ERROR(RED "[SERIAL]: Supports: sensor_msgs::Imu\n" RESET);
      //   return EXIT_FAILURE;
      // }
      msgs.push_back(msg);
    }
    // 收集所有相机的消息
    for (int i = 0; i < params.state_options.num_cameras; i++) {
      if (msg.getTopic() == topic_cameras.at(i)) {
        // sensor_msgs::CompressedImage::ConstPtr img_c = msg.instantiate<sensor_msgs::CompressedImage>();
        // sensor_msgs::Image::ConstPtr img_i = msg.instantiate<sensor_msgs::Image>();
        // if (img_c == nullptr && img_i == nullptr) {
        //   PRINT_ERROR(RED "[SERIAL]: Image topic has unmatched message types!!\n" RESET);
        //   PRINT_ERROR(RED "[SERIAL]: Supports: sensor_msgs::Image and sensor_msgs::CompressedImage\n" RESET);
        //   return EXIT_FAILURE;
        // }
        msgs.push_back(msg);
        max_camera_time = std::max(max_camera_time, msg.getTime().toSec());
      }
    }
  }
  PRINT_INFO("[SERIAL]: 消息收集完成！共收集到 %zu 条相关消息（IMU + 相机）\n", msgs.size());
  PRINT_INFO("[SERIAL]: 开始处理消息...\n");

  //===================================================================================
  // 消息处理循环部分
  //===================================================================================

  // 遍历消息数组并处理它们
  PRINT_INFO("[SERIAL]: 开始处理 %zu 条消息...\n", msgs.size());
  std::set<int> used_index; // 记录已使用的消息索引，避免重复处理
  size_t processed_count = 0;
  size_t camera_frame_count = 0;
  
  for (int m = 0; m < (int)msgs.size(); m++) {
    processed_count++;
    
    // 每处理 1000 条消息显示一次进度（对于大量IMU消息很有用）
    if (processed_count % 1000 == 0) {
      double progress = 100.0 * processed_count / msgs.size();
      PRINT_INFO("[SERIAL]: 处理进度: %zu / %zu (%.1f%%) | 已处理 %zu 帧图像\n", 
                 processed_count, msgs.size(), progress, camera_frame_count);
    }

    // 如果到达结束时间或超过最大相机时间，则结束循环
    // 如果消息时间早于开始时间，则跳过（通常不应该发生）
    if (!ros::ok() || msgs.at(m).getTime() > time_finish || msgs.at(m).getTime().toSec() > max_camera_time)
      break;
    if (msgs.at(m).getTime() < time_init)
      continue;

    // 跳过已经使用过的消息
    if (used_index.find(m) != used_index.end()) {
      used_index.erase(m);
      continue;
    }

    // IMU消息处理
    if (msgs.at(m).getTopic() == topic_imu) {
      // PRINT_DEBUG("processing imu = %.3f sec\n", msgs.at(m).getTime().toSec() - time_init.toSec());
      viz->callback_inertial(msgs.at(m).instantiate<sensor_msgs::Imu>());
    }

    // 相机消息处理
    for (int cam_id = 0; cam_id < params.state_options.num_cameras; cam_id++) {

      // 如果这个消息不是当前相机的话题，则跳过
      if (msgs.at(m).getTopic() != topic_cameras.at(cam_id))
        continue;

      // 找到匹配的相机话题后，查找同一时刻的其他相机消息
      // 对于每个相机，我们将查找时间戳最接近的消息（在0.02秒内）
      // 如果无法找到，则跳过此消息，因为它不是同步的图像对！
      std::map<int, int> camid_to_msg_index; // 相机ID到消息索引的映射
      double meas_time = msgs.at(m).getTime().toSec();
      for (int cam_idt = 0; cam_idt < params.state_options.num_cameras; cam_idt++) {
        if (cam_idt == cam_id) {
          // 当前相机直接使用当前消息索引
          camid_to_msg_index.insert({cam_id, m});
          continue;
        }
        // 查找其他相机在同一时刻的消息
        int cam_idt_idx = -1;
        for (int mt = m; mt < (int)msgs.size(); mt++) {
          if (msgs.at(mt).getTopic() != topic_cameras.at(cam_idt))
            continue;
          if (std::abs(msgs.at(mt).getTime().toSec() - meas_time) < 0.02)
            cam_idt_idx = mt;
          break;
        }
        if (cam_idt_idx != -1) {
          camid_to_msg_index.insert({cam_idt, cam_idt_idx});
        }
      }

      // 如果无法找到所有相机的消息，则跳过处理
      if ((int)camid_to_msg_index.size() != params.state_options.num_cameras) {
        PRINT_DEBUG(YELLOW "[SERIAL]: Unable to find stereo pair for message %d at %.2f into bag (will skip!)\n" RESET, m,
                    meas_time - time_init.toSec());
        continue;
      }

      // 检查是否应该使用真值进行初始化
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() && ov_core::DatasetReader::get_gt_state(meas_time, imustate, gt_states)) {
        // 通常偏置值很差，所以将它们置零
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      }

      // 将数据传递给可视化器的回调函数！
      camera_frame_count++;
      
      // 在初始化阶段显示提示
      if (!sys->initialized() && camera_frame_count % 10 == 0) {
        PRINT_INFO("[SERIAL]: 正在初始化... 已处理 %zu 帧图像，等待足够的特征点...\n", camera_frame_count);
      }
      
      // PRINT_DEBUG("processing cam = %.3f sec\n", msgs.at(m).getTime().toSec() - time_init.toSec());
      if (params.state_options.num_cameras == 1) {
        // 单目相机处理
        viz->callback_monocular(msgs.at(camid_to_msg_index.at(0)).instantiate<sensor_msgs::Image>(), 0);
      } else if (params.state_options.num_cameras == 2) {
        // 双目相机处理
        auto msg0 = msgs.at(camid_to_msg_index.at(0));
        auto msg1 = msgs.at(camid_to_msg_index.at(1));
        used_index.insert(camid_to_msg_index.at(0)); // 标记此消息已使用，跳过
        used_index.insert(camid_to_msg_index.at(1)); // 标记此消息已使用，跳过
        viz->callback_stereo(msg0.instantiate<sensor_msgs::Image>(), msg1.instantiate<sensor_msgs::Image>(), 0, 1);
      } else {
        PRINT_ERROR(RED "[SERIAL]: We currently only support 1 or 2 camera serial input....\n" RESET);
        return EXIT_FAILURE;
      }
      
      // 初始化完成后显示提示
      if (sys->initialized() && camera_frame_count % 100 == 0) {
        PRINT_INFO("[SERIAL]: 系统已初始化！已处理 %zu 帧图像\n", camera_frame_count);
      }

      break;
    }
  }

  PRINT_INFO("[SERIAL]: 消息处理完成！共处理 %zu 条消息，%zu 帧图像\n", processed_count, camera_frame_count);
  PRINT_INFO("[SERIAL]: 系统初始化状态: %s\n", sys->initialized() ? "已初始化" : "未初始化");

  // 最终可视化：打印最终的校准参数、性能指标和运行时间
  // 注意：此函数不会阻塞线程，只是同步地执行打印操作，执行时间很短
  // 主要输出内容：
  // - 相机-IMU时间偏移、相机内外参、IMU内参等校准结果（如果启用）
  // - RMSE和NEES性能指标（如果提供了真值数据）
  // - 总运行时间
  viz->visualize_final();

  // 完成！
  return EXIT_SUCCESS;
}
