//
// VIO C API 实现 - 将 vio_interface_api.h 声明的接口转发到 VioInterface 类
//

#include "system/vio_interface_api.h"

#include <string>

#include "system/vio_interface.h"

vio_handle_t vio_create(const char* yaml_path) {
  auto* ptr =
      new VioInterface(yaml_path ? std::string(yaml_path) : std::string());
  return reinterpret_cast<vio_handle_t>(ptr);
}

int vio_init(vio_handle_t handle) {
  auto* ptr = reinterpret_cast<VioInterface*>(handle);
  return ptr->init();
}

void vio_reset(vio_handle_t handle) {
  auto* ptr = reinterpret_cast<VioInterface*>(handle);
  ptr->reset();
}

void vio_destroy(vio_handle_t handle) {
  auto* ptr = reinterpret_cast<VioInterface*>(handle);
  delete ptr;
}

void vio_push_imu(vio_handle_t handle, const vio_imu_msg_t* imu) {
  if (handle && imu) {
    reinterpret_cast<VioInterface*>(handle)->OnIMU(*imu);
  }
}

void vio_push_image(vio_handle_t handle, const vio_image_msg_t* img) {
  if (handle && img) {
    reinterpret_cast<VioInterface*>(handle)->OnImage(*img);
  }
}

void vio_register_state_callback(vio_handle_t handle,
                                 vio_state_callback_t callback,
                                 void* user_data) {
  if (handle && callback) {
    reinterpret_cast<VioInterface*>(handle)->RegisterStateCallback(callback,
                                                                   user_data);
  }
}
