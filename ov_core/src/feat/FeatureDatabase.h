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

#ifndef OV_CORE_FEATURE_DATABASE_H
#define OV_CORE_FEATURE_DATABASE_H

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace ov_core {

class Feature;

/**
 * @brief 包含当前正在跟踪的特征的数据库
 * Database containing features we are currently tracking.
 *
 * 每个视觉跟踪器都包含此数据库，它存储所有正在跟踪的特征。
 * 跟踪器在进行跟踪获得新测量时会将信息插入此数据库。
 * 用户可以查询此数据库以获取可用于更新的特征，并在处理完成后移除它们。
 * Each visual tracker has this database in it and it contains all features that we are tracking.
 * The trackers will insert information into this database when they get new measurements from doing tracking.
 * A user would then query this database for features that can be used for update and remove them after they have been processed.
 *
 *
 * @m_class{m-note m-warning}
 *
 * @par 关于多线程支持的说明
 * A Note on Multi-Threading Support
 * 支持异步多线程访问。
 * 由于每个特征是指针，直接返回和使用它们不是线程安全的。
 * 因此，要保证线程安全，请使用每个函数的"remove"标志，这将从特征数据库中移除该特征。
 * 这可以防止跟踪器添加新测量和编辑特征信息。
 * 例如，如果你正在异步跟踪相机并选择更新状态，那么移除你将在更新中使用的所有特征。
 * 特征跟踪器将在你更新时继续添加特征，这些测量可以在下一步更新中使用！
 * There is some support for asynchronous multi-threaded access.
 * Since each feature is a pointer just directly returning and using them is not thread safe.
 * Thus, to be thread safe, use the "remove" flag for each function which will remove it from this feature database.
 * This prevents the trackers from adding new measurements and editing the feature information.
 * For example, if you are asynchronous tracking cameras and you chose to update the state, then remove all features you will use in update.
 * The feature trackers will continue to add features while you update, whose measurements can be used in the next update step!
 *
 */
class FeatureDatabase {

public:
  /**
   * @brief 默认构造函数
   * Default constructor
   */
  FeatureDatabase() {}

  /**
   * @brief 获取指定的特征
   * Get a specified feature
   * @param id 要获取的特征ID
   * What feature we want to get
   * @param remove 如果设置为true，将从数据库中移除该特征（你需要负责释放内存）
   * Set to true if you want to remove the feature from the database (you will need to handle the freeing of memory)
   * @return 特征对象，如果不在数据库中则返回null
   * Either a feature object, or null if it is not in the database.
   */
  std::shared_ptr<Feature> get_feature(size_t id, bool remove = false);

  /**
   * @brief 获取指定特征的克隆（指针是线程安全的）
   * Get a specified feature clone (pointer is thread safe)
   * @param id 要获取的特征ID
   * What feature we want to get
   * @param feat 用于存储特征数据的对象
   * Feature with data in it
   * @return 如果找到特征则返回true
   * True if the feature was found
   */
  bool get_feature_clone(size_t id, Feature &feat);

  /**
   * @brief 更新特征对象
   * Update a feature object
   * @param id 要更新的特征ID
   * ID of the feature we will update
   * @param timestamp 测量发生的时间
   * time that this measurement occured at
   * @param cam_id 测量来自哪个相机
   * which camera this measurement was from
   * @param u 原始u坐标
   * raw u coordinate
   * @param v 原始v坐标
   * raw v coordinate
   * @param u_n 去畸变/归一化的u坐标
   * undistorted/normalized u coordinate
   * @param v_n 去畸变/归一化的v坐标
   * undistorted/normalized v coordinate
   *
   * 这将根据传递的ID更新给定特征。
   * 如果这是一个我们之前未见过的ID，它将创建一个新特征。
   * This will update a given feature based on the passed ID it has.
   * It will create a new feature, if it is an ID that we have not seen before.
   */
  void update_feature(size_t id, double timestamp, size_t cam_id, float u, float v, float u_n, float v_n);

  /**
   * @brief 获取没有比指定时间更新的测量的特征
   * Get features that do not have newer measurement then the specified time.
   *
   * 此函数将返回所有在指定时间之后没有测量的特征。
   * 例如，这可用于获取未成功跟踪到最新帧的特征。
   * 返回的所有特征都不会有发生在指定时间之后的任何测量。
   * This function will return all features that do not a measurement at a time greater than the specified time.
   * For example this could be used to get features that have not been successfully tracked into the newest frame.
   * All features returned will not have any measurements occurring at a time greater then the specified.
   */
  std::vector<std::shared_ptr<Feature>> features_not_containing_newer(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief 获取具有比指定时间更旧的测量的特征
   * Get features that has measurements older then the specified time.
   *
   * 这将收集所有在指定时间戳之前有测量的特征。
   * 例如，我们可能想要移除所有比滑动窗口中最后一个克隆/状态更旧的特征。
   * This will collect all features that have measurements occurring before the specified timestamp.
   * For example, we would want to remove all features older then the last clone/state in our sliding window.
   */
  std::vector<std::shared_ptr<Feature>> features_containing_older(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief 获取在指定时间有测量的特征
   * Get features that has measurements at the specified time.
   *
   * 此函数将返回所有包含指定时间的特征。
   * 这可用于获取在特定克隆/状态下发生的所有特征。
   * This function will return all features that have the specified time in them.
   * This would be used to get all features that occurred at a specific clone/state.
   */
  std::vector<std::shared_ptr<Feature>> features_containing(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief 删除所有已用完的特征
   * This function will delete all features that have been used up.
   *
   * 如果特征无法使用，它仍将保留，因为它不会有删除标志设置
   * If a feature was unable to be used, it will still remain since it will not have a delete flag set
   */
  void cleanup();

  /**
   * @brief 删除所有比指定时间戳更旧的特征测量
   * This function will delete all feature measurements that are older then the specified timestamp
   */
  void cleanup_measurements(double timestamp);

  /**
   * @brief 删除所有在指定时间戳的特征测量
   * This function will delete all feature measurements that are at the specified timestamp
   */
  void cleanup_measurements_exact(double timestamp);

  /**
   * @brief 返回特征数据库的大小
   * Returns the size of the feature database
   */
  size_t size() {
    std::lock_guard<std::mutex> lck(mtx);
    return features_idlookup.size();
  }

  /**
   * @brief 返回内部数据（通常不应使用）
   * Returns the internal data (should not normally be used)
   */
  std::unordered_map<size_t, std::shared_ptr<Feature>> get_internal_data() {
    std::lock_guard<std::mutex> lck(mtx);
    return features_idlookup;
  }

  /**
   * @brief 获取数据库中最旧的时间
   * Gets the oldest time in the database
   */
  double get_oldest_timestamp();

  /**
   * @brief 使用此数据库的最新特征信息更新传入的数据库
   * Will update the passed database with this database's latest feature information.
   */
  void append_new_measurements(const std::shared_ptr<FeatureDatabase> &database);

protected:
  /// 用于保护map的互斥锁
  /// Mutex lock for our map
  std::mutex mtx;

  /// 允许我们基于ID查询的查找数组
  /// Our lookup array that allow use to query based on ID
  std::unordered_map<size_t, std::shared_ptr<Feature>> features_idlookup;
};

} // namespace ov_core

#endif /* OV_CORE_FEATURE_DATABASE_H */