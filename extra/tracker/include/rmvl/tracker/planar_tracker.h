/**
 * @file planar_tracker.h
 * @author RoboMaster Vision Community
 * @brief 平面目标追踪器头文件
 * @version 1.0
 * @date 2021-08-21
 *
 * @copyright Copyright 2021 (c), RoboMaster Vision Community
 *
 */

#pragma once

#include "rmvl/algorithm/kalman.hpp"
#include "rmvl/combo/armor.h"

#include "tracker.h"

namespace rm
{

//! @addtogroup planar_tracker
//! @{

//! 平面目标追踪器
class RMVL_EXPORTS_W_DES PlanarTracker final : public tracker
{
public:
    using ptr = std::shared_ptr<PlanarTracker>;
    using const_ptr = std::shared_ptr<const PlanarTracker>;

    //! @cond
    explicit PlanarTracker(combo::ptr p_combo);
    //! @endcond

    /**
     * @brief 构建 PlanarTracker
     *
     * @param[in] p_combo 第一帧平面目标组合特征（不允许为空）
     */
    RMVL_W static inline ptr make_tracker(combo::ptr p_combo) { return std::make_shared<PlanarTracker>(p_combo); }

    /**
     * @brief 从另一个追踪器进行构造
     *
     * @return 指向新追踪器的共享指针
     */
    RMVL_W tracker::ptr clone() override;

    RMVL_TRACKER_CAST(PlanarTracker)

    /**
     * @brief 丢失目标时，使用时间点和 IMU 数据更新平面目标追踪器
     *
     * @param[in] tick 时间点
     * @param[in] imu_data IMU 数据
     */
    RMVL_W void update(double tick, const ImuData &imu_data) override;

    /**
     * @brief 使用捕获的 `combo` 更新平面目标追踪器
     *
     * @param[in] p_combo 待传入 tracker 的平面目标，必须严格保证不为空
     */
    RMVL_W void update(combo::ptr p_combo) override;

    //! 判断追踪器是否无效
    RMVL_W bool invalid() const override;

private:
    /**
     * @brief 将 combo 中的数据更新至 tracker
     *
     * @param[in] p_combo combo::ptr 指针
     */
    void updateData(combo::ptr p_combo);

    //! 初始化 tracker 的距离和运动滤波器
    void initFilter();

    /**
     * @brief 更新距离滤波器
     */
    void updateDistanceFilter();

    /**
     * @brief 更新运动滤波器
     * @note 帧差时间 t: (若只有一帧则取默认采样时间，否则取平均数值)
     */
    void updateMotionFilter();

    KF21f _distance_filter; //!< 距离滤波器
    KF42f _motion_filter;   //!< 运动滤波器
};

//! @} planar_tracker

} // namespace rm
