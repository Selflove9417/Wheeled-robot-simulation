#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace bbot_jump
{
// 插值只使用同一时钟的采样时间；禁止把最新关节角与旧里程计直接相减。
class JointPoseHistory
{
    struct Sample { double stamp; std::array<double, 4> q; };
    std::deque<Sample> samples_;
public:
    void push(double stamp, const std::array<double, 4> & q) {
        if (!std::isfinite(stamp)) return;
        for (double value : q) if (!std::isfinite(value)) return;
        if (!samples_.empty() && stamp < samples_.back().stamp) samples_.clear();
        if (!samples_.empty() && stamp == samples_.back().stamp) samples_.back() = {stamp,q};
        else samples_.push_back({stamp,q});
        while (samples_.size()>64) samples_.pop_front();
    }
    bool interpolate(double stamp, std::array<double,4> & q) const {
        if (!std::isfinite(stamp) || samples_.empty() ||
            stamp < samples_.front().stamp || stamp > samples_.back().stamp) return false;
        for (size_t i=0; i<samples_.size(); ++i) {
            if (std::abs(stamp-samples_[i].stamp)<1e-8) { q=samples_[i].q; return true; }
            if (samples_[i].stamp>stamp && i>0) {
                const auto & a=samples_[i-1]; const auto & b=samples_[i];
                if (b.stamp-a.stamp>0.030) return false;
                const double u=(stamp-a.stamp)/(b.stamp-a.stamp);
                for (size_t j=0;j<4;++j) q[j]=a.q[j]+u*(b.q[j]-a.q[j]);
                return true;
            }
        }
        return false;
    }
};

class TakeoffConfirmation
{
    double last_stamp_ = -1.0;
    double first_stamp_ = -1.0;
    int count_ = 0;
public:
    void reset() { last_stamp_=-1.0; first_stamp_=-1.0; count_=0; }
    int count() const { return count_; }
    bool update(double stamp, double now, bool aligned_valid,
                double left_clearance, double right_clearance,
                bool unloaded, bool lift_motion_observed) {
        if (!aligned_valid || !std::isfinite(stamp) || now-stamp>0.080 || stamp>now+0.001) {
            if (last_stamp_>=0.0 && now-last_stamp_>0.080) reset();
            return false;
        }
        if (last_stamp_>=0.0 && stamp<last_stamp_) reset();
        if (stamp==last_stamp_) return false; // 200 Hz循环不能重复累计同一传感器帧
        if (last_stamp_>=0.0 && stamp-last_stamp_>0.040) count_=0;
        last_stamp_=stamp;
        const double clearance=std::min(left_clearance,right_clearance);
        // 仍以两轮12 mm净空建立证据；6 mm退出阈值防止毫米级抖动清零。
        const double threshold=count_>0 ? 0.006 : 0.012;
        // 已对齐的两轮净空均达到20mm时，不再由单次IMU内部反作用冲击否决。
        // 仍需要两个不同时间戳的净空样本；接地微弹性不能走此分支。
        if (!std::isfinite(left_clearance) || !std::isfinite(right_clearance) ||
            clearance<=threshold || (!unloaded && clearance<0.020) || !lift_motion_observed) {
            count_=0; first_stamp_=-1.0; return false;
        }
        if (count_==0) first_stamp_=stamp;
        ++count_;
        return count_>=2 && stamp-first_stamp_>=0.010-1e-8;
    }
};
// Persistent wheel contact must be accepted even while the deploy trajectory
// is unfinished. Motor-reported effort may be zero in this simulator.
class TouchdownConfirmation {
    double last_stamp_=-1.0;
    double first_stamp_=-1.0;
    int count_=0;
public:
    void reset() { last_stamp_=-1.0; first_stamp_=-1.0; count_=0; }
    bool update(double stamp, double now, bool aligned, double clearance,
                bool descending, double specific_force) {
        if (!aligned || !std::isfinite(stamp) || now-stamp>0.080 || stamp>now+0.001) {
            if (last_stamp_>=0.0 && now-last_stamp_>0.080) reset();
            return false;
        }
        if (last_stamp_>=0.0 && stamp<last_stamp_) reset();
        if (stamp==last_stamp_) return false;
        if (last_stamp_>=0.0 && stamp-last_stamp_>0.040) count_=0;
        last_stamp_=stamp;
        if (!descending || !std::isfinite(clearance) || clearance>0.002 ||
            !std::isfinite(specific_force) || specific_force<3.0) {
            count_=0; first_stamp_=-1.0; return false;
        }
        if (count_==0) first_stamp_=stamp;
        ++count_;
        return count_>=2 && stamp-first_stamp_>=0.010-1e-8;
    }
};
} // namespace bbot_jump
