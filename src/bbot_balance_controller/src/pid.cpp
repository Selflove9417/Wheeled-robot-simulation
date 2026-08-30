#include "bbot_balance_controller/pid.h"

// 限幅函数
// 作用：把输入值 amt 限制在 [low, high] 范围内

static float constrain_val(float amt, float low, float high)
{
    if (amt < low)
        return low;

    if (amt > high)
        return high;

    return amt;
}

// PID 控制器构造函数
//
// 参数说明：
// p     ：比例系数 P
// i     ：积分系数 I
// d     ：微分系数 D
// ramp  ：输出变化率限制，防止输出突变
// lim   ：PID 输出最大幅值限制
//
// 成员变量说明：
// error_prev      ：上一次误差
// output_prev     ：上一次输出
// integral_prev   ：上一次积分项
// derivative_prev ：上一次滤波后的微分项

PIDController::PIDController(float p, float i, float d, float ramp, float lim)
    : P(p), I(i), D(d),
      output_ramp(ramp),
      limit(lim),
      error_prev(0.0f),
      output_prev(0.0f),
      integral_prev(0.0f),
      derivative_prev(0.0f)
{
}

// PID 主计算函数
//
// 输入：
// error ：当前误差，一般为 目标值 - 实际值
// dt    ：本次控制周期，单位通常是秒
//
// 输出：
// PID 计算后的控制量
//
// 该函数重载了 operator()
// 所以可以像函数一样调用 PID 对象，例如：
// float u = pid(error, dt);

float PIDController::operator()(float error, float dt)
{

    // 1. 防止 dt 太小或异常
    //
    // 如果 dt 太小，微分项 (error - error_prev) / dt
    // 会变得非常大，导致控制输出突然爆炸
    // 所以这里给 dt 一个最小值保护

    if (dt <= 0.0001f)
        dt = 0.001f;

    // 2. 比例项 P
    //
    // 比例项直接与当前误差成正比
    // 误差越大，输出越大

    float proportional = P * error;

    // 3. 积分项 I
    //
    // 使用梯形积分法：
    // integral = 上一次积分 + I * dt * (当前误差 + 上一次误差) / 2
    //
    // 相比简单积分 I * error * dt，
    // 梯形积分更加平滑，数值稳定性更好

    float integral = integral_prev + I * dt * 0.5f * (error + error_prev);

    // 4. 积分限幅
    // float PIDController::operator()(float error, float dt)
    // 防止积分项无限累积，也就是防止“积分饱和”

    integral = constrain_val(integral, -limit / 3.0f, limit / 3.0f);

    // 5. 原始微分项
    //
    // 微分项根据误差变化速度计算：
    //
    // derivative_raw = 当前误差变化量 / 时间

    float derivative_raw = (error - error_prev) / dt;

    // 6. 微分低通滤波
    //
    // 原始微分项对噪声非常敏感，
    // IMU 角度、速度等传感器数据如果有抖动，
    // 微分项会被放大成明显的控制抖动
    //
    // 这里使用一阶低通滤波：
    //
    // derivative_filtered =
    //     alpha * 当前原始微分
    //   + (1 - alpha) * 上一次滤波后的微分
    //
    // alpha 越大：
    // 微分响应越快，但噪声更明显
    //
    // alpha 越小：
    // 微分更平滑，但响应更慢

    float alpha = 0.15f;

    float derivative_filtered =
        alpha * derivative_raw +
        (1.0f - alpha) * derivative_prev;

    // 7. 微分输出 D
    //
    // 将滤波后的误差变化率乘以 D 系数
    // 得到最终微分控制量

    float derivative = D * derivative_filtered;

    // 8. PID 三项相加
    //
    // output = P项 + I项 + D项

    float output = proportional + integral + derivative;

    // 9. 总输出限幅
    //
    // 防止控制量超过允许范围
    //
    // 比如输出最终要转成电机速度、电机电流或力矩，
    // 那么这里的 limit 就相当于最大允许控制输出

    output = constrain_val(output, -limit, limit);

    // 10. 输出斜率限制 output_ramp
    //
    // 作用：
    // 限制 PID 输出变化速度，避免输出突变
    //
    // 如果 output_ramp > 0，说明启用输出变化率限制
    //
    // output_rate 表示当前输出相对于上一次输出的变化速度：
    //
    // output_rate = (output - output_prev) / dt
    //
    // 如果变化太快，就按照 output_ramp 限制它

    if (output_ramp > 0.0f)
    {
        float output_rate = (output - output_prev) / dt;

        // 正方向变化太快，限制最大上升速度
        if (output_rate > output_ramp)
            output = output_prev + output_ramp * dt;

        // 负方向变化太快，限制最大下降速度
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp * dt;
    }

    // 11. 保存当前状态，供下一次 PID 计算使用
    //
    // integral_prev   ：保存积分项
    // output_prev     ：保存输出值
    // error_prev      ：保存当前误差
    // derivative_prev ：保存滤波后的微分项

    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    derivative_prev = derivative_filtered;

    // 返回最终控制输出
    return output;
}

// PID 复位函数
//
// 作用：
// 清空 PID 的历史状态

void PIDController::reset()
{
    integral_prev = 0.0f;
    output_prev = 0.0f;
    error_prev = 0.0f;
    derivative_prev = 0.0f;
}