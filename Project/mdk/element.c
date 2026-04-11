#include "element.h"
#include "zf_common_headfile.h"

// 假设定义了全局的当前元素状态
Element_Type current_element = Normal;

// 元素状态机的阶段标志位
uint8 seesaw_state = 0; // 跷跷板状态
uint8 loop_state = 0;   // 环岛状态
uint8 cross_state = 0;  // 十字状态
uint8 roller_state = 0; // 滚筒状态

// 陀螺仪积分阈值（用于判断动作是否完成）
float element_yaw_integral = 0;

/**
 * @brief 赛道元素综合检测函数
 * @param L1, L2, L3, L4 归一化后的电感值 (0-100)
 */
void Element_Detect(uint16 *inductance_norm_data)
{
    // ==========================================================
    // 1. 立体元素检测 (最高优先级，依赖 IMU 数据)
    // ==========================================================

    // 【垂直面 / 飞檐走壁】
    // 飞檐走壁时，车身会经历极大的俯仰角变化（接近垂直）或者极大的横滚角（贴壁跑）
    if (first_complement.angle.pitch > 60.0f || first_complement.angle.pitch < -60.0f || first_complement.angle.pitch > 60.0f || first_complement.angle.pitch < -60.0f)
    {
        current_element = Vertical;
        return;
    }

    // 【跷跷板】
    // 特征：车头上坡(Pitch变正) -> 越过重心点砸板(Pitch瞬间变负) -> 落地平稳(Pitch归零)
    if (seesaw_state == 0 && first_complement.angle.pitch > 12.0f && first_complement.angle.pitch < 25.0f) // 上板阶段
    {
        seesaw_state = 1;
        current_element = Seesaw;
    }
    else if (seesaw_state == 1 && first_complement.angle.pitch < -15.0f) // 越过平衡点，砸板向下
    {
        seesaw_state = 2;
    }
    else if (seesaw_state == 2 && first_complement.angle.pitch > -5.0f && first_complement.angle.pitch < 5.0f) // 下板完毕，恢复平地
    {
        seesaw_state = 0;
        current_element = Normal;
    }

    // 【滚筒路面】
    // 特征：Z轴加速度（或X轴/Y轴高频振动）出现连续的剧烈波动，同时 pitch 变化不规则
    // 需要结合 IMU_Data.acc_z 的方差或高频分量来判断
    if (My_abs(IMU_Data.acc_z - 1.0f) > 0.5f) // 假设1.0g为静止状态，剧烈颠簸时Z轴加速度异常
    {
        roller_state++;
        if (roller_state > 10) // 持续颠簸
        {
            current_element = Roller;
        }
    }
    else
    {
        roller_state = 0;
        if (current_element == Roller)
            current_element = Normal; // 驶出滚筒
    }

    // ==========================================================
    // 2. 平面元素检测 (次优先级，依赖电磁数据)
    // ==========================================================

    // 如果已经在立体元素中，则屏蔽平面元素的误判
    if (current_element == Vertical || current_element == Seesaw)
        return;

    // 【十字路口】
    // 特征：四个电感同时变大，且内侧电感达到极值
    if (cross_state == 0 && L1 > 60 && L2 > 80 && L3 > 80 && L4 > 60)
    {
        cross_state = 1; // 识别到十字
        current_element = Crossroads;
        // 执行十字策略：通常是强制锁死方向环（输出0），靠惯性直行冲过去
    }
    else if (cross_state == 1 && L2 < 50 && L3 < 50) // 驶出十字中心
    {
        cross_state = 0;
        current_element = Normal;
    }

    // 【环岛 / 六边形环岛】
    // 环岛的电磁特征非常明显的不对称性。以左环岛为例：L1激增，L4减小，然后L2变大
    // 这里使用状态机处理：0发现，1入环，2环中，3出环
    if (loop_state == 0 && L1 > 85 && L2 > 60 && L4 < 30) // 远端发现左环岛
    {
        loop_state = 1;
        current_element = Loop;
        element_yaw_integral = 0; // 清零陀螺仪积分，准备计算转过的角度
    }
    else if (loop_state == 1) // 准备打角入环
    {
        // 累加 Z 轴角速度，判断是否完成入环 (大约转过 45~90 度)
        element_yaw_integral += Daty_Z;
        if (element_yaw_integral > 45.0f)
        {
            loop_state = 2; // 进入环中
        }
    }
    else if (loop_state == 2) // 环中巡航
    {
        // 环岛内通常一侧电感一直很大。当转过接近 360 度时准备出环
        element_yaw_integral += IMU_Data.gyro_z * 0.005;
        if (element_yaw_integral > 270.0f && L1 > 70) // 遇到出环口的电磁线
        {
            loop_state = 3; // 准备出环
        }
    }
    else if (loop_state == 3) // 出环
    {
        element_yaw_integral += IMU_Data.gyro_z * 0.005;
        if (element_yaw_integral > 340.0f) // 彻底转完一圈
        {
            loop_state = 0;
            current_element = Normal;
        }
    }
}