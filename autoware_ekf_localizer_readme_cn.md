# 概述

**扩展卡尔曼滤波器定位器（Extend Kalman Filter Localizer）** 通过将二维车辆动力学模型与输入的自身位姿和自身速度信息相融合，估计出鲁棒性更强、噪声更小的机器人位姿和速度。该算法专为高速移动机器人（如自动驾驶系统）设计。

## 流程图

autoware_ekf_localizer 的整体流程图如下所示。

<p align="center">
  <img src="./media/ekf_flowchart.png" width="800">
</p>

## 功能特性

本软件包包含以下功能：

- **输入消息的时间延迟补偿**，能够正确整合具有不同时间延迟的输入信息。这对于高速移动机器人（如自动驾驶车辆）尤为重要（见下图）。
- **偏航偏置的自动估计**，可防止由传感器安装角度误差引起的建模误差，从而提高估计精度。
- **马氏距离门控**，实现概率性异常值检测，以决定哪些输入应被使用或忽略。
- **平滑更新**，卡尔曼滤波器的测量更新通常在获得测量值时执行，但这可能导致估计值发生较大变化，尤其是对于低频测量。由于算法可以考虑测量时间，因此可将测量数据分割成多个片段并平滑地整合，同时保持一致性（见下图）。
- **根据俯仰角计算垂直修正量**，可缓解斜坡上的定位不稳定性。例如，在上坡时，由于 EKF 仅考虑 3DoF（x， y， 偏航），会导致定位结果如同陷入地面（见“由俯仰角计算增量”图的左侧）。因此，EKF 根据公式修正 z 坐标（见该图的右侧）。

<p align="center">
<img src="./media/ekf_delay_comp.png" width="800">
</p>

<p align="center">
  <img src="./media/ekf_smooth_update.png" width="800">
</p>

<p align="center">
  <img src="./media/calculation_delta_from_pitch.png" width="800">
</p>

## 节点

### 订阅话题

| 名称                             | 类型                                             | 描述                                                                                                                              |
| -------------------------------- | ------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `measured_pose_with_covariance`  | `geometry_msgs::msg::PoseWithCovarianceStamped`  | 带有测量协方差矩阵的输入位姿源。                                                                                                                |
| `measured_twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | 带有测量协方差矩阵的输入速度源。                                                                                                               |
| `initialpose`                    | `geometry_msgs::msg::PoseWithCovarianceStamped`  | EKF 的初始位姿。估计位姿在启动时初始化为零。每当发布该消息时，将使用它初始化估计位姿。 |

### 发布话题

| 名称                              | 类型                                                | 描述                                           |
| --------------------------------- | --------------------------------------------------- | ----------------------------------------------------- |
| `ekf_odom`                        | `nav_msgs::msg::Odometry`                           | 估计的里程计。                                   |
| `ekf_pose`                        | `geometry_msgs::msg::PoseStamped`                   | 估计的位姿。                                       |
| `ekf_pose_with_covariance`        | `geometry_msgs::msg::PoseWithCovarianceStamped`     | 带有协方差的估计位姿。                       |
| `ekf_biased_pose`                 | `geometry_msgs::msg::PoseStamped`                   | 包含偏航偏置的估计位姿。                 |
| `ekf_biased_pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped`     | 包含偏航偏置的带有协方差的估计位姿。 |
| `ekf_twist`                       | `geometry_msgs::msg::TwistStamped`                  | 估计的速度。                                      |
| `ekf_twist_with_covariance`       | `geometry_msgs::msg::TwistWithCovarianceStamped`    | 带有协方差的估计速度。                  |
| `diagnostics`                     | `diagnostics_msgs::msg::DiagnosticArray`            | 诊断信息。                           |
| `debug/processing_time_ms`        | `autoware_internal_debug_msgs::msg::Float64Stamped` | 处理时间 [毫秒]。                             |

### 发布的 TF

- base_link
  从 `map` 坐标系到估计位姿的 TF。

## 功能

### 预测

使用给定的预测模型，根据先前估计的数据预测当前机器人状态。该计算以固定间隔（`predict_frequency [Hz]`）调用。预测方程在本页末尾给出。

### 测量更新

在更新之前，计算测量输入与预测状态之间的马氏距离；对于马氏距离超过给定阈值的输入，不执行测量更新。

预测状态会使用最新的测量输入（measured_pose 和 measured_twist）进行更新。更新与预测以相同的频率执行，通常为高频，以实现平滑的状态估计。

## 参数说明

参数在 `launch/ekf_localizer.launch` 中设置。

### 节点参数

{{ json_to_markdown("localization/autoware_ekf_localizer/schema/sub/node.sub_schema.json") }}

### 位姿测量参数

{{ json_to_markdown("localization/autoware_ekf_localizer/schema/sub/pose_measurement.sub_schema.json") }}

### 速度测量参数

{{ json_to_markdown("localization/autoware_ekf_localizer/schema/sub/twist_measurement.sub_schema.json") }}

### 过程噪声参数

{{ json_to_markdown("localization/autoware_ekf_localizer/schema/sub/process_noise.sub_schema.json") }}

注意：位置 x 和 y 的过程噪声由非线性动力学自动计算。

### 简单一维滤波器参数

{{ json_to_markdown("localization/autoware_ekf_localizer/schema/sub/simple_1d_filter_parameters.sub_schema.json") }}

### 诊断参数

{{ json_to_markdown("localization/autoware_ekf_localizer/schema/sub/diagnostics.sub_schema.json") }}

### 杂项参数

{{ json_to_markdown("localization/autoware_ekf_localizer/schema/sub/misc.sub_schema.json") }}

## 如何调整 EKF 参数

### 0. 预备工作

- 检查位姿和速度消息中的 header 时间是否已正确设置为传感器时间，因为时间延迟是根据该值计算的。如果由于定时器同步问题难以设置合适的时间，请使用 `twist_additional_delay` 和 `pose_additional_delay` 来校正时间。
- 检查测量位姿与速度之间的关系是否合理（即位姿的导数是否与速度值相近）。这种差异主要由单位错误（如混淆弧度/度）或偏置噪声引起，会导致较大的估计误差。

### 1. 调整传感器参数

为每个传感器设置标准差。`pose_measure_uncertainty_time` 用于 header 时间戳数据的不确定性。
您还可以通过调整 `*_smoothing_steps` 来调整每个观测传感器数据的平滑步数。
增加步数将提高估计的平滑性，但可能对估计性能产生不利影响。

- `pose_measure_uncertainty_time`
- `pose_smoothing_steps`
- `twist_smoothing_steps`

### 2. 调整过程模型参数

- `proc_stddev_vx_c`：设置为最大线加速度
- `proc_stddev_wz_c`：设置为最大角加速度
- `proc_stddev_yaw_c`：该参数描述偏航与偏航率之间的相关性。较大的值表示偏航变化与估计的偏航率不相关。若设为 0，则表示估计偏航的变化等于偏航率。通常应设为 0。
- `proc_stddev_yaw_bias_c`：该参数是偏航偏置变化率的标准差。大多数情况下，偏航偏置是恒定的，因此可以设置得很小，但必须非零。

### 3. 调整门控参数

EKF 在观测更新前使用马氏距离进行门控。门控大小由 `pose_gate_dist` 和 `twist_gate_dist` 参数决定。如果马氏距离大于该值，则忽略该观测。

该门控过程基于卡方分布的统计检验。按模型假设，位姿的马氏距离服从自由度为 3 的卡方分布，速度则服从自由度为 2 的卡方分布。

目前，协方差估计本身的精度并不很好，因此建议将显著性水平设置得非常小，以减少因误报导致的拒绝。

| 显著性水平 | 自由度 2 的阈值 | 自由度 3 的阈值 |
| ------------------ | ------------------- | ------------------- |
| $10 ^ {-2}$        | 9.21                | 11.3                |
| $10 ^ {-3}$        | 13.8                | 16.3                |
| $10 ^ {-4}$        | 18.4                | 21.1                |
| $10 ^ {-5}$        | 23.0                | 25.9                |
| $10 ^ {-6}$        | 27.6                | 30.7                |
| $10 ^ {-7}$        | 32.2                | 35.4                |
| $10 ^ {-8}$        | 36.8                | 40.1                |
| $10 ^ {-9}$        | 41.4                | 44.8                |
| $10 ^ {-10}$       | 46.1                | 49.5                |

## 卡尔曼滤波器模型

### 更新函数中的运动学模型

<img src="./media/ekf_dynamics.png" width="320">

其中，$\theta_k$ 表示车辆的航向角（包括安装角偏置）。
$b_k$ 是偏航偏置的修正项，建模使得 $(\theta_k+b_k)$ 成为 base_link 的航向角。
位姿估计器预期发布 map 坐标系下的 base_link。然而，由于标定误差，偏航角可能存在偏移。该模型补偿此误差，提高估计精度。

### 时间延迟模型

测量时间延迟通过增广状态处理 [1]（参见第 7.3 节“固定滞后平滑”）。

<img src="./media/delay_model_eq.png" width="320">

注意，尽管由于增广状态的特定结构，可应用解析展开，维度会增加，但计算复杂度不会显著增加。

## 与 Autoware NDT 的测试结果

<p align="center">
<img src="./media/ekf_autoware_res.png" width="600">
</p>

## 诊断

<p align="center">
<img src="./media/ekf_diagnostics.png" width="320">
</p>

<p align="center">
<img src="./media/ekf_diagnostics_callback_pose.png" width="320">
</p>
<p align="center">
<img src="./media/ekf_diagnostics_callback_twist.png" width="320">
</p>

### 导致 WARN 状态的条件

- 节点未处于激活状态。
- 通过位姿/速度话题连续无测量更新的次数超过 `pose_no_update_count_threshold_warn` / `twist_no_update_count_threshold_warn`。
- 位姿/速度话题的时间戳超出延迟补偿范围。
- 位姿/速度话题的马氏距离超出协方差估计范围。
- 协方差椭圆的长轴大于阈值 `warn_ellipse_size`，或横向大于 `warn_ellipse_size_lateral_direction`。

### 导致 ERROR 状态的条件

- 未设置初始位姿。
- 通过位姿/速度话题连续无测量更新的次数超过 `pose_no_update_count_threshold_error` / `twist_no_update_count_threshold_error`。
- 协方差椭圆的长轴大于阈值 `error_ellipse_size`，或横向大于 `error_ellipse_size_lateral_direction`。

## 已知问题

- 如果使用多个位姿估计器，则 EKF 的输入将包含对应于每个源的多个偏航偏置。然而，当前 EKF 假设仅存在一个偏航偏置。因此，当前 EKF 状态中的偏航偏置 `b_k` 将没有任何意义，且无法正确处理这些多个偏航偏置。因此，未来的工作包括为每个带有偏航估计的传感器引入偏航偏置。

## 参考文献

[1] Anderson, B. D. O., & Moore, J. B. (1979). Optimal filtering. Englewood Cliffs, NJ: Prentice-Hall.