# ParallelDog 全自动越障代码方案（STM32 + 雷达 + IMU + 上位机视觉）

## 1. 目标与边界

- 目标：机器人狗在 AUTO_OFFROAD 模式下完成“自主感知 -> 路径决策 -> 步态执行 -> 越障通过”。
- 已有基础：
  - 姿态链路已具备（HWT605 + 卡尔曼）
  - 步态状态机与底层控制已具备（dogTaskCtrl + Bezier/VMC）
  - 上位机数据输入通道已具备（USB VCP）
- 系统边界：
  - STM32 侧负责实时控制与短时规划
  - 上位机视觉负责障碍识别与语义分类

## 2. 现有工程接入点

- AUTO 模式线程入口：Core/Src/freertos.c
- 姿态更新入口：Core/Src/freertos.c, Core/Src/hwt605.c
- 主控行为状态机：Lib/Scr/postrue_control.c
- 机器狗状态定义：Lib/Inc/dog.h
- 上位机输入解析：Radar/ReadData.c, Radar/ReadData.h

建议保留现有 RC 模式逻辑不动，新增 AUTO 任务链路，做到可回退、可对比测试。

## 3. 软件分层架构

### 3.1 感知层（Perception）

- Radar Localizer：提供狗体在场地坐标系位置（x, y, vx, vy, 置信度）
- IMU Estimator：提供姿态（roll, pitch, yaw, yaw_rate）
- Vision Obstacle Feed：提供障碍列表（类型、中心点、尺寸、朝向、置信度）

输出统一世界模型 world_model。

### 3.2 任务层（Mission）

- 场景假设：障碍物随机摆放，但每类障碍尺寸固定，视觉可稳定给出障碍中心与朝向
- 输入：world_model、任务规则（所有障碍都需通过）、障碍尺寸模板库（按类型预置）
- 输出：下一个目标障碍 target_obstacle，及其通过策略 crossing_plan
- 作用：
  - 依据障碍相对位置和朝向做目标排序与通过顺序分配
  - 依据类型查表尺寸模板，减少尺寸估计误差对决策的影响
  - 处理失败重试与目标切换（超时、姿态超限、目标丢失）

### 3.3 规划层（Planning）

- 全局：障碍序列级规划（访问顺序、是否绕行）
- 局部：足端/机身目标轨迹（速度、步长、摆腿高度、机身姿态补偿）
- 关键逻辑：
  - 使用视觉给出的 heading 生成对齐航向 approach_heading
  - 使用障碍类型对应的固定尺寸模板，计算安全通过窗口与抬腿裕量
  - 当视觉朝向与历史估计突变时，触发短时平滑与重规划，避免控制量跳变
- 输出：motion_cmd（期望速度、期望航向、期望机身高度、步态模板）

### 3.4 控制层（Control）

- 复用现有 dog.state 与各 gait 函数
- 新增 AUTO 控制器，将 motion_cmd 映射为状态机切换与参数调节
- 维持 1kHz 级控制循环稳定性，不将重计算放进高频任务
Protocol_ReceiveHandler
## 4. 关键数据结构设计

建议新增以下结构体并集中管理（可放在独立模块头文件中）：

1) 传感器输入
- pose_2d_t: x, y, yaw, vx, vy, yaw_rate, timestamp, valid
- imu_state_t: roll, pitch, yaw, gx, gy, gz, timestamp, valid

2) 视觉障碍
- obstacle_type_t: HURDLE, STEP, GAP, SLOPE, UNKNOWN
- obstacle_t: id, type, cx, cy, width, depth, height, heading, confidence, timestamp
- obstacle_list_t: obstacle_t arr[N], count

3) 世界模型
- world_model_t:
  - self_pose
  - imu
  - obstacles
  - data_age_ms（每路数据时延）

4) 任务与规划
- mission_state_t: IDLE, SELECT_TARGET, APPROACH, PREPARE_CROSS, CROSSING, RECOVER, DONE, FAILSAFE
- crossing_mode_t: WALK_OVER, STEP_OVER, JUMP_OVER, DETOUR
- crossing_plan_t: mode, target_id, approach_heading, approach_speed, foot_clearance, body_height, timeout_ms
- motion_cmd_t: vx_cmd, vy_cmd, yaw_rate_cmd, body_height_cmd, gait_id

## 5. 线程与周期设计（FreeRTOS）

建议新增或拆分为 4 类任务：

1) SensorTask（100Hz）
- 收集雷达、IMU、视觉输入并时间戳对齐
- 更新 world_model 原子快照

2) MissionTask（20Hz）
- 根据障碍列表和当前状态选择 target_obstacle
- 生成 crossing_plan

3) PlannerTask（50Hz）
- 将 crossing_plan 转为连续 motion_cmd
- 做限幅和可达性检查

4) ControlTask（1kHz，沿用当前 calculateFunc）
- 读取 motion_cmd
- 映射 dog.state + gait 参数
- 调用 dogTaskCtrl 执行

实时性原则：
- 高频任务只做“执行”，低频任务做“思考”
- 跨任务通信使用队列 + 双缓冲快照，避免锁竞争

## 6. 通信协议建议（上位机 -> STM32）

现有 ReadData 仅解析 2 个 float（x/y），无法承载障碍类型与多障碍列表。建议升级为 TLV 帧：

帧头:
- SOF: 0xAA55
- version: 1B
- msg_type: 1B
- payload_len: 2B
- seq: 2B
- payload: variable
- crc16: 2B

msg_type 建议：
- 0x01: 自车定位
- 0x02: IMU补充信息（可选）
- 0x10: 障碍列表
- 0x20: 心跳

障碍列表 payload：
- count: 1B
- repeated obstacle item:
  - id: 1B
  - type: 1B
  - cx, cy, w, d, h, heading, confidence: float x7

健壮性策略：
- seq 连续性检查
- 超时失效（例如视觉超过 200ms 未更新视为 stale）
- CRC 错误计数并上报

## 7. 状态机设计（任务级 FSM）

mission_state 转移建议：

1) IDLE
- 条件：AUTO 模式使能且传感器健康
- 转移：-> SELECT_TARGET

2) SELECT_TARGET
- 动作：从未完成障碍中选最近且可达目标
- 转移：-> APPROACH 或 -> DONE

3) APPROACH
- 动作：机身对准目标，速度渐进接近
- 条件：距离 < d_prepare
- 转移：-> PREPARE_CROSS

4) PREPARE_CROSS
- 动作：降速、调姿、切换预备步态
- 条件：姿态稳定 + 相对位姿进入窗口
- 转移：-> CROSSING

5) CROSSING
- 动作：按障碍类型执行对应 crossing_mode
- 条件：越障成功判据成立
- 转移：-> RECOVER

6) RECOVER
- 动作：恢复常规步态、标记目标完成
- 转移：-> SELECT_TARGET

7) FAILSAFE
- 条件：关键传感器丢失、姿态过限、电机异常
- 动作：降级（站立/阻尼/停机）

## 8. 障碍类型到动作策略映射

- HURDLE（栏杆类）:
  - 提高摆腿高度 foot_clearance
  - 减小步速，保持机身俯仰补偿
- STEP（台阶）:
  - 分阶段抬前腿 -> 推进重心 -> 后腿跟进
  - body_height 先抬后稳
- GAP（沟壑）:
  - 增大跨步长度 + 瞬时速度提升
  - 必要时使用跳跃模板
- SLOPE（斜坡）:
  - 航向对齐坡向
  - pitch 前馈补偿，降低速度

先从 2 类障碍打通（HURDLE + STEP），其余类型按同一接口扩展。

## 9. 关键控制接口改造建议

### 9.1 dog.state 与参数化解耦

当前 dog.state 偏“离散动作”。建议增加 gait_param：
- step_length
- step_height
- duty_factor
- speed_scale
- body_height_offset

这样 PlannerTask 输出连续参数，ControlTask 再映射到离散状态 + 参数。

### 9.2 自动模式专用执行入口

保留 dogTaskCtrl 作为底层执行器，在 AUTO 模式中增加：
- auto_control_update(world_model, mission_state, crossing_plan, motion_cmd)

避免将任务决策逻辑直接塞进 gait 文件，保持可维护性。

## 10. 伪代码（主流程）

初始化：
1. init sensors
2. init protocol
3. init world model
4. set mission_state = IDLE

循环（MissionTask 20Hz）：
1. wm = 获取世界模型_snapshot()
2. if not healthy(wm): mission_state = FAILSAFE
3. switch mission_state
4. case IDLE: if auto_enabled -> SELECT_TARGET
5. case SELECT_TARGET: target = select_obstacle(wm)
6. case APPROACH: plan = make_approach_plan(target, wm)
7. case PREPARE_CROSS: plan = make_prepare_plan(target, wm)
8. case CROSSING: plan = make_cross_plan(target, wm)
9. case RECOVER: mark_done(target)
10. publish crossing_plan

循环（PlannerTask 50Hz）：
1. plan = get_crossing_plan()
2. cmd = plan_to_motion_cmd(plan, wm)
3. cmd = saturate(cmd)
4. publish motion_cmd

循环（ControlTask 1kHz）：
1. cmd = get_motion_cmd_latest()
2. apply_gait_param(cmd)
3. update dog.state
4. dogTaskCtrl(&dog)

## 11. 安全与鲁棒性

- 传感器健康监测：超时、跳变、置信度阈值
- 倾倒保护：|roll| 或 |pitch| 超限立即降级
- 速度/扭矩限幅：保持现有 VMC 限幅策略
- 通信失联：视觉丢失后降级为“保守通行/停止”
- 日志：状态转移、异常码、障碍通过结果

## 12. 分阶段落地计划（建议 4 周）

第 1 周：通信与数据模型
- 完成协议升级与障碍列表接收
- world_model 与数据时效管理

第 2 周：任务状态机
- mission_state 跑通 SELECT -> APPROACH -> RECOVER
- 先不做越障，仅到达目标点

第 3 周：越障策略
- 打通 HURDLE/STEP 两类 crossing_plan
- 调 gait_param 与姿态补偿

第 4 周：鲁棒性与联调
- 超时降级、异常恢复、统计日志
- 全障碍串行任务测试与参数定标

## 13. 验收指标

- 所有障碍完成率 >= 95%
- 单障碍平均通过时间（按类型统计）
- 任务中姿态超限次数、跌倒次数
- 通信丢包/CRC 错误率
- 失败后恢复成功率

## 14. 与当前代码的最小侵入改造原则

- 不改动 RC 模式路径
- 不重写现有 gait 核心函数
- 新增 AUTO 模块并通过接口接入
- 每阶段可回退、可单测、可灰度开关
