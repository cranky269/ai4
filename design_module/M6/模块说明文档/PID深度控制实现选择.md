# PID 深度控制实现选择

这是模块 6 的实现选择，不是对 `design/` 官方设计原文的改写。

当前实现采用：

```text
safe_area = max(area_ratio, min_area)
depth_output = depth_pid.compute(desired_area, safe_area, dt)
```

在正 PID 增益下，面积小于期望面积时误差为正，`velocity.x > 0` 明确表示向前
逼近；面积接近期望值时输出接近零；面积大于期望值时输出非正值或由末端策略
给出的配置速度。输出同时受 PID `max_out` 和正向逼近速度上限限制，并使用最小
面积保护、积分限幅/抗积分饱和以及 `dt<=0` 防护。

官方设计中保留的倒数面积示例仍是设计参考。直接面积误差是当前模块 6 的实现
选择，原因是符号直观、远目标正向逼近且限幅容易验证；它与倒数面积示例的差异
需在后续团队评审中单独确认。本实现未改变消息字段、话题、坐标帧或 MAVROS
转换责任。

## 保护与测试约定

- `area_ratio < desired_area` 时输出正向前进速度。
- `area_ratio == desired_area` 时输出接近零。
- `area_ratio > desired_area` 时输出负值、零或配置的末端速度。
- 输出同时受 PID `max_out` 和 `PID_FORWARD_SPEED_LIMIT` 约束。
- `area_ratio >= PID_IMPACT_AREA_THRESHOLD` 时使用配置的末端速度。
- `dt <= 0` 时不推进积分和微分；积分使用限幅和抗饱和。

单元及隔离验证覆盖远目标正向输出、中心期望面积零输出、近目标限幅、图像右下
目标的负 `yaw_rate`/负 `velocity.z`、多目标选择以及断流后的全零输出。
