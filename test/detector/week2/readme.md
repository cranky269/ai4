# 第二周 M4 发送侧接收核验

仅调用现有未修改的 M5 和 M8 代码，观察真实 Gazebo bag 中的检测如何被消费。M8 使用原始 StateMachineNode 类，在审计节点中增加内存计数日志；不等价于完整生产 launch 验收。

不创建或接管 test/flow/M4-M5、M4-M8 接收方目录。不注入人为高置信检测，不改锁定门槛。锁定/回退的受控消息测试应由 M8 接收方完成。

源 bag 固定 /home/demo/m4-ort1-exit-reentry.NeAllp/live.bag，偏移100秒，时长100秒，以真实时间回放。
