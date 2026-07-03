# hs.robot.nova_robot_graspnet

完整文档见扩展根目录：[../README.md](../README.md)

## 启用

```bash
conda activate isaac_env
bash scripts/start_isaac.sh
```

菜单：**Window → Hs Robot Nova Robot GraspNet**

## 使用流程

1. **Load scene** — 不锈钢桌 + Nova 机器人 + 8×26×16 cm 盒子
2. **Timeline Play** 或 **Start ROS stream** — 发布 ROS
3. **Box 6D Pose** — 调整盒子位姿；Play 时可 Live apply

## 源码位置

```text
hs/robot/nova_robot_graspnet/impl/
```

改后 Reload Extension，无需 sync。
