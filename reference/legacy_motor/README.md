# 旧版电机算法参考区

本目录保存架构重构前的旧电机算法，目的只有两个：

1. 需要核对旧公式或旧实现时可以回看；
2. 以后迁移某个算法时，有明确的历史来源可以对照。

这里的文件不进入 CMake 或 Keil 正式固件构建，也不是新增功能的开发入口。正式代码应放在：

- 纯数学算法：`firmware/AxDr_App/User/control`；
- 驱动状态、模式和保护：`firmware/AxDr_App/User/drive`；
- 板卡硬件访问：`firmware/AxDr_App/User/bsp`；
- 硬件数据到物理量的换算：`firmware/AxDr_App/User/adapter`。

不要在本目录直接继续维护新功能。需要恢复或迁移某段算法时，应先提取独立输入、输出和状态，再写入正式模块并补电脑端测试。
