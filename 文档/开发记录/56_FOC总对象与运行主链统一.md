# FOC 总对象与运行主链统一

## 本节点解决的问题

历史代码同时使用 `pmsm_t`、全局 `pm`、函数参数 `pm` 和较长的模式函数名，
导致读代码时很难快速判断“这是整个控制器，还是某个局部算法对象”。

现在正式运行链统一为：

```c
foc_t g_foc;

foc_init(&g_foc);
fast_loop_step(&g_foc);
drive_fast_step(&g_foc);
```

普通函数参数统一写成 `foc_t *foc`。`g_` 只出现在固件唯一的全局实例上，
局部函数不再假装自己必须访问全局变量。

## Debug 时怎样展开对象

| 路径 | 内容 |
|---|---|
| `g_foc.motor` | 当前电机电阻、电感、磁链、极对数、减速比和零位 |
| `g_foc.board` | ADC 比例、采样电路和死区等板卡参数 |
| `g_foc.enc` | MA732/MT6816 原始计数、单圈角和位置状态 |
| `g_foc.sig` | 电流、电压、dq 量、速度、位置、转矩和占空比 |
| `g_foc.rate` | FOC 与三闭环频率、周期和分频计数 |
| `g_foc.ctrl` | 当前控制给定、限制和级联中间参考 |
| `g_foc.cmd` | 上层正式命令 |
| `g_foc.app` | 应用极性、轨迹选项和命令范围 |
| `g_foc.state/req` | Drive 当前状态和请求 |
| `g_foc.fault/prot_*` | 故障结果、保护配置和连续计数 |

## 关键函数命名

| 新名字 | 作用 |
|---|---|
| `foc_init()` | 初始化整套 FOC 实例 |
| `cur_pi_init()` | 根据电机模型初始化 d/q 电流 PI |
| `spd_pi_init()` | 根据电机模型初始化速度 PI |
| `ctrl_fb_update()` | 更新母线、限幅、转矩和速度反馈 |
| `foc_volt_step()` | 电压模式的一拍 FOC |
| `foc_cur_step()` | 电流模式的一拍 FOC |
| `foc_spd_step()` | 速度、电流级联的一拍 FOC |
| `foc_pos_step()` | 位置、速度、电流级联的一拍 FOC |
| `open_volt_step()` | 开环角度加电压控制 |
| `open_cur_step()` | 开环角度加电流控制 |
| `cst_step/csv_step/csp_step` | 正式转矩、速度、位置模式 |
| `quick_stop_step()` | 快速停机减速过程 |

## 架构边界

- Target 中断只选择唯一实例 `g_foc`，随后调用 `fast_loop_step()`；
- App、Drive、Adapter 和 Motor 函数都通过 `foc_t *foc` 传递对象；
- FOC Core 仍只接收 `foc_ctrl_t/foc_fb_t/foc_ref_t/foc_out_t`，不依赖总对象；
- 初始化函数不再偷偷操作全局实例，而是明确初始化调用者传入的对象。

## 本节点没有改什么

- 没有改变状态机转移；
- 没有改变四种 FOC 模式公式；
- 没有改变 ADC、编码器和 PWM 时序；
- 没有改变 PI 参数或保护阈值。

## 验收条件

- 正式运行代码不再出现根类型 `pmsm_t` 或全局 `pm`；
- 固件唯一实例名为 `g_foc`；
- 所有正式函数通过 `foc_t *foc` 明确传递对象；
- 主机测试、规范检查、CMake/Keil 清单、Debug 和 Release 构建全部通过。
