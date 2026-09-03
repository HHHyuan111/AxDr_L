# FOC 反馈、参考与输出分组

## 为什么修改

原来的 `sig` 是 signal 的缩写，其中同时保存了采样反馈、参考相关数据、坐标变换结果和 PWM 输出。它能工作，但阅读代码时无法从路径判断数据方向，也容易在调试器中把给定值和反馈值看混。

## 当前结构

FOC 总对象只按三个实际数据方向分组：

| 分组 | 含义 | 典型变量 |
|---|---|---|
| `g_foc.fb` | 实际采样或由采样计算出的反馈 | `ia/ib/ic`、`vbus`、`theta_e`、`spd_r`、`id/iq` |
| `g_foc.ref` | 当前模式正在使用的参考、轨迹结果和限制 | `id/iq`、`spd_r`、`pos_r`、`iq_lim` |
| `g_foc.out` | FOC 本周期计算结果 | `vd/vq`、`valpha/vbeta`、`duty_a/b/c` |

例子：

```c
g_foc.ref.iq       /* q 轴电流参考 */
g_foc.fb.iq        /* q 轴电流反馈 */
g_foc.out.vq       /* q 轴电压输出 */
g_foc.out.duty_a   /* A 相占空比输出 */
```

因此读一行代码时，不需要再靠注释猜测变量方向。

## 改动边界

- 只重新归类并改名，没有改变控制周期顺序、状态机条件、PI 公式或 SVPWM 公式。
- `foc_ctrl_t` 仍是纯 Control 层的控制器运行上下文，不和 `g_foc.ref` 混为一体。
- `i_valid/vbus_valid/pos_valid` 与对应物理量一起放在 `g_foc.fb` 中。
- 旧数值对照测试改用测试目录自己的 `legacy_foc_state_t`，不再依赖生产代码已经删除的 `sig` 类型。

## 验证

本节点使用既有 Host 数值对照、状态机、保护、适配层、诊断算法以及 Keil 双配置构建进行验证。实机行为仍按 PR60 与 MA732 上电清单逐层确认。
