# 快速上手（给**外部调用者**的入口面）

本文只讲"别人怎么把 `vxl_phys` 接进自己的程序"，不涉及仓库内部的门禁与里程碑。
内部视角看 `docs/INDEX.md`；每个域的能力边界看 `docs/CATALOG.md` 与 `docs/CAPABILITIES.yaml`。

## 1. 依赖哪个 crate

| 你要做的事 | 依赖 | 说明 |
|---|---|---|
| 刚体/接触/关节/地形/破坏/布绳（绝大多数） | `vxl-phys` | 门面。**只需要它**：内部 20+ 个 crate 由它组装 |
| 声音/震动（把冲击归约成音频/触觉 cue） | `vxl-phys` + `vxl-phys-sense` | 感官域是**只读消费者**（`impl SenseExt for World`） |
| 声学传播（射线追踪：直达/反射/遮挡 + 孔隙传声 + 透射泄漏 + 介质体积场 + 频带 + 绕射 + 散射） | `vxl-phys` + `vxl-phys-acoustics` | 第二条输出侧域；同样**只读**，几何（含开口/衍射棱）与逐带材质表/介质场都由调用方交进来 |
| 体素几何（自己造 `VoxelVolume`） | `vxl-phys-terrain` | 门面已转出常用件；要直接建体素加它 |
| 布/绳（自己造 `ClothSheet`/`Rope`） | `vxl-phys-soft` | 同上 |

导入一行：`use vxl_phys::prelude::*;`（稳定面清单见 `crates/vxl-phys/src/prelude.rs`）。

## 2. 最小可跑（刚体）

```rust
use vxl_phys::prelude::*;

let mut w = World::new(PhysConfig::default());      // 默认档：2 子步 × 3 迭代，60 Hz
w.add_static(                                        // 地面
    Shape::Box { half: Vec3::new(4.0, 0.5, 4.0) },
    Vec3::new(0.0, -0.5, 0.0),
    Quat::IDENTITY,
);
let ball = w.add_dynamic(
    Shape::Sphere { radius: 0.25 },
    Vec3::new(0.0, 2.0, 0.0),
    Quat::IDENTITY,
    1000.0,                                          // 密度 kg/m³
);
for _ in 0..240 {
    w.step();                                        // 一次 = 一个固定 tick（没有 dt 参数）
}
println!("{:?} {:?}", w.bodies.position[ball as usize], w.health());
```

**要守的三条契约**（不守就别指望复现）：

1. **固定节拍**：`step()` 一次 = `config.dt`（默认 1/60 s）。要慢放就少调几次，别去改 `dt` 的语义。
2. **严格 f32**：不要把 f64 算出来的量喂进物理状态（跨平台逐位一致的根据就是这个）。
3. **复现自检**：`w.state_hash()` 每 60 tick 比一次，同场景同输入必须同值（跨机复现的承诺范围见 `docs/adr/`）。

## 3. 命令行（不写代码先看效果）

```bash
cargo run -p vxl-phys-cli -- demo  drop  240     # 三盒落下：末态位姿 + 健康
cargo run -p vxl-phys-cli -- demo  stack 240     # 5 层塔（堆叠/睡眠）
cargo run -p vxl-phys-cli -- demo  cloth 240     # 一张 2×2 的布（两角钉住）
cargo run -p vxl-phys-cli -- demo  voxel 240     # 体素地面（破坏/感官需要 provider 地面）
cargo run -p vxl-phys-cli -- hash  stack 600     # 末态 state_hash（两次跑必须同值）
cargo run -p vxl-phys-cli -- check drop  600     # 同场景跑两遍、逐观测点比对哈希（**CI 门**：不一致退出码 2）
cargo run -p vxl-phys-cli -- cue   voxel 240     # 感官 cue 流（JSONL，一行一条）
cargo run -p vxl-phys-cli -- acoustic voxel 240  # 声学路径流（JSONL；逐带 bands + 声明式"房间"，与碰撞体无关）
cargo run -p vxl-phys-cli -- hash  stack 600 --json   # 任意子命令加 --json ⇒ 机器可读（脚本/CI 取数）
cargo run -p vxl-phys-cli -- record stack 600 base.jsonl   # 录一份观测点基线（一行一个 tick 的哈希）
cargo run -p vxl-phys-cli -- replay stack 600 base.jsonl   # 与之逐行对拍（跨机器/跨版本复现检查）
```

集成方最省事的"确定性自检"就是这条：把它塞进你自己的 CI，改引擎档位/依赖后跑一遍，
`check` 报 PASS 才算你没把可复现性弄丢（它内部走 `Recorder` 的观测点 + 末态 `state_hash`）。
要**跨机器/跨版本**比，就在参考机上 `record` 留一份 `*.jsonl` 基线，别的机器上 `replay` 逐行对拍
（不一致会指到第一个不同的观测点；行数不同也判 FAIL）。

`crates/vxl-phys-cli/src/lib.rs` 本身就是"只用公共 API 能写完整个工具"的样板 —— 接自己的场景
抄它比抄仓库里的里程碑示例更快（后者是内部视角：直接摸 `bodies`/`manifolds` 做诊断）。

## 4. 域扩展 trait（本仓的域 API 约定）

新能力**不往 `World` 加方法**（那个类型的公开面有尺寸预算），而是挂扩展 trait：

```rust
use vxl_phys::prelude::*;            // DestructionExt / VoxelConversionExt 在其中
let id = w.add_voxel(vol);           // 注册体素
let dug = w.carve_sphere(id, center, 0.6);                     // 破坏域
let st = w.sense_cues(&cfg, &mut cues);                        // 感官域（vxl-phys-sense）
```

用哪个 trait：看该能力所在 crate 的 `lib.rs` 顶部（每个域都写明自己的入口 trait 与判据）。

## 5. 出问题时的自检顺序

1. `w.health()`：`nan_bodies` 非 0（两次 step 之间被写脏 / 极值输入）、`deep_penetrations`（穿透）、
   `awake_bodies`（睡不着）。
2. `w.impacts()`：本 tick 的冲击快照 —— **只对 provider 接触有记录**（盒地面永远看不到它）。
3. 二分复现：固定 tick 数，用 `w.state_hash()` 找从哪一步起不同。
