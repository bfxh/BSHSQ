//! registry：arena_bench 的**场景注册表**（PhysArena 复刻批的统一入口）。
//!
//! `main.rs` 的默认分派只认识最早那批场景（pyramid/wall/ballpit/trimesh + 打印型
//! 探针）；此后所有复刻场景都注册在这张表里：`arena_bench <id>` 单个跑、
//! `arena_bench --all-arena` 全量跑、`arena_bench --list` 看目录。
//!
//! 每条注记（`note`）逐项说明**与 arena 的差异**：本仓无逐体阻尼 / 无 kinematic /
//! 无 sensor / 无动态三角网等，降级处一律显式写出，不静默近似。
use super::{
    scene_avalanche, scene_big_world, scene_bool_carve, scene_bool_slice, scene_bouncy_balls,
    scene_bowling_pins, scene_cannonball, scene_capsule_rain, scene_carom, scene_ccd_control,
    scene_ccd_onslaught, scene_compound_crates, scene_crane, scene_cylinder_jenga,
    scene_destruct_bridge, scene_destruct_columns, scene_destruct_fracture, scene_destruct_tower,
    scene_destruct_wall, scene_dom_carve_impact, scene_dom_cloth_wind, scene_dom_fluid_dam,
    scene_dom_fluid_float, scene_dom_rope_terrain, scene_dom_sph_scale, scene_dom_splash,
    scene_dom_splat_rest, scene_dom_voronoi, scene_domino, scene_domino_circle,
    scene_domino_spiral, scene_fluid_cascade, scene_fluid_dam_break, scene_fluid_drain,
    scene_fluid_pool, scene_fragmentation, scene_free_fall_ladder, scene_gear_train,
    scene_jenga_stack, scene_mass_ratio, scene_mixed_convex, scene_mixed_pile, scene_motor_wheel,
    scene_multi_contact_grid, scene_narrow_corridor, scene_newton_cradle, scene_piston_bank,
    scene_pressure_column, scene_pressure_vise, scene_pyramid_jitter, scene_ragdoll,
    scene_ragdoll_pile, scene_ramp_roll, scene_random_pile, scene_rope_bridge,
    scene_rotating_platform, scene_scissor_lift, scene_sensor_field, scene_shape_shells,
    scene_shape_slices, scene_shape_zoo, scene_shape_zoo_hard, scene_slider_crank,
    scene_small_objects, scene_sphere_pyramid, scene_spinning_tops, scene_spring_bed,
    scene_spring_net, scene_stack_arch, scene_stack_honeycomb, scene_stress_long_chain,
    scene_stress_many_tiny, scene_stress_slender_rod, scene_suspension_span, scene_teeter_totter,
    scene_tower, PhysConfig, World,
};

/// 冲击破坏场景的**专用计时窗**参数（provider id / 触发阈值 / 碎块密度）。
pub(crate) struct ImpactSpec {
    pub provider: u32,
    pub threshold: f32,
    pub density: f32,
}

/// 场景就绪包：世界 + 可选的专用计时窗（`None` = 通用 `bench()`）。
pub(crate) struct Ready {
    pub world: World,
    pub impact: Option<ImpactSpec>,
}

/// 一条场景目录项。
pub(crate) struct Entry {
    /// arena 的场景 id（同名同参）；本仓域场景用 `dom-` 前缀。
    pub id: &'static str,
    pub group: &'static str,
    /// arena 的 `defaultBodies`（体数口径；实际投放数可与之不同，见 arena 源码）。
    pub bodies: usize,
    /// 与 arena 的差异注记（空 = 逐条同参）。
    pub note: &'static str,
    pub build: fn(PhysConfig) -> World,
}

pub(crate) const CATALOG: &[Entry] = &[
    // ── 堆叠与结构（stacking.ts） ────────────────────────────────────────────
    Entry {
        id: "pyramid-jitter",
        group: "堆叠与结构",
        bodies: 210,
        note: "",
        build: scene_pyramid_jitter,
    },
    Entry {
        id: "tower",
        group: "堆叠与结构",
        bodies: 60,
        note: "",
        build: scene_tower,
    },
    Entry {
        id: "random-pile",
        group: "堆叠与结构",
        bodies: 300,
        note: "",
        build: scene_random_pile,
    },
    Entry {
        id: "irregular-block-jenga",
        group: "堆叠与结构",
        bodies: 90,
        note: "",
        build: scene_jenga_stack,
    },
    Entry {
        id: "sphere-pyramid",
        group: "堆叠与结构",
        bodies: 120,
        note: "",
        build: scene_sphere_pyramid,
    },
    Entry {
        id: "cylinder-jenga",
        group: "堆叠与结构",
        bodies: 90,
        note: "",
        build: scene_cylinder_jenga,
    },
    Entry {
        id: "stack-arch",
        group: "堆叠与结构",
        bodies: 30,
        note: "",
        build: scene_stack_arch,
    },
    Entry {
        id: "stack-honeycomb",
        group: "堆叠与结构",
        bodies: 60,
        note: "",
        build: scene_stack_honeycomb,
    },
    Entry {
        id: "stack-mixed-pile",
        group: "堆叠与结构",
        bodies: 70,
        note: "",
        build: scene_mixed_pile,
    },
    // ── 经典动力学（dynamics.ts） ────────────────────────────────────────────
    Entry {
        id: "domino",
        group: "经典动力学",
        bodies: 150,
        note: "arena 逐体角阻尼 0.02 未复刻（本仓无逐体阻尼）",
        build: scene_domino,
    },
    Entry {
        id: "domino-spiral",
        group: "经典动力学",
        bodies: 180,
        note: "",
        build: scene_domino_spiral,
    },
    Entry {
        id: "ramp-roll",
        group: "经典动力学",
        bodies: 60,
        note: "",
        build: scene_ramp_roll,
    },
    Entry {
        id: "newton-cradle",
        group: "经典动力学",
        bodies: 20,
        note: "arena 逐体 ccd=true：本仓 CCD 按全局速度阈值（默认关），此处两者都无实质作用",
        build: scene_newton_cradle,
    },
    Entry {
        id: "spinning-tops",
        group: "经典动力学",
        bodies: 40,
        note: "arena 逐体阻尼 0.005 未复刻；初始自转 28 rad/s 已复刻",
        build: scene_spinning_tops,
    },
    Entry {
        id: "bouncy-balls",
        group: "经典动力学",
        bodies: 200,
        note: "arena 逐体线性阻尼 0.005 未复刻",
        build: scene_bouncy_balls,
    },
    Entry {
        id: "free-fall-ladder",
        group: "经典动力学",
        bodies: 80,
        note: "",
        build: scene_free_fall_ladder,
    },
    Entry {
        id: "teeter-totter",
        group: "经典动力学",
        bodies: 30,
        note: "",
        build: scene_teeter_totter,
    },
    Entry {
        id: "rotating-platform",
        group: "经典动力学",
        bodies: 120,
        note: "**降级**：本仓无 kinematic 体 ⇒ 「静态锚 + 转动关节马达 1.2 rad/s」等效（体数 +1），与 arena 的 kinematic 速度推导不同源",
        build: scene_rotating_platform,
    },
    Entry {
        id: "domino-circle",
        group: "经典动力学",
        bodies: 60,
        note: "",
        build: scene_domino_circle,
    },
    Entry {
        id: "bowling-pins",
        group: "经典动力学",
        bodies: 40,
        note: "实际投放 = 15 瓶 + 1 球（arena 的 defaultBodies 只是滑块提示）",
        build: scene_bowling_pins,
    },
    Entry {
        id: "avalanche",
        group: "经典动力学",
        bodies: 260,
        note: "arena 逐体角阻尼 0.2 未复刻",
        build: scene_avalanche,
    },
    Entry {
        id: "carom",
        group: "经典动力学",
        bodies: 18,
        note: "arena 逐体阻尼 0.05/0.2 未复刻",
        build: scene_carom,
    },
    // ── 极端工况（stress.ts） ────────────────────────────────────────────────
    Entry {
        id: "ccd-onslaught",
        group: "极端工况",
        bodies: 60,
        note: "本仓 CCD 是全局速度阈值（无逐体开关）⇒ 本场景开阈值 20 m/s；arena 为逐体 ccd:true",
        build: scene_ccd_onslaught,
    },
    Entry {
        id: "ccd-control",
        group: "极端工况",
        bodies: 60,
        note: "同几何、CCD 关（本仓默认阈值 INFINITY）= arena 对照组同义",
        build: scene_ccd_control,
    },
    Entry {
        id: "cannonball",
        group: "极端工况",
        bodies: 180,
        note: "arena 侧炮弹 ccd:true；本仓全局阈值默认关（本场景按默认档跑）",
        build: scene_cannonball,
    },
    Entry {
        id: "small-objects",
        group: "极端工况",
        bodies: 500,
        note: "arena 逐体线性阻尼 0.02 未复刻",
        build: scene_small_objects,
    },
    Entry {
        id: "narrow-corridor",
        group: "极端工况",
        bodies: 300,
        note: "实际 280 体（arena 源码 clamp 到 280）",
        build: scene_narrow_corridor,
    },
    Entry {
        id: "mass-ratio",
        group: "极端工况",
        bodies: 120,
        note: "",
        build: scene_mass_ratio,
    },
    Entry {
        id: "big-world",
        group: "极端工况",
        bodies: 150,
        note: "",
        build: scene_big_world,
    },
    Entry {
        id: "fragmentation",
        group: "极端工况",
        bodies: 400,
        note: "arena 逐体角阻尼 0.1 未复刻",
        build: scene_fragmentation,
    },
    Entry {
        id: "multi-contact-grid",
        group: "极端工况",
        bodies: 600,
        note: "",
        build: scene_multi_contact_grid,
    },
    Entry {
        id: "stress-long-chain",
        group: "极端工况",
        bodies: 100,
        note: "",
        build: scene_stress_long_chain,
    },
    Entry {
        id: "stress-many-tiny",
        group: "极端工况",
        bodies: 900,
        note: "",
        build: scene_stress_many_tiny,
    },
    Entry {
        id: "stress-slender-rod",
        group: "极端工况",
        bodies: 12,
        note: "arena 逐体角阻尼 0.02 未复刻；实际 6 杆 + 6 压块",
        build: scene_stress_slender_rod,
    },
    // ── 碰撞形状（shapes.ts） ────────────────────────────────────────────────
    Entry {
        id: "mixed-convex",
        group: "碰撞形状",
        bodies: 200,
        note: "",
        build: scene_mixed_convex,
    },
    Entry {
        id: "shape-zoo",
        group: "碰撞形状",
        bodies: 140,
        note: "",
        build: scene_shape_zoo,
    },
    Entry {
        id: "compound-crates",
        group: "碰撞形状",
        bodies: 90,
        note: "",
        build: scene_compound_crates,
    },
    Entry {
        id: "capsule-rain",
        group: "碰撞形状",
        bodies: 300,
        note: "arena 逐体角阻尼 0.12 未复刻",
        build: scene_capsule_rain,
    },
    Entry {
        id: "sensor-field",
        group: "碰撞形状",
        bodies: 160,
        note: "**降级**：本仓无 sensor（触发体）⇒ 面板已省略，只保留球场成本；不与 arena 数字对表",
        build: scene_sensor_field,
    },
    Entry {
        id: "shape-zoo-hard",
        group: "碰撞形状",
        bodies: 48,
        note: "三角网臂走一等三角网形状（`add_trimesh`），无降级",
        build: scene_shape_zoo_hard,
    },
    Entry {
        id: "shape-shells",
        group: "碰撞形状",
        bodies: 24,
        note: "",
        build: scene_shape_shells,
    },
    Entry {
        id: "shape-slices",
        group: "碰撞形状",
        bodies: 140,
        note: "arena 逐体角阻尼 0.1 未复刻",
        build: scene_shape_slices,
    },
    // ── 约束与关节（joints.ts） ──────────────────────────────────────────────
    Entry {
        id: "rope-bridge",
        group: "约束与关节",
        bodies: 70,
        note: "",
        build: scene_rope_bridge,
    },
    Entry {
        id: "ragdoll",
        group: "约束与关节",
        bodies: 100,
        note: "arena 逐体角阻尼 0.25–0.3 未复刻",
        build: scene_ragdoll,
    },
    Entry {
        id: "slider-crank",
        group: "约束与关节",
        bodies: 20,
        note: "",
        build: scene_slider_crank,
    },
    Entry {
        id: "motor-wheel",
        group: "约束与关节",
        bodies: 24,
        note: "",
        build: scene_motor_wheel,
    },
    Entry {
        id: "spring-net",
        group: "约束与关节",
        bodies: 120,
        note: "",
        build: scene_spring_net,
    },
    Entry {
        id: "crane",
        group: "约束与关节",
        bodies: 12,
        note: "arena 吊索的弹簧参数未复刻（本仓距离关节为刚性）",
        build: scene_crane,
    },
    Entry {
        id: "piston-bank",
        group: "约束与关节",
        bodies: 16,
        note: "零重力 + ground y=−6 已复刻",
        build: scene_piston_bank,
    },
    Entry {
        id: "suspension-span",
        group: "约束与关节",
        bodies: 46,
        note: "arena 的弹簧参数（stiffness/damping）按刚性距离落地",
        build: scene_suspension_span,
    },
    Entry {
        id: "ragdoll-pile",
        group: "约束与关节",
        bodies: 60,
        note: "",
        build: scene_ragdoll_pile,
    },
    Entry {
        id: "gear-train",
        group: "约束与关节",
        bodies: 12,
        note: "零重力 + ground y=−8 已复刻",
        build: scene_gear_train,
    },
    Entry {
        id: "scissor-lift",
        group: "约束与关节",
        bodies: 20,
        note: "",
        build: scene_scissor_lift,
    },
    Entry {
        id: "spring-bed",
        group: "约束与关节",
        bodies: 40,
        note: "arena 的 spring 关节（stiffness 90 / damping 0.35）按**刚性距离**落地",
        build: scene_spring_bed,
    },
    // ── 破坏与流体（havoc.ts；球堆"假水"，非真实 SPH） ───────────────────────
    Entry {
        id: "bool-slice",
        group: "破坏与流体",
        bodies: 8,
        note: "炮击类：arena 逐体 ccd:true ⇒ 本仓按全局阈值 20 m/s 打开",
        build: scene_bool_slice,
    },
    Entry {
        id: "bool-carve",
        group: "破坏与流体",
        bodies: 140,
        note: "炮击类：本仓 CCD 阈值 20 m/s（弹丸 26 m/s）",
        build: scene_bool_carve,
    },
    Entry {
        id: "destruct-tower",
        group: "破坏与流体",
        bodies: 60,
        note: "炮击类：本仓 CCD 阈值 20 m/s（弹丸 46 m/s）",
        build: scene_destruct_tower,
    },
    Entry {
        id: "destruct-bridge",
        group: "破坏与流体",
        bodies: 24,
        note: "",
        build: scene_destruct_bridge,
    },
    Entry {
        id: "fluid-dam-break",
        group: "破坏与流体",
        bodies: 320,
        note: "球堆假水（同 arena）；炮击类：CCD 阈值 20 m/s",
        build: scene_fluid_dam_break,
    },
    Entry {
        id: "fluid-pool",
        group: "破坏与流体",
        bodies: 400,
        note: "球堆假水（同 arena）",
        build: scene_fluid_pool,
    },
    Entry {
        id: "pressure-vise",
        group: "破坏与流体",
        bodies: 180,
        note: "arena 逐体角阻尼 0.3 未复刻",
        build: scene_pressure_vise,
    },
    Entry {
        id: "pressure-column",
        group: "破坏与流体",
        bodies: 40,
        note: "arena 逐体角阻尼 0.2 未复刻",
        build: scene_pressure_column,
    },
    Entry {
        id: "fluid-cascade",
        group: "破坏与流体",
        bodies: 260,
        note: "球堆假水（同 arena）",
        build: scene_fluid_cascade,
    },
    Entry {
        id: "fluid-drain",
        group: "破坏与流体",
        bodies: 280,
        note: "球堆假水（同 arena）",
        build: scene_fluid_drain,
    },
    Entry {
        id: "destruct-wall",
        group: "破坏与流体",
        bodies: 180,
        note: "炮击类：本仓 CCD 阈值 20 m/s（弹丸 60 m/s）",
        build: scene_destruct_wall,
    },
    Entry {
        id: "destruct-columns",
        group: "破坏与流体",
        bodies: 60,
        note: "炮击类：本仓 CCD 阈值 20 m/s（弹丸 52 m/s）",
        build: scene_destruct_columns,
    },
    Entry {
        id: "destruct-fracture",
        group: "破坏与流体",
        bodies: 150,
        note: "实际 126 块（rows 上限 14）；炮击类：CCD 阈值 20 m/s",
        build: scene_destruct_fracture,
    },
    // ── 本仓域（Arena 谱系之外）：软体 / 流体 / 喷溅 / 破坏 / 耦合 ──────────
    Entry {
        id: "dom-cloth-wind",
        group: "本仓域·软体",
        bodies: 81,
        note: "源 tests/cloth_wind_scene.rs（布 × 风贯通；风 20 m/s）",
        build: scene_dom_cloth_wind,
    },
    Entry {
        id: "dom-rope-terrain",
        group: "本仓域·软体",
        bodies: 36,
        note: "源 tests/rope_scene.rs（33 节点绳落三角网 + 刚体代理通道）",
        build: scene_dom_rope_terrain,
    },
    Entry {
        id: "dom-fluid-dam",
        group: "本仓域·流体",
        bodies: 504,
        note: "源 examples/m5_dam_break.rs（M5 溃坝金样场景/504 粒）",
        build: scene_dom_fluid_dam,
    },
    Entry {
        id: "dom-fluid-float",
        group: "本仓域·流体",
        bodies: 512,
        note: "源 examples/m5_float_box.rs（M5 浮箱金样场景：水下轻盒靠浮力升上来）",
        build: scene_dom_fluid_float,
    },
    Entry {
        id: "dom-sph-scale",
        group: "本仓域·流体",
        bodies: 32768,
        note: "源 vxl-phys-fluid/examples/sph_scale.rs 形态（32³ 晶格、无 provider；SPEC §3 目标 30 万粒）",
        build: scene_dom_sph_scale,
    },
    Entry {
        id: "dom-splat-rest",
        group: "本仓域·喷溅",
        bodies: 3,
        note: "源 examples/splat_rest_probe.rs（平场 σ=0.5；盒/球落定）",
        build: scene_dom_splat_rest,
    },
    Entry {
        id: "dom-voronoi",
        group: "本仓域·破坏",
        bodies: 0,
        note: "M3 预断裂：体素墙 → Voronoi 碎块体 + 10 m/s 弹丸（碎块数见运行输出）",
        build: scene_dom_voronoi,
    },
    Entry {
        id: "dom-carve-impact",
        group: "本仓域·破坏",
        bodies: 0,
        note: "源 examples/m3_impact.rs（**专用计时窗**：每步都调 `apply_impact_destruction`）",
        build: scene_dom_carve_impact,
    },
    Entry {
        id: "dom-splash",
        group: "本仓域·耦合",
        bodies: 514,
        note: "源 tests/splash_scene.rs（刚-液-风三域：2b 流体 + 落球推水 + 风帆 + 风 12 m/s）",
        build: scene_dom_splash,
    },
];

/// 专用计时窗（与 `CATALOG` 分离，只给需要的场景）。
pub(crate) fn impact_spec(id: &str) -> Option<ImpactSpec> {
    (id == "dom-carve-impact").then_some(ImpactSpec {
        provider: 0,
        threshold: 8.0,
        density: 1000.0,
    })
}

/// 按 id 建场景（未注册 ⇒ None）。非空注记在此打印一次（进基准日志）。
pub(crate) fn build(name: &str, cfg: PhysConfig) -> Option<Ready> {
    let e = find(name)?;
    if !e.note.is_empty() {
        println!("  ⚠️ {}：{}", e.id, e.note);
    }
    Some(Ready {
        world: (e.build)(cfg),
        impact: impact_spec(name),
    })
}

pub(crate) fn find(name: &str) -> Option<&'static Entry> {
    CATALOG.iter().find(|e| e.id == name)
}

/// 全量场景 id（`--all-arena` 用）。
pub(crate) fn all_ids() -> Vec<&'static str> {
    CATALOG.iter().map(|e| e.id).collect()
}

/// `--list`：打印目录（按组）。
pub(crate) fn list() {
    let mut group = "";
    for e in CATALOG {
        if e.group != group {
            group = e.group;
            println!("── {group}");
        }
        println!(
            "  {:<22} {:>5} 体  {}",
            e.id,
            e.bodies,
            if e.note.is_empty() {
                ""
            } else {
                "⚠️ 有注记"
            }
        );
    }
    println!(
        "共 {} 个基准场景（arena 复刻 + 本仓域；另有 std：pyramid / wall / ballpit / trimesh + 打印型探针）",
        CATALOG.len()
    );
}
