//! `acoustic` 子命令：声学路径流 + 声明的演示房间 / 分层介质（纯搬移自 `lib.rs`）。

use crate::scene::scene;
use vxl_phys::Vec3;
use vxl_phys_acoustics::{
    jsonl as aco_jsonl, AcousticConfig, AcousticGeometry, AcousticMedium, AcousticTier,
    AcousticsExt, Edge, LayeredMedium, Listener, Medium, Portal, SoundPath, BANDS,
};

/// 声学几何的**声明式房间**（`acoustic` 子命令用）：顶点 / 三角 / 吸声 / 透射 / 开口。
pub(super) struct Room {
    verts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    absorb: Vec<[f32; BANDS]>,
    trans: Vec<[f32; BANDS]>,
    /// 逐面散射系数（A4 散射）：镜面那条按 `(1 - s)` 打折，剩下的走漫反射。
    scatter: Vec<[f32; BANDS]>,
    portals: Vec<Portal>,
    /// **衍射棱**（A4）：门框靠外那一侧的竖棱 —— 声音会从门框绕到墙后（"拐角后仍可闻"）。
    edges: Vec<Edge>,
}

impl Room {
    pub(super) fn geom<'a>(
        &'a self,
        medium: Option<&'a dyn AcousticMedium>,
    ) -> AcousticGeometry<'a> {
        AcousticGeometry {
            verts: &self.verts,
            tris: &self.tris,
            absorption: &self.absorb,
            transmission: &self.trans,
            scattering: &self.scatter,
            portals: &self.portals,
            edges: &self.edges,
            medium,
        }
    }
}

/// **介质体积场**（A3）：地板附近一层"雾 / 湿热层"（声速略低、**高频吸收明显更强**），
/// 上面是常温空气 —— 分频带之后，"听起来闷"就是一个可算的量。
///
/// 分层介质给**解析积分**（按层边界切段）⇒ 时延与衰减都不是采样近似。要换密度/温度剖面，
/// 实现 `AcousticMedium` 交进来即可（也可用固定步长回退档）。
pub(super) fn medium() -> LayeredMedium {
    let haze = Medium {
        speed_of_sound: 330.0,
        attenuation: [0.02, 0.06, 0.15],
    };
    LayeredMedium::split(1.0, haze, Medium::AIR)
}

/// 一间"声学房间"：地板（y = 0）+ 墙（z = 1.5）+ **门洞开口**（全开）+ **门框衍射棱**。
///
/// 它**不是**碰撞几何（`scene()` 里那些盒/体素才是）—— 这正是"声学几何 ≠ 碰撞几何"的样板：
/// 关上门（`open: 0.0`）就只剩墙的透射泄漏，开门则房间外听得见。
/// ⚠️ 地板放在 `y = 0`（**低于**体素地面顶 0.25）：冲击点在 y ≈ 0.25，若地板与它同高就是"掠射"
/// （入射余弦为 0 ⇒ 散射/反射都拿不到能量），这是声学几何可以比碰撞几何"更低"的一个实例。
pub(super) fn room() -> Room {
    let verts = vec![
        Vec3::new(-4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, -4.0),
        Vec3::new(4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 4.0),
        Vec3::new(-4.0, 0.0, 1.5),
        Vec3::new(4.0, 0.0, 1.5),
        Vec3::new(4.0, 3.0, 1.5),
        Vec3::new(-4.0, 3.0, 1.5),
        // 听者那间屋的**高天花板**（y = 6）：它同时"看得见"声源与听者（都在它下方），
        // 而且两条腿都从墙顶上方过去 ⇒ 漫反射散射能把声音绕过来。
        Vec3::new(-4.0, 6.0, 1.5),
        Vec3::new(4.0, 6.0, 1.5),
        Vec3::new(4.0, 6.0, 4.0),
        Vec3::new(-4.0, 6.0, 4.0),
    ];
    Room {
        verts,
        tris: vec![
            [0u32, 1, 2],
            [0, 2, 3],
            [4, 5, 6],
            [4, 6, 7],
            [8, 9, 10],
            [8, 10, 11],
        ],
        // 地板吸声一般；墙**逐带**吸声（越高频越吸 ⇒ 门缝里漏出来的声音自然"发闷"）。
        absorb: vec![
            [0.2f32; BANDS],
            [0.2; BANDS],
            [0.6, 0.9, 0.95],
            [0.6, 0.9, 0.95],
            [0.4; BANDS],
            [0.4; BANDS],
        ],
        // 墙的透射也是逐带的：高频更难透（现实里隔声量随频率上升）。
        trans: vec![
            [0.0f32; BANDS],
            [0.0; BANDS],
            [0.03, 0.02, 0.008],
            [0.03, 0.02, 0.008],
            [0.0; BANDS],
            [0.0; BANDS],
        ],
        // 散射：地板略漫反射；墙粗糙、高频更散；天花板也有（它是唯一能把声音"从墙上方绕过来"的面）。
        scatter: vec![
            [0.2f32; BANDS],
            [0.2; BANDS],
            [0.35, 0.5, 0.6],
            [0.35, 0.5, 0.6],
            [0.3; BANDS],
            [0.3; BANDS],
        ],
        portals: vec![Portal {
            quad: [
                Vec3::new(-1.0, 0.0, 1.5),
                Vec3::new(1.0, 0.0, 1.5),
                Vec3::new(1.0, 2.5, 1.5),
                Vec3::new(-1.0, 2.5, 1.5),
            ],
            open: 1.0,
            regions: None,
        }],
        edges: vec![Edge {
            a: Vec3::new(1.0, 0.0, 1.5),
            b: Vec3::new(1.0, 2.5, 1.5),
            coeff: [0.5, 0.35, 0.2],
            wedge: None,
        }],
    }
}

/// `acoustic`：声学路径流（JSONL，一行一条；机器可读形式就是它本身）。
///
/// 听者站在墙**外侧**（z = 3.0）⇒ 听到的是"隔壁房间"的冲击：穿门洞的 `portal` 路径 +
/// 墙的透射泄漏 `transmitted` + 门框绕射 + 天花板漫反射 + 反射。逐 tick 归约（每 tick 用干净
/// 缓冲 ⇒ 预算按 tick 算）。
///
/// `realtime` = 用 [`AcousticTier::Realtime`] 的**确定性工作量上界**（候选与产出都收紧）；
/// 档位不是墙钟预算，宿主想自适应请自己按帧率选档。
pub(super) fn acoustic_cmd(
    name: &str,
    ticks: usize,
    realtime: bool,
    order2: bool,
) -> Result<String, String> {
    let mut w = scene(name)?;
    let room = room();
    let field = medium();
    let geo = room.geom(Some(&field));
    let cfg = acoustic_cfg(realtime, order2);
    let listener = Listener {
        pos: Vec3::new(0.0, 1.0, 3.0),
        radius: 0.5,
    };
    let mut all: Vec<SoundPath> = Vec::new();
    for _ in 0..ticks {
        w.step();
        let mut tick: Vec<SoundPath> = Vec::new();
        w.acoustic_cues(&geo, &listener, &cfg, &mut tick);
        all.extend(tick);
    }
    Ok(aco_jsonl(&all))
}

/// 声学配置：`--realtime` 走实时档（候选与产出都收紧）；`--order2` 把反射阶数开到 2（A8）。
///
/// ⚠️ `--order2` 的候选 ≈ `max_reflections²` 条链 ⇒ 明显更慢，是给**离线试听/对表**用的。
pub(super) fn acoustic_cfg(realtime: bool, order2: bool) -> AcousticConfig {
    let mut cfg = if realtime {
        AcousticTier::Realtime.config()
    } else {
        AcousticConfig::default()
    };
    if order2 {
        cfg.reflection_order = 2;
    }
    cfg
}
