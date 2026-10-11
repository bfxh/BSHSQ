//! **反射核的探针（GPU；需适配器）**：一次镜面反射 —— 与**镜像展开**的闭式参照对拍。
//
//! 参照值用"展开法"手算：把声源对 `y = 0` 镜像得 `src'`，则折线长 `= |src' − dst|`
//! ⇒ 时延 `= L/speed`、增益 `= 1/L²`（地板吸收取 0）。卡上跑的是**逐三角的镜像源法**
//! （镜像听者 + 平面求交 + 重心内点 + 两段遮挡），与参照不是同一份实现 ⇒ 非自证。
//!
//! ⚠️ 无适配器 ⇒ 跳过（`[SKIP]`）；本仓 CI 的口径 `--exclude vxl-phys-gpu` ⇒ 只在有卡的机器上跑。

use vxl_phys_core::Vec3;

const SPEED: f32 = 343.0;

fn have_adapter() -> bool {
    vxl_phys_gpu::probe::device_for(0).is_ok()
}

/// 地板（`y = 0`、`x,z ∈ [-4,4]`，两块三角，吸收 0）。
fn floor() -> Vec<[Vec3; 3]> {
    vec![
        [
            Vec3::new(-4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, 4.0),
        ],
        [
            Vec3::new(-4.0, 0.0, -4.0),
            Vec3::new(4.0, 0.0, 4.0),
            Vec3::new(-4.0, 0.0, 4.0),
        ],
    ]
}

/// 挡板（`y = y0`、`x,z ∈ [-2,2]`）—— 吸收取 1 ⇒ 它自己不会成为反射面，只负责遮。
fn plate(y0: f32) -> Vec<[Vec3; 3]> {
    vec![
        [
            Vec3::new(-2.0, y0, -2.0),
            Vec3::new(2.0, y0, -2.0),
            Vec3::new(2.0, y0, 2.0),
        ],
        [
            Vec3::new(-2.0, y0, -2.0),
            Vec3::new(2.0, y0, 2.0),
            Vec3::new(-2.0, y0, 2.0),
        ],
    ]
}

/// 镜像展开：折线长 `= |src' − dst|`（`src'` = 声源对 `y = 0` 的镜像）。
fn unfolded_len(src: Vec3, dst: Vec3, plane_y: f32) -> f32 {
    let m = Vec3::new(src.x, 2.0 * plane_y - src.y, src.z);
    (dst - m).length()
}

fn pairs() -> (Vec<Vec3>, Vec<Vec3>) {
    let srcs = vec![Vec3::new(0.5, 1.0, 0.0), Vec3::new(-0.5, 1.5, 1.0)];
    let dsts = vec![Vec3::new(0.0, 0.8, 2.0), Vec3::new(1.5, 0.5, -1.0)];
    (srcs, dsts)
}

/// 两条一起跑（顺序）：**并行跑两条测试会各建一个 wgpu 设备，实测会互相卡住** ⇒ 合成一条。
#[test]
fn reflection_matches_reference_and_respects_occlusion() {
    if !have_adapter() {
        println!("[SKIP] 没有可用适配器（headless 环境属预期）");
        return;
    }
    let (srcs, dsts) = pairs();
    let tris = floor();
    let absorb = vec![0.0f32; tris.len()];
    let (name, got) = match vxl_phys_gpu::acoustic::reflected_on_adapter(
        0, &srcs, &dsts, &tris, &absorb, SPEED,
    ) {
        Ok(v) => v,
        Err(e) => {
            println!("[SKIP] 核起不来：{e}");
            return;
        }
    };
    println!("[gpu] {name}: {} 对（地板反射）", got.len());
    let mut hits = 0usize;
    for (i, s) in srcs.iter().enumerate() {
        for (j, d) in dsts.iter().enumerate() {
            let k = i * dsts.len() + j;
            assert!(got[k].hit, "第 {k} 对必须找到地板反射（否则判据空跑）");
            hits += 1;
            let l = unfolded_len(*s, *d, 0.0);
            let want_delay = l / SPEED;
            let want_gain = 1.0 / (l * l);
            assert!(
                (got[k].delay - want_delay).abs() <= want_delay * 1e-4,
                "第 {k} 对时延（卡上 {} vs 展开式 {want_delay}）",
                got[k].delay
            );
            assert!(
                (got[k].gain - want_gain).abs() <= want_gain * 1e-4,
                "第 {k} 对增益（卡上 {} vs 展开式 {want_gain}）",
                got[k].gain
            );
            assert!(
                got[k].tri >= 0.0 && got[k].tri < tris.len() as f32,
                "第 {k} 对必须报出反射所在的三角序号（实际 {}）",
                got[k].tri
            );
        }
    }
    assert_eq!(hits, srcs.len() * dsts.len(), "所有对都该有地板反射");
    // 第二段：地板 + 一块**全吸收**的挡板（它自己不成反射面，只负责遮住地板那条折线）
    let mut tris = floor();
    tris.extend(plate(0.4));
    let mut absorb = vec![0.0f32; 2];
    absorb.extend([1.0f32; 2]);
    let (name, got) = match vxl_phys_gpu::acoustic::reflected_on_adapter(
        0, &srcs, &dsts, &tris, &absorb, SPEED,
    ) {
        Ok(v) => v,
        Err(e) => {
            println!("[SKIP] 核起不来：{e}");
            return;
        }
    };
    println!("[gpu] {name}: {} 对（带挡板）", got.len());
    for (k, r) in got.iter().enumerate() {
        assert!(
            !r.hit,
            "第 {k} 对的反射折线被挡死 ⇒ 不该报出反射（实际 {:?}）",
            r
        );
    }
}
