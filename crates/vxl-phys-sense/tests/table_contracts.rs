//! **S3 材质表 / 资产映射的判据**（`docs/ROUTE.md` §3.2 分期表：S3 = 外部表 → 音色组 id，判据 =
//! **缺项显式拒绝**（不静默用默认音））。
//!
//! S3 与 S1/S2 是**三种不同性质的东西**：前两片是"物理 → cue 流"的**归约**（所以在 `World` 上、
//! 要守零干扰 / 确定性 / 预算三条契约），S3 是作用在**已经产出**的 cue 流上的**纯映射**
//! （`map_banks(&SoundTable, &[T]) -> Result<Vec<BankId>, TableError>`）。零干扰在这里是**类型级**
//! 保证：它不借 `&World`、也不借 `&mut`——物理末态不可能被它碰到。
//!
//! 于是本片真正要判的是三件事：
//! ① **缺项 = 拒绝**（唯一那条硬判据）：表里没有的材质槽**不许回落默认音**，必须 `Err` 并把槽报出来；
//! ② **表文本可错得明白**：重复登记 / 语法错都拒绝，且**带行号**（否则表一长就没法定位）；
//! ③ **两条流一视同仁**：S1 的 [`Cue`] 与 S2 的 [`SlideCue`] 走**同一张表**（`MaterialSlot`），加第三条
//!    流不用改本片；且同输入两跑映射结果逐位一致。
//!
//! **写法约定**：本片测试**零 `unwrap` / `expect` / `panic!`**——`unwrap-gate` 与 `todo-gate` 都是
//! **计数棘轮**（新文件新增即红，存量在 `scripts/*.baseline.json`）。所以"理应成功"一律用 `?`
//! 传播（测试函数返回 `Result`），失败信息由 `Debug` 给出。

use vxl_phys::{core::MaterialId, BodyBuildExt, Material, PhysConfig, Quat, Shape, Vec3, World};
use vxl_phys_sense::{
    jsonl, jsonl_banked, map_banks, slide_jsonl_banked, write_jsonl_banked, BankId, Cue,
    MaterialSlot, SenseConfig, SenseExt, SlideCue, SoundTable, TableError,
};

/// 本片所有测试的返回类型（`?` 传播：`TableError` 与 `io::Error` 都能进来）。
type R = Result<(), Box<dyn std::error::Error>>;

/// 造一条只关心材质槽的 cue（其余字段与映射无关，填最小合法值）。
fn cue_with_material(material: u32) -> Cue {
    Cue {
        tick: 0,
        body: 0,
        material,
        gain: 1.0,
        pos: [0.0; 3],
        approach: 1.0,
        priority: 0,
    }
}

/// 体素地板（provider）——**S1 冲击**的地板必须是 provider（冲击记录只对 `Shape::Provider` 接触
/// 产生，见 `World::record_impacts`）。
fn floor_voxel() -> World {
    let mut w = World::new(PhysConfig::default());
    let mut vol =
        vxl_phys_terrain::voxel::VoxelVolume::new(Vec3::new(-4.0, -0.5, -4.0), 0.5, 16, 3, 16);
    vol.fill_box(Vec3::new(-4.0, -0.5, -4.0), Vec3::new(4.0, 1.0, 4.0));
    w.add_voxel(vol);
    w
}

/// 平滑静态盒地板（S2 用 `World::manifolds()`，不要求 provider）。
fn floor_smooth() -> World {
    let mut w = World::new(PhysConfig::default());
    w.add_static(
        Shape::Box {
            half: Vec3::new(4.0, 0.5, 4.0),
        },
        Vec3::new(0.0, 0.5, 0.0),
        Quat::IDENTITY,
    );
    w
}

/// **多材质**场景（S1 冲击那一套盒雨 + 逐体挂材质槽）：`mats[i % n]` 轮流挂到每只盒上。
///
/// 三种材质**参数可以完全相同**（本测试只考"映射"，不考"摩擦"）：材质槽是**标签**，映射表按标签
/// 分派音色包——物理差异与本片无关。这正是 S3 不往 `Material` 塞声学字段的理由：标签与音色包是
/// 两张皮，各自演进。
///
/// `vx` 给水平初速：体素地板 + `vx = 0` ⇒ 冲击流（S1）；平滑地板 + `vx = 3` ⇒ 滑行流（S2）。
fn scene_multi(floor: World, bodies: usize, mats: &[MaterialId], vx: f32) -> World {
    let mut w = floor;
    for i in 0..bodies {
        let (ix, iz) = ((i % 8) as f32, (i / 8) as f32);
        let p = Vec3::new(-1.75 + 0.5 * ix, 1.22, -1.75 + 0.5 * iz);
        let id = w.add_dynamic(
            Shape::Box {
                half: Vec3::splat(0.2),
            },
            p,
            Quat::IDENTITY,
            1000.0,
        );
        w.bodies.set_material(id as usize, mats[i % mats.len()]);
        w.bodies.linvel[id as usize] = Vec3::new(vx, 0.0, 0.0);
    }
    w
}

/// 注册三种材质并返回它们的**槽号**（`add_material` 的返回序；默认材质占槽 0）。
fn three_materials(w: &mut World) -> Vec<MaterialId> {
    vec![
        w.add_material(Material::default()),
        w.add_material(Material::default()),
        w.add_material(Material::default()),
    ]
}

/// **判据 ①：缺项显式拒绝**（不静默用默认音）。
#[test]
fn missing_material_slot_is_rejected_not_defaulted() -> R {
    let table = SoundTable::from_entries(&[(0, 100), (1, 110)])?;
    assert_eq!(table.lookup(1), Ok(BankId(110)), "登记过的槽必须查到");
    assert_eq!(
        table.lookup(7),
        Err(TableError::Missing { material: 7 }),
        "未登记的槽必须拒绝，且报出**是哪个槽**"
    );

    // 空表合法（表可分文件），但**不是**"什么都能查"：查谁都是缺项。
    let empty = SoundTable::parse("# 只有注释\n\n")?;
    assert!(empty.is_empty());
    assert_eq!(
        empty.lookup(0),
        Err(TableError::Missing { material: 0 }),
        "空表不许兜底"
    );

    // 真 cue 流上同一条判据：给定材质槽 0 的表，去映射带槽 1 的流 ⇒ 整条拒绝（不部分映射）。
    let slots: Vec<u32> = vec![0, 0, 1];
    let fake: Vec<Cue> = slots.iter().map(|m| cue_with_material(*m)).collect();
    let only_zero = SoundTable::from_entries(&[(0, 100)])?;
    assert_eq!(
        map_banks(&only_zero, &fake),
        Err(TableError::Missing { material: 1 }),
        "流里出现缺项必须**整条**拒绝（不做部分映射）"
    );
    Ok(())
}

/// **判据 ②：表文本的错误都带行号**（重复登记 / 缺 `->` / `->` 多于一个 / 非数字）。
#[test]
fn table_errors_are_explicit_with_line_numbers() -> R {
    // 注释与空行不占"语法"（但仍占行号）。
    let table = SoundTable::parse("# 注释\n\n3 -> 130\n 1 -> 110 \n")?;
    assert_eq!(
        table.entries(),
        &[(1, BankId(110)), (3, BankId(130))],
        "按槽升序"
    );
    assert_eq!(table.len(), 2);

    assert_eq!(
        SoundTable::parse("0 -> 1\n0 -> 2\n"),
        Err(TableError::Duplicate {
            line: 2,
            material: 0
        }),
        "重复登记必须拒绝并指向第二行"
    );
    assert!(matches!(
        SoundTable::parse("0 -> 1\n# 注释\nfoo\n"),
        Err(TableError::Syntax { line: 3, .. })
    ));
    assert!(matches!(
        SoundTable::parse("1 -> 2 -> 3\n"),
        Err(TableError::Syntax { line: 1, .. })
    ));
    assert!(matches!(
        SoundTable::parse("0 -> x\n"),
        Err(TableError::Syntax { line: 1, .. })
    ));
    assert!(matches!(
        SoundTable::from_entries(&[(4, 1), (4, 2)]),
        Err(TableError::Duplicate { material: 4, .. })
    ));
    // 错误信息必须**可读**（工具链会把它直接打到 stderr）。
    let e = TableError::Missing { material: 9 }.to_string();
    assert!(e.contains('9'), "错误文案要带上槽号：{e}");
    Ok(())
}

/// **判据 ③：两条流一视同仁，且映射确定性**（同输入两跑逐位一致；每个槽映射到它自己那行的 id）。
#[test]
fn both_streams_map_through_one_table() -> R {
    // 三种材质 → 三个槽（1/2/3；槽 0 是默认材质，本场景的盒子显式挂了材质）。
    let mut probe = World::new(PhysConfig::default());
    let mats = three_materials(&mut probe);
    assert!(mats.iter().all(|m| *m > 0), "默认材质占槽 0：{mats:?}");
    let table =
        SoundTable::from_entries(&[(0, 100), (mats[0], 110), (mats[1], 120), (mats[2], 130)])?;

    // —— S1 流：多材质盒雨的冲击 cue ——
    let cfg = SenseConfig::default();
    let mut world = scene_multi(floor_voxel(), 24, &mats, 0.0);
    let mut cues: Vec<Cue> = Vec::new();
    for _ in 0..120 {
        world.step();
        world.sense_cues(&cfg, &mut cues);
    }
    assert!(!cues.is_empty(), "场景必须真的产 cue，否则本判据空洞");
    let banks_a = map_banks(&table, &cues)?;
    // 槽号 → 音色组 id 必须**逐一对应**（不是"全体同一 id"，那说明查错了行）。
    let seen: std::collections::BTreeSet<u32> = cues.iter().map(|c| c.material).collect();
    let got: std::collections::BTreeSet<u32> = banks_a.iter().map(|b| b.0).collect();
    assert!(
        seen.len() > 1,
        "多材质场景应出现不止一个槽（实测 {seen:?}）"
    );
    let mut want: std::collections::BTreeSet<u32> = std::collections::BTreeSet::new();
    for m in &seen {
        want.insert(table.lookup(*m)?.0);
    }
    assert_eq!(got, want, "每个槽必须映射到它自己那行的 id");

    // —— 同一个表，S2 流照样走（`MaterialSlot` 对两条流一视同仁）——
    let mut sw = scene_multi(floor_smooth(), 24, &mats, 3.0);
    let mut slides: Vec<SlideCue> = Vec::new();
    for _ in 0..120 {
        sw.step();
        sw.slide_cues(&cfg, &mut slides);
    }
    assert!(!slides.is_empty(), "S2 流也要真的非空");
    let banks_s = map_banks(&table, &slides)?;
    assert_eq!(banks_s.len(), slides.len(), "与入参同长");
    for (s, b) in slides.iter().zip(&banks_s) {
        assert_eq!(
            *b,
            table.lookup(s.material_slot())?,
            "滑行流也要按**自己那个**材质槽映射"
        );
    }

    // —— 确定性：同输入两跑逐位一致 ——
    assert_eq!(banks_a, map_banks(&table, &cues)?, "映射确定性");
    assert_eq!(
        slide_jsonl_banked(&slides, &banks_s).lines().count(),
        slides.len(),
        "S2 的带 bank 写出与入参同长"
    );
    Ok(())
}

/// **判据 ③b：写出器的逐字节关系**（`bank` 的位置；`None` 分支 = S1/S2 基线原样）。
///
/// 这一条**不需要物理场景**：`Cue` 是普通结构体，合成几条 cue 就直接压写出器——它考的是**格式**，
/// 考"映射"的是上一条。
#[test]
fn bank_field_is_appended_at_the_line_tail() -> R {
    let table = SoundTable::from_entries(&[(0, 100), (1, 110)])?;
    let cues = vec![
        cue_with_material(0),
        cue_with_material(1),
        cue_with_material(0),
    ];
    let banks = map_banks(&table, &cues)?;
    let banked = jsonl_banked(&cues, &banks);
    let plain = jsonl(&cues);
    assert_eq!(banked.lines().count(), cues.len());

    // 不带 bank ⇒ 与老写出器**逐字节**相同（S1/S2 基线不受 S3 影响）。
    let mut via_none = Vec::new();
    write_jsonl_banked(&mut via_none, &cues, None)?;
    assert_eq!(
        via_none,
        plain.as_bytes(),
        "`None` 必须与 `jsonl` 逐字节相同（S1 基线）"
    );
    // 带 bank ⇒ 与 `jsonl_banked` 逐字相同。
    let mut via_some = Vec::new();
    write_jsonl_banked(&mut via_some, &cues, Some(&banks))?;
    assert_eq!(
        String::from_utf8_lossy(&via_some),
        banked,
        "两次带 bank 写出必须逐字一致"
    );
    // `bank` 必须**追加在行尾**（收尾 `}` 之前）：剥掉它之后每一行都与不带 bank 的那份**逐字**相同
    // —— 这条保证 S1/S2 基线的字段序不被 S3 搅动。
    for (i, (p, b)) in plain.lines().zip(banked.lines()).enumerate() {
        let head = p
            .strip_suffix('}')
            .ok_or_else(|| std::io::Error::other("每行以 `}` 收尾"))?;
        assert_eq!(
            b,
            format!("{head},\"bank\":{}}}", banks[i].0),
            "第 {i} 行：只有行尾（`}}` 前）多了 `,\"bank\":N`"
        );
    }
    Ok(())
}
