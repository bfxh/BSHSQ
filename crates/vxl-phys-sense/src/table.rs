//! table：**S3 材质表 / 资产映射**（外部表 → 音色组 id；`docs/ROUTE.md` §3.2）。
//!
//! 前两片（S1 冲击、S2 持续接触）产出的 cue 只带**材质槽**（`bodies.material[body]`，即 cue 的
//! `mat` 字段）——那是**物理**侧的概念（摩擦 / 恢复系数）。"这个材质该发什么声音"是**资产**侧
//! 的概念：换一套音效包只该换一张表，不该动引擎。所以本片是一条**纯映射**：
//!
//! ```text
//! 材质槽（物理，u32）── 外部表 ──→ 音色组 id（资产，不透明 u32）
//! ```
//!
//! **为什么不往 `Material` 塞声学字段**：`Material` 是棘轮管的类型（`god.gate.json` 登记了成员
//! 数），且"摩擦系数"与"用哪个音色包"的变化速率完全不同——把后者焊进前者，材质表一改就要动物理
//! 类型。
//!
//! **核心判据：缺项显式拒绝**（`docs/ROUTE.md` §3.2 唯一一条）——表里没有的材质槽**不静默回落
//! 到"默认音"**，而是 [`TableError::Missing`]。理由：静默默认音在听感上把"未知材质"伪装成"已知
//! 材质"，混响 / 配音错了只在后期才被人耳发现；显式拒绝让缺项在 CI 与首次运行就炸出来。同理，
//! 同一材质槽登记两次也**拒绝**（[`TableError::Duplicate`]），不静默取后者。
//!
//! **无状态、无 HashMap**：本片是作用在 cue 切片与表上的**纯函数**（[`map_banks`]）——不碰
//! `World`、不借 `&mut` ⇒ S1 的三条契约（零干扰 / 确定性 / 预算）在这里天然成立。表用**按材质槽
//! 升序的向量 + 二分查找**存储 ⇒ 查找结果与迭代序都与登记顺序无关，也不含哈希迭代序。

use crate::{Cue, SlideCue};

/// **音色组 id**（资产侧的不透明编号）。本域只做**搬运**，不解释它的内部含义（也不假设声部数 /
/// 变体数——那是音效包的私事）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BankId(pub u32);

/// **材质表的错**（S3 唯一的判据就在这里：**都拒绝，都不静默回落**）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableError {
    /// 表里没有这个材质槽 ⇒ **拒绝**（绝不回落"默认音"）。
    Missing {
        /// 缺失的材质槽（就是 cue 的 `mat` 字段）。
        material: u32,
    },
    /// 表文本第 `line` 行（1 起）不合法。
    Syntax {
        /// 行号（1 起）。
        line: usize,
        /// 为什么不行（`&'static str` ⇒ 错误类型保持 `Copy`，文案也不会被上游数据污染）。
        reason: &'static str,
    },
    /// 同一材质槽登记了两次 ⇒ **拒绝**（不静默取后者）。
    Duplicate {
        /// 行号（1 起；经 [`SoundTable::from_entries`] 构造时是**条目序号**）。
        line: usize,
        /// 重复的材质槽。
        material: u32,
    },
}

impl std::fmt::Display for TableError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TableError::Missing { material } => {
                write!(
                    f,
                    "音色表缺项：材质槽 {material} 未登记（显式拒绝，不回落默认音）"
                )
            }
            TableError::Syntax { line, reason } => {
                write!(f, "音色表第 {line} 行语法错误：{reason}")
            }
            TableError::Duplicate { line, material } => write!(
                f,
                "音色表第 {line} 行重复登记材质槽 {material}（显式拒绝，不静默取后者）"
            ),
        }
    }
}

impl std::error::Error for TableError {}

/// **音色表**：材质槽 → 音色组 id（外部资产的解析结果）。
///
/// 内部是**按材质槽升序**的 `Vec<(u32, BankId)>`：查表走二分、[`SoundTable::entries`] 迭代序稳定
/// ⇒ 工具链输出与"登记顺序"无关（确定性）。**空表不是语法错误**（表可以按大类分文件），但空表
/// 查任何槽都会 [`TableError::Missing`]。
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SoundTable {
    /// 按 `material` 升序、无重复。
    entries: Vec<(u32, BankId)>,
}

impl SoundTable {
    /// **解析外部表文本**：
    ///
    /// · 整行空白 ⇒ 跳过；整行以 `#` 起 ⇒ 注释，跳过（**不支持行内注释**：一行要么整行注释，
    ///   要么恰好一条映射——没有引号机制，切坏了更难查）；
    /// · 否则该行必须形如 `<材质槽> -> <音色组 id>`，两侧都是 `u32`。
    ///
    /// 任一行不合法 ⇒ **整表拒绝**（[`TableError::Syntax`] / [`TableError::Duplicate`]），不做
    /// "跳过坏行"——那会让缺项推迟到运行时才暴露。
    pub fn parse(text: &str) -> Result<Self, TableError> {
        let mut table = Self::default();
        for (i, raw) in text.lines().enumerate() {
            let line = i + 1;
            let s = raw.trim();
            if s.is_empty() || s.starts_with('#') {
                continue;
            }
            let (lhs, rhs) =
                split_arrow(s).map_err(|reason| TableError::Syntax { line, reason })?;
            let material = lhs.parse::<u32>().map_err(|_| TableError::Syntax {
                line,
                reason: "左侧材质槽不是 u32",
            })?;
            let bank = rhs.parse::<u32>().map_err(|_| TableError::Syntax {
                line,
                reason: "右侧音色组 id 不是 u32",
            })?;
            table.insert(material, BankId(bank), line)?;
        }
        Ok(table)
    }

    /// 程序构造（判据与内嵌小表用）。重复登记 ⇒ [`TableError::Duplicate`]，
    /// 其 `line` 是**条目序号**（1 起）。
    pub fn from_entries(entries: &[(u32, u32)]) -> Result<Self, TableError> {
        let mut table = Self::default();
        for (i, (material, bank)) in entries.iter().enumerate() {
            table.insert(*material, BankId(*bank), i + 1)?;
        }
        Ok(table)
    }

    /// **查表**：登记了 ⇒ `Ok`；没登记 ⇒ [`TableError::Missing`]（**不回落默认音**）。
    pub fn lookup(&self, material: u32) -> Result<BankId, TableError> {
        self.entries
            .binary_search_by_key(&material, |e| e.0)
            .map(|i| self.entries[i].1)
            .map_err(|_| TableError::Missing { material })
    }

    /// 登记条数。
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// 是否一张空表（空表合法，但查任何槽都会 [`TableError::Missing`]）。
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// 全部条目（**按材质槽升序** ⇒ 迭代序稳定，工具链/判据据此逐条对拍）。
    pub fn entries(&self) -> &[(u32, BankId)] {
        &self.entries
    }

    /// 插入一条（保持升序）；已登记 ⇒ [`TableError::Duplicate`]。
    fn insert(&mut self, material: u32, bank: BankId, line: usize) -> Result<(), TableError> {
        match self.entries.binary_search_by_key(&material, |e| e.0) {
            Ok(_) => Err(TableError::Duplicate { line, material }),
            Err(pos) => {
                self.entries.insert(pos, (material, bank));
                Ok(())
            }
        }
    }
}

/// 能报出自己**材质槽**的 cue（S1 的 [`Cue`] 与 S2 的 [`SlideCue`] 都实现）——S3 的映射由此与
/// "两条流"解耦：**表对流一视同仁**，加第三条流也不用改本片。
pub trait MaterialSlot {
    /// cue 的材质槽（= `bodies.material[body]`）。
    fn material_slot(&self) -> u32;
}

impl MaterialSlot for Cue {
    fn material_slot(&self) -> u32 {
        self.material
    }
}

impl MaterialSlot for SlideCue {
    fn material_slot(&self) -> u32 {
        self.material
    }
}

/// **映射一条 cue 流 → 音色组 id 流**（与入参**同序、同长**）。
///
/// **全有或全无**：任一 cue 的材质槽缺项 ⇒ **整条 `Err`**（[`TableError::Missing`]，在流序上取
/// **第一个**缺项 ⇒ 报错可复现）。不做"部分映射"——半张表会让音效师拿到一半有音、一半静音的流，
/// 比直接报错更难查。
pub fn map_banks<T: MaterialSlot>(
    table: &SoundTable,
    cues: &[T],
) -> Result<Vec<BankId>, TableError> {
    let mut out = Vec::with_capacity(cues.len());
    for c in cues {
        out.push(table.lookup(c.material_slot())?);
    }
    Ok(out)
}

/// 切出 `->` 两侧（`->` 必须恰有一个）。
fn split_arrow(s: &str) -> Result<(&str, &str), &'static str> {
    let mut it = s.split("->");
    let lhs = it.next().unwrap_or("");
    let rhs = it
        .next()
        .ok_or("缺少 `->`（本行既不是注释，也不是 `<材质槽> -> <音色组 id>`）")?;
    if it.next().is_some() {
        return Err("`->` 多于一个");
    }
    Ok((lhs.trim(), rhs.trim()))
}
