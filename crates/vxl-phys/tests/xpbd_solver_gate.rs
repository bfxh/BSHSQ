//! **共求解器的立项门**（《LSSMJ-CICD…v5》§15.4）。
//!
//! `ElementKind::Xpbd` 至今**零消费方**（只有定义、没有使用）。这条事实必须有门看着 ——
//! 否则谁"顺手接线"就等于把「与刚体共求解器」从 `unresolved` 悄悄改成了 `implemented`，
//! 而 §15.4 明说那件事**必须先过独立设计评审 + 基线场景**（共享状态 / 约束雅可比与残差 /
//! 质量与惯量 / 求解顺序 / 收敛标准），**不得塞进常规 CI 清理 PR**。
//!
//! **判据**：全仓 `crates/**/*.rs` 里 `ElementKind::Xpbd` 出现次数 == 0。
//! 一旦有人开始消费它 ⇒ 本测试红 ⇒ 被迫回来看这条立项门（补设计评审，或回退）。
//!
//! ⚠️ **这不是"禁止实现"**：走完立项门之后，把本测试与
//! `docs/CAPABILITIES.yaml` 里的 `PHY-SCOPE-UNIFIED-SOLVER` 一起更新，是**正常路径**。
//! 门拦的是"没走流程就接线"。
//!
//! 对应的登记条目：`PHY-SCOPE-UNIFIED-SOLVER`（`scope: unresolved`）；与之相邻但**不同**的
//! `PHY-SCOPE-XPBD-COUPLING`（`implementation: coupled`）是现状，两者不许混为一谈。
use std::path::{Path, PathBuf};

/// 拼接出来的待查串：让**本文件自身**不匹配，否则测试会把自己数进去。
fn needle() -> String {
    format!("ElementKind::{}", "Xpbd")
}

/// 递归收集 `dir` 下的 `.rs`（跳过 `target` / `.git`）。
fn collect_rs(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for e in entries.flatten() {
        let p = e.path();
        if p.is_dir() {
            let skip = p.file_name().is_some_and(|n| n == "target" || n == ".git");
            if !skip {
                collect_rs(&p, out);
            }
        } else if p.extension().is_some_and(|x| x == "rs") {
            out.push(p);
        }
    }
}

/// **判据**：`ElementKind::Xpbd` 全仓零消费方（见文件头注：门拦的是"没走立项门就接线"）。
#[test]
fn xpbd_element_kind_still_has_no_consumers() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let crates = root.join("crates");
    let mut files = Vec::new();
    collect_rs(&crates, &mut files);
    assert!(
        files.len() > 100,
        "只扫到 {} 个源文件（root={}）⇒ 门失效，先修扫描而不是放行",
        files.len(),
        root.display()
    );
    let needle = needle();
    let mut hits: Vec<String> = Vec::new();
    for f in &files {
        // 跳过本文件：它的头注为了说明这条门，写了完整的待查串（自匹配不是消费方）。
        if f.file_name().is_some_and(|n| n == "xpbd_solver_gate.rs") {
            continue;
        }
        let Ok(text) = std::fs::read_to_string(f) else {
            continue;
        };
        if text.contains(&needle) {
            hits.push(f.display().to_string());
        }
    }
    println!("扫描 {} 个 .rs，命中 {} 处", files.len(), hits.len());
    assert!(
        hits.is_empty(),
        "`ElementKind::Xpbd` 出现了消费者 ⇒ 共求解器不得这样悄悄接线（§15.4 立项门要求先过\
         独立设计评审 + 基线场景）；若确已走完流程，请同时更新本测试与 \
         docs/CAPABILITIES.yaml 的 PHY-SCOPE-UNIFIED-SOLVER：{hits:#?}"
    );
}
