//! wide_insert：从 wide_impl.rs 按族拆出（纯搬移，语义未改）。
use super::*;
// 共享 AABB 助手在 crate 根（`aabb_util`）：孙模块的 `use super::*` 只到父模块 ⇒ 显式点名。
use crate::aabb_util::*;

impl WideBvh {
    /// 增量插入：返回叶下标。
    ///
    /// 下行 = 自根逐层取「并入后周长增量最小」的子（平局取小下标 ⇒ 确定性）；
    /// 落叶有位则就地落位 + `refit_upwards`（仅叶盒变化 ⇒ 提前退出版合法）；
    /// 叶满则切 4+5 并**向上传播溢出**（见模块注释）。
    pub fn insert(&mut self, body: u32, aabb: Aabb) -> u32 {
        self.ensure_capacity(body as usize);
        self.body_box[body as usize] = aabb;
        if self.root == NULL {
            let mut leaf = WideNode {
                aabb,
                height: 0,
                parent: NULL,
                slot: [0; WIDE],
                count: 1,
            };
            leaf.slot[0] = body;
            let leaf = self.push_node(leaf);
            self.root = leaf;
            self.leaf_of[body as usize] = leaf;
            return leaf;
        }
        let mut cur = self.root;
        while self.nodes[cur as usize].height != 0 {
            let n = self.nodes[cur as usize].count as usize;
            let mut best = 0usize;
            let mut best_inc = f32::MAX;
            for k in 0..n {
                let child = self.nodes[cur as usize].slot[k] as usize;
                let inc = enlargement(&self.nodes[child].aabb, &aabb);
                if inc < best_inc {
                    best_inc = inc;
                    best = k;
                }
            }
            cur = self.nodes[cur as usize].slot[best];
        }
        let leaf = cur;
        let count = self.nodes[leaf as usize].count as usize;
        self.leaf_of[body as usize] = leaf;
        if count < WIDE {
            // 叶有位：落位 + 自父向上 refit（仅叶盒变化 ⇒ 可用提前退出版）。
            let node = &mut self.nodes[leaf as usize];
            node.slot[count] = body;
            node.count = (count + 1) as u8;
            node.aabb = union(&node.aabb, &aabb);
            let parent = self.nodes[leaf as usize].parent;
            self.refit_upwards(parent);
            return leaf;
        }
        // 叶满（8 体）：第 9 体不入槽，直接交给分裂（split_leaf 以 extra 接收）。
        let sib = self.split_leaf(leaf, body);
        self.settle_split(leaf, sib);
        self.leaf_of[body as usize]
    }

    /// 叶溢出（8 旧体 + `extra` 新体 = 9）⇒ 按最长轴排序切成 4+5：
    /// `leaf` 留前 4 体，新叶（返回值）带走后 5 体。两叶的盒/体数/`leaf_of`
    /// 全部就位；新叶父指针待 `settle_split` 挂接。
    pub(crate) fn split_leaf(&mut self, leaf: u32, extra: u32) -> u32 {
        let n = self.nodes[leaf as usize].count as usize;
        debug_assert_eq!(n, WIDE);
        let mut items: Vec<(u32, Aabb)> = (0..n)
            .map(|k| {
                let b = self.nodes[leaf as usize].slot[k];
                (b, self.body_box[b as usize])
            })
            .collect();
        items.push((extra, self.body_box[extra as usize]));
        // 轴取自 9 体并集（叶盒可能不含新体、且可能被 fat 边距撑大 ⇒ 不复用叶盒）。
        let mut bound = items[0].1;
        for (_, b) in items.iter().skip(1) {
            bound = union(&bound, b);
        }
        let axis = longest_axis(&bound);
        items.sort_by(|a, b| cmp_axis_center(a, b, axis));
        let keep = items.len() / 2; // 4
        let mut lslot = [0u32; WIDE];
        let mut rslot = [0u32; WIDE];
        let mut lbox = items[0].1;
        let mut rbox = items[keep].1;
        for (k, (b, bx)) in items.iter().enumerate() {
            if k < keep {
                lslot[k] = *b;
                lbox = union(&lbox, bx);
            } else {
                rslot[k - keep] = *b;
                rbox = union(&rbox, bx);
            }
        }
        let sib = self.push_node(WideNode {
            aabb: rbox,
            height: 0,
            parent: NULL,
            slot: rslot,
            count: (items.len() - keep) as u8,
        });
        {
            let node = &mut self.nodes[leaf as usize];
            node.slot = lslot;
            node.count = keep as u8;
            node.aabb = lbox;
        }
        for (b, _) in items.iter().take(keep) {
            self.leaf_of[*b as usize] = leaf;
        }
        for (b, _) in items.iter().skip(keep) {
            self.leaf_of[*b as usize] = sib;
        }
        sib
    }

    /// 内部节点溢出（8 子 + `extra` 新子 = 9）⇒ 按最长轴排序切成 5+4：
    /// `node` 留前 5 子，新兄弟（返回值）带走后 4 子。子父指针 / 盒 / 高 /
    /// 子数全部就位；`node.parent` 不动（新兄弟的父指针待 `settle_split` 挂接）。
    ///
    /// 切分不改变「并集盒」与「最大子高」⇒ 祖先的盒/高在切分前后逐位相同。
    pub(crate) fn split_node(&mut self, node: u32, extra: u32) -> u32 {
        let n = self.nodes[node as usize].count as usize;
        debug_assert_eq!(n, WIDE);
        // 先收养 `extra`：它尚未挂接（父指针为空），若排序后落在**保留侧**，
        // 下面的「右侧改父」循环不会碰它 ⇒ 必须现在指向 `node`。
        self.nodes[extra as usize].parent = node;
        let mut kids: Vec<(u32, Aabb)> = (0..n)
            .map(|k| {
                let c = self.nodes[node as usize].slot[k];
                (c, self.nodes[c as usize].aabb)
            })
            .collect();
        kids.push((extra, self.nodes[extra as usize].aabb));
        let mut bound = kids[0].1;
        for (_, b) in kids.iter().skip(1) {
            bound = union(&bound, b);
        }
        let axis = longest_axis(&bound);
        kids.sort_by(|a, b| cmp_axis_center(a, b, axis));
        let keep = kids.len().div_ceil(2); // 5
        let mut lslot = [0u32; WIDE];
        let mut rslot = [0u32; WIDE];
        for (k, (c, _)) in kids.iter().enumerate() {
            if k < keep {
                lslot[k] = *c;
            } else {
                rslot[k - keep] = *c;
            }
        }
        let (lbox, lh) = self.children_bound(&lslot[..keep]);
        let (rbox, rh) = self.children_bound(&rslot[..kids.len() - keep]);
        let sib = self.push_node(WideNode {
            aabb: rbox,
            height: rh + 1,
            parent: NULL,
            slot: rslot,
            count: (kids.len() - keep) as u8,
        });
        for (c, _) in kids.iter().skip(keep) {
            self.nodes[*c as usize].parent = sib;
        }
        {
            let kept = &mut self.nodes[node as usize];
            kept.slot = lslot;
            kept.count = keep as u8;
            kept.aabb = lbox;
            kept.height = lh + 1;
        }
        sib
    }

    /// 子下标集的 (并集盒, 最大子高)。
    pub(crate) fn children_bound(&self, kids: &[u32]) -> (Aabb, u32) {
        debug_assert!(!kids.is_empty());
        let mut aabb = self.nodes[kids[0] as usize].aabb;
        let mut h = self.nodes[kids[0] as usize].height;
        for c in kids.iter().skip(1) {
            let cn = self.nodes[*c as usize];
            aabb = union(&aabb, &cn.aabb);
            h = h.max(cn.height);
        }
        (aabb, h)
    }

    /// 把 `sib` 挂到 `child` 的父上：父有位 ⇒ 挂尾 + `refit_upwards`（提前退出
    /// 合法：祖先只经盒/高依赖本层）；父已满 ⇒ `split_node` 切分后继续向上；
    /// 一路到根仍溢出 ⇒ 新建根（宽 2）。**仅此一处**新建根。
    pub(crate) fn settle_split(&mut self, mut child: u32, mut sib: u32) {
        loop {
            let p = self.nodes[child as usize].parent;
            if p == NULL {
                // child 是根 ⇒ 建新根（两个子等高，故高 = 子高 + 1）。
                let ch = self.nodes[child as usize].height;
                let sh = self.nodes[sib as usize].height;
                let mut root = WideNode {
                    aabb: union(
                        &self.nodes[child as usize].aabb,
                        &self.nodes[sib as usize].aabb,
                    ),
                    height: ch.max(sh) + 1,
                    parent: NULL,
                    slot: [0; WIDE],
                    count: 2,
                };
                root.slot[0] = child;
                root.slot[1] = sib;
                let root = self.push_node(root);
                self.nodes[child as usize].parent = root;
                self.nodes[sib as usize].parent = root;
                self.root = root;
                return;
            }
            let pc = self.nodes[p as usize].count as usize;
            if pc < WIDE {
                self.nodes[sib as usize].parent = p;
                let node = &mut self.nodes[p as usize];
                node.slot[pc] = sib;
                node.count = (pc + 1) as u8;
                self.refit_upwards(p);
                return;
            }
            // 父已满：切 5+4（新兄弟 = 多出的那个），继续向上传播。
            let sib2 = self.split_node(p, sib);
            child = p;
            sib = sib2;
        }
    }

    pub(crate) fn push_node(&mut self, node: WideNode) -> u32 {
        self.nodes.push(node);
        (self.nodes.len() - 1) as u32
    }
}
