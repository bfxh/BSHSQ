# SURVEY-SOFT-CLOTH-AND-CONVERSION · 分片索引

> 按 §编号/标题**点名取单片**，不要整份读 `SURVEY-SOFT-CLOTH-AND-CONVERSION.md`（≈51,799 估算 token）。分片是原文连续切片、§编号未重排；**原文仍是唯一权威版本**。

| 分片 | 起始行 | 估算 token |
|---|---|---|
| `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 第 1 行起 | 29,599 |
| `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 第 1120 行起 | 22,200 |

## 小节 → 分片

| §编号 | 标题 | 分片 | 原文行 |
|---|---|---|---|
| — | 调研：XPBD/布料 · 体素↔多边形转换的物理 · 外部仓库对标（2026-09-26） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 3 |
| 1 |   1. 现状：三条问题的答案（先说结论） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 32 |
| 1.1 |     1.1 XPBD —— **没做。零实现，只有骨架 + 死声明** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 34 |
| 1.2 |     1.2 体素↔多边形转换的物理 —— **四个子需求里，三个半是空白** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 55 |
| 1.3 |     1.3 外部仓库 `shyhdm/Opengl_Physx` —— **对 XPBD/布料/SPH 零可借鉴**（这条要直说） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 67 |
| 2 |   2. 精选待办（按"判据现成度 × 收益/代价"排；不动代码，供拍板） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 76 |
| — |     T1（M4 前置）把"三角形作为一等几何"补齐 —— **布料的最硬缺口，且不依赖 XPBD** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 78 |
| — |     T2（新维度）体素↔多边形"转换途中的物理" —— 拆成 **4 个可独立开关**的子特性 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 85 |
| — |     T3 借外部仓库的 4 条（三档判定，按用户既有的"吸收协议"） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 99 |
| 3 |   3. 与既有路线的关系（别重复造） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 109 |
| 4 |   4. 补记（2026-09-26）：把"窄相缺口"变成判据时撞到的**可见性墙** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 117 |
| 5 |   5. ✅ 缺口已**实测确认**（2026-09-26，判据 `crates/vxl-phys/tests/provider_shape_coverage.rs`，`cbdeb82`） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 145 |
| 5.1 |     5.1 下一步：修胶囊（判据现成，一条命令验收） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 163 |
| 6 |   6. ✅ 胶囊已修（2026-09-26，`vxl-phys-narrow/src/provider.rs::capsule_provider_contacts`） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 184 |
| 7 |   7. ✅ 圆柱 / 圆锥已修（2026-09-26 同日，`ring_provider_contacts`）——窄相缺口清单**清空** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 232 |
| 8 |   8. ✅ T1 绳索最小闭环已落地（2026-09-26 同日，`vxl-phys-soft::Rope`） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 288 |
| 8.1 |     8.1 ✅ 门面接线已落地（同日第二片） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 325 |
| 8.2 |     8.2 ✅ 摩擦（切向）已落地 —— 并抓到"蠕变"这个真 bug（同日第三片） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 346 |
| 8.3 |     8.3 ✅ 粒子↔刚体耦合（Akinci 式最小实现）已落地（同日第四片） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 374 |
| 8.4 |     8.4 ⚠️ 接触柔度/稳定：**量过了，真因是结构性的**（同日第五片；缺口已钉成判据） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 410 |
| 8.4.2 |       8.4.2 ⚠️ 又两个否定结果 —— 计数 4 次之后，**换模型**而不是继续调（同日） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 429 |
| 8.4.3 |       8.4.3 ⚠️ 评估结果：胶囊链 + 距离关节**不能直接替代**（100 tick 内 NaN 爆炸） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 453 |
| 8.4.4 |       8.4.4 ⭐ 仪器教训：**NaN 会被 `f32::max` 静默吞掉**（本轮实测，值得全仓记） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 464 |
| 8.4.5 |       8.4.5 📐 下一刀：**"穿过面法线"**（"段 vs 面"的最小实现）—— 精确到可照着写 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 472 |
| 8.4.5.1 |         8.4.5.1 ⚠️ 照着上面写出来实测：**穿越测试只在"进来的那一子步"成立**（已回退，但换回了确切规格） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 500 |
| 8.4.5.2 |         8.4.5.2 ⚠️ 照 8.4.5.1 实现后的第二次实测：**仍零接触** ⇒ 先去"单元对拍"，别再往上叠 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 522 |
| 8.4.6 |       8.4.6 ⚠️ 根因已修（软体侧 −2091.5 → −7.9）＋ 门面级判据抓到两件事（同日） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 541 |
| 8.4.7 |       8.4.7 ✅ 对拍做完了：差异**不在"有没有接触"，而在"门面里接触不稳定"** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 565 |
| 8.4.7.1 |         8.4.7.1 ❌ 上一条"少走 2 个 tick"是**我算错了**（当场更正，留痕）+ 门面是**剧烈失稳** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 584 |
| 8.4.1 |       8.4.1 ⚠️ 入口法线**单用也不行**（同日实测） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 603 |
| 8.4.8 |       8.4.8 ✅ 门面被**免罪**：真因 = "接触集是 1 ulp 阈值的**离散选择器**"（2026-09-27 位级对拍） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 623 |
| 8.4.8.1 |         8.4.8.1 ❌ 上面那条"第一动作"**实测是回归**（同日）⇒ 负载路径不在接触法线，在**链的软** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 695 |
| 8.4.9 |         8.4.9 ❌ "链的软是负载路径"**也被实测否掉**（同日）＋**下沉 = 位置漏**这句量出来了 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 719 |
| 8.4.10 |         8.4.10 ✅ **"绳托住刚体"闭环成立**（软体侧）：位置口径回填 + 停用角反作用 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 765 |
| 8.4.11 |         8.4.11 ❌ 上面最后那句推断**被实测否掉**：拆掉离散选择器**对门面毫无帮助**（同日） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 798 |
| 8.4.12 |         8.4.12 ❌ 四个单变量臂**全不成立**：门面的残留既不是质量 ulp、也不是 2 子步积分 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 825 |
| 8.4.13 |         8.4.13 ✅ 机制找到了（门面 = 每 tick 少落 `a·h²`）＋ ❌ 但**忠实镜像反而托住** ⇒ 门面那一环在管线里 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 855 |
| 8.4.14 |         8.4.14 ⭐ "喂进去的那一份"打出来了 ⇒ **判决落在 ~1e-7 级：接触响应是阈值敏感的** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 895 |
| 8.4.15 |         8.4.15 ⭐ 匹配积分器的锁步：**矛盾确立**（位级同输入 + 同绳状态 ⇒ 响应仍不同） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 927 |
| 8.4.16 |         8.4.16 ⭐⭐ 谜底：**"自扮引擎"是 1 维的**（只积 `y`、只吃反作用的 `y` 分量）—— 全部"门面 vs 自扮"差异的总根 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 958 |
| 8.4.17 |         8.4.17 ⭐⭐ 定论：**两侧一致了**（3 维口径下门面与自扮引擎都托不住）⇒ 本会话"门面 vs 自扮"整条线索是**仪器维数**造成的 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 993 |
| 8.4.18 |         8.4.18 ✅ 横向漂移判据立起来了（把"滑出"与"下沉"拆成两种可分辨的形态） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 1027 |
| 8.4.19 |         8.4.19 ⭐ 杠杆找到了：**是摩擦**（`μ = 0` 时盒子 3 维下**托得住**）⇒ 竖直负载路径其实是通的 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 1049 |
| 8.4.20 |         8.4.20 ✅✅ 修好了：**摩擦的相对速度与法向同口径** ⇒ 盒子在绳上停住（3 维 + 门面级双绿） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p01.md` | 1083 |
| 8.4.21 |         8.4.21 ✅ 那一刀的**泛化检验**：13/14 组托住；**边界 = 轻盒**（0.25 kg 仍横向逃逸，漂移 ∝ 1/m） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1120 |
| 8.4.22 |         8.4.22 ⭐ 残余漂移的**标度**量出来了：不是"常量冲量偏置"，是**摩擦模型的轻载阈值** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1150 |
| 8.4.23 |         8.4.23 ⭐⭐ 把两个残余**分开了**：**顺序 GS 的偏差**（可修）＋**轻载摩擦regime**（另一件事） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1177 |
| 8.4.24 |         8.4.24 ✅ A 落地：**施加顺序改成几何的函数** ⇒ 数组方向不再影响结果（6/6 逐位相同） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1207 |
| 8.4.24.1 |         8.4.24.1 ❌ 先试的另一种"规范序"**把 0.5 kg 打坏了**（留档：GS 路径的选择决定落在哪个盆） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1232 |
| 8.4.25 |         8.4.25 ✅✅ B 成了：库仑锥取**冲量口径** `μ·w_p·λ` ⇒ **轻载 regime 修好**（0.25 kg 从巡航逃逸变托住） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1242 |
| 8.4.26 |         8.4.26 ✅ 泛化表升级为**在树判据**：新口径下 **14/14 格托住**（`rope_coupling_robustness.rs`） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1270 |
| 8.4.27 |         8.4.27 ✅ 睡眠体对软体域**呈现为静态**（一行）⇒ 绳不再打扰睡眠体（含**金丝雀验证**） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1287 |
| 8.4.28 |         8.4.28 ✅ 角冲量注入的**契约**钉住了（2× 口径缺陷：从"记录的隐患"变成"被判据守住的规格"） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1311 |
| 8.4.29 |         8.4.29 ✅ 2c-1 落地：**虚拟位姿带转动**（零换代门逐位通过）——计划见 `PLAN-rope-rotation-aware.md` | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1329 |
| 8.4.30 |         8.4.30 ✅ 2c-2 落地：法向与摩擦**一次改齐**到"接触点速度" `v + ω × (q − vpos)` | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1359 |
| 8.4.31 |         8.4.31 ✅ 2c-3 落地（**默认关闭的开关 + 实测登记**）：角反作用可用、符号对、托住不崩 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1384 |
| 8.4.32 |         8.4.32 ✅ 最后一块拼图量到了：**开了角反作用，中心场景照旧托住**（判据已入树） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1416 |
| 8.4.33 |         8.4.33 ✅ T1a 落地：**三角形进 `Shape`**（几何层：变体 + 仓库 + 薄壳质量 + 宽相 + 支持矩阵） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1439 |
| 8.4.34 |         8.4.34 ✅ T1b-1 落地：**三角网 × 提供者**（逐顶点采样）⇒ 判据 ① 的 provider 腿通了 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1478 |
| 8.4.35 |         8.4.35 ✅ T1b-2 落地：**三角网 × {盒, 球}**（凸体腿，解析逐顶点）⇒ 判据 ① 的凸体腿也通了 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1505 |
| 8.4.36 |         8.4.36 ✅ T1b-3 落地：**三角网 × 高度场**（缺口 #6 关闭）⇒ 判据 ① 四条腿全通 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1533 |
| 8.4.37 |         8.4.37 ✅ T1b-4 落地：三角网 × {胶囊, 圆柱} ⇒ 判据 ① **六条腿全通** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1558 |
| 8.4.39 |         8.4.39 ✅ T2 第一片落地：**点-三角**（外壳 × 三角网）—— 石块被布片托住 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1583 |
| 8.4.40 |         8.4.40 ✅ T4 第一片落地：**面元气动**（判据 ③）—— `vxl-phys-aero` 从 28 行配置变消费方 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1603 |
| 8.4.41 |         8.4.41 ✅ 软体切片 1 落地：**布料最小闭环**（`ClothSheet`：三角网 + XPBD 距离约束） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1616 |
| 8.4.42 |         8.4.42 ✅ 软体切片 2/2b-i 落地：布片**提供者接触 + 静态刚体** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1644 |
| 8.4.43 |         8.4.43 ✅ 软体切片 2b-ii 落地：布片 × **动态刚体**（反作用两腿）——并抓到"位置腿不进虚拟位姿 ⇒ 弹跳"这条真缺陷 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1664 |
| 8.4.44 |         8.4.44 ✅ 软体切片 T3 落地：**布片自碰撞**（空间哈希 + 默认关开关）⇒ **判据 ② 达成（含金丝雀）** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1737 |
| 8.4.45 |         8.4.45 ✅ **判据 ④ 落地（门面级）：布片 × 动态刚体双向** —— 计划的 ③④ 两条判据齐了 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1793 |
| 8.4.46 |         8.4.46 ✅ 切片 T4 收尾：**风由布片消费**（`ClothSheet` 接 `vxl_phys_aero::face_force`） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1822 |
| 8.4.47 |         8.4.47 ✅ 还欠账（气动零分配）+ **T2 续的口径决定**（下一片直接开工的那条） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1859 |
| 8.4.48 |         8.4.48 ✅ **T2 续落地：三角网 × 三角网**（双面口径）——**计划的 4 条判据 + 代码触点全部闭环** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1889 |
| 8.4.49 |         8.4.49 ✅ mesh×mesh 的**空间加速**：O(n⁴) → ~O(n²)，且**读数逐位不变** | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1919 |
| 8.4.50 |         8.4.50 ✅ **布料撕裂**（`TearStrain` 骨架的落点）——"超阈即断"，默认关 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1949 |
| 8.4.51 |         8.4.51 ✅ **布片自摩擦**（切向库仑锥）——T3 登记的边界关门，默认关 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 1982 |
| 8.4.50 |         8.4.50 ✅ **rope 也吃到那一刀**（2026-10-01）：位置腿进 `body_disp` —— 有界换代 | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 2015 |
| 8.4.52 |         8.4.52 ⚠️ 「布片↔动态代理冻结夹具穿落」调查收口：**否证两条解释 + 夹具语义定论**（2026-10-01） | `SURVEY-SOFT-CLOTH-AND-CONVERSION.p02.md` | 2037 |
