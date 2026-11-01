//! # vxl-phys-gpu
//!
//! GPU 后端（§8，纯 Rust 路线）—— M4。统一入口 trait；主实现 = wgpu（Vulkan/Metal/DX12/WebGPU），
//! 着色器 = rust-gpu → SPIR-V；NVIDIA 极致路径 = cuda-oxide SIMT + C ABI 主机桥（唯一允许的独立后端）。
//!
//! 硬性要求：**必须 CPU 回退且结果一致**（同档参数下位姿哈希一致，容差 = 严格模式量化容差）；必须出 GPU vs CPU 对比数据；禁止单一厂商绑定。

#![forbid(unsafe_code)]

pub mod acoustic;
pub mod backend;
pub mod bbox;
pub mod broad;
pub mod grid;
pub mod mapped;
pub mod narrow;
pub mod pipeline;
pub mod probe;
pub mod solve_tier;
pub use backend::{CpuFallback, PhysGpuBackend};
