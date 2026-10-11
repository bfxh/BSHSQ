//! 取 argv → 调 `run` → 打印/退出码（逻辑都在 lib.rs，便于判据直接测函数）。

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match vxl_phys_cli::run(&args) {
        Ok(text) => print!("{text}"),
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    }
}
