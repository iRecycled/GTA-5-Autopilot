//! Every scenario in `scenarios/` must pass (this is how `cargo xtask test`
//! gates the controllers without a game, D20).

use std::path::PathBuf;

#[test]
fn all_scenarios_pass() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
        .expect("scenarios/ directory")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    files.sort();
    assert!(!files.is_empty(), "no scenarios found");
    let mut failures = Vec::new();
    for f in &files {
        let sc = scenario::load(f).unwrap();
        for r in scenario::run_all(&sc) {
            if !r.passed() {
                failures.push(format!("{} run {}: {}", sc.name, r.run, r.failures.join("; ")));
            }
        }
    }
    assert!(failures.is_empty(), "failing scenarios:\n{}", failures.join("\n"));
}
