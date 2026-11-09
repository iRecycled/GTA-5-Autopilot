//! `scenario [files...]`: runs scenarios (default: every `scenarios/*.toml`)
//! and prints each run's metrics. Exits non-zero if any run fails.

use std::path::PathBuf;

fn main() {
    let mut files: Vec<PathBuf> = std::env::args().skip(1).map(PathBuf::from).collect();
    if files.is_empty() {
        let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
        files = std::fs::read_dir(&dir)
            .map(|rd| rd.filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.extension().is_some_and(|x| x == "toml")).collect())
            .unwrap_or_default();
        files.sort();
    }
    let mut failed = 0;
    for f in &files {
        let sc = match scenario::load(f) {
            Ok(s) => s,
            Err(e) => {
                println!("{e}");
                failed += 1;
                continue;
            }
        };
        let reports = scenario::run_all(&sc);
        let passed = reports.iter().filter(|r| r.passed()).count();
        println!("{} — {passed}/{} passed", sc.name, reports.len());
        for r in &reports {
            let surround = if r.bsd_truth != [0, 0] || r.bsd_false > 0 || r.rcta > 0 || r.park > 0 || r.clearance.is_finite() {
                format!(
                    " | BSD {}/{} L, {}/{} R, latency {:.2} s, false {} | RCTA {} | park {} | room {:.2} m",
                    r.bsd_detected[0], r.bsd_truth[0], r.bsd_detected[1], r.bsd_truth[1], r.bsd_latency, r.bsd_false, r.rcta, r.park, r.clearance
                )
            } else {
                String::new()
            };
            println!(
                "  run {:2}: {} | jerk {:.2} | gap err {:.2} s | AEB {} | min TTC {:.2} s | end {:.1} km/h, gap {:.1} m{surround}{}",
                r.run,
                if r.passed() { "pass" } else { "FAIL" },
                r.max_jerk,
                r.gap_error,
                r.aeb,
                r.min_ttc,
                r.end_speed * 3.6,
                r.end_gap,
                if r.failures.is_empty() { String::new() } else { format!(" | {}", r.failures.join("; ")) }
            );
        }
        if passed != reports.len() {
            failed += 1;
        }
    }
    if failed > 0 {
        std::process::exit(1);
    }
}
