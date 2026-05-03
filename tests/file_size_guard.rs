//! Regression guard: asserts every .rs file in src/ is under 2000 lines.
//! This enforces the CLAUDE.md "files must be less than 2000 lines" policy
//! automatically, so future modifications that would breach the limit fail CI.
use std::fs;
use std::path::PathBuf;

const MAX_LINES: usize = 2000;

fn collect_rs_files(dir: &PathBuf, out: &mut Vec<PathBuf>) -> std::io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let p = entry?.path();
        if p.is_dir() {
            if p.file_name().and_then(|s| s.to_str()) == Some("target") {
                continue;
            }
            collect_rs_files(&p, out)?;
        } else if p.extension().and_then(|s| s.to_str()) == Some("rs") {
            out.push(p);
        }
    }
    Ok(())
}

#[test]
fn no_source_file_exceeds_2000_lines() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let src = manifest_dir.join("src");
    let mut files = Vec::new();
    collect_rs_files(&src, &mut files).expect("could not walk src/");
    let mut violations: Vec<String> = Vec::new();
    for f in &files {
        let count = match fs::read_to_string(f) {
            Ok(c) => c.lines().count(),
            Err(_) => continue,
        };
        if count >= MAX_LINES {
            violations.push(format!("  {}: {} lines", f.display(), count));
        }
    }
    assert!(
        violations.is_empty(),
        "Files at or over {} lines (CLAUDE.md violation — must be < 2000):\n{}",
        MAX_LINES,
        violations.join("\n")
    );
}
