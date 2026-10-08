//! GPS 참조 파일: 줄마다 `이름 위도 경도 고도`

use crate::error::{Error, Result};
use std::path::Path;

#[derive(Clone, Debug, PartialEq)]
pub struct GpsRecord {
    pub name: String,
    /// 도(십진).
    pub lat: f64,
    pub lon: f64,
    /// 미터.
    pub alt: f64,
}

/// 파일 순서를 유지해 읽는다(첫 줄이 ENU 원점). 빈 줄은 건너뛰고, 주석 문법은 없다(`#` 줄은 오류).
pub fn read_gps_file(path: impl AsRef<Path>) -> Result<Vec<GpsRecord>> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path)?;
    parse_gps(&text, path)
}

pub(crate) fn parse_gps(text: &str, path: &Path) -> Result<Vec<GpsRecord>> {
    let mut out = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        // 설계 결정: 필드가 4개를 넘는 줄은 오류로 본다.
        if f.len() != 4 {
            return Err(Error::parse(path, ln + 1, format!("필드 4개 필요, {}개", f.len())));
        }
        let num = |s: &str| s.parse::<f64>().map_err(|_| Error::parse(path, ln + 1, format!("숫자 아님: {s}")));
        out.push(GpsRecord { name: f[0].to_string(), lat: num(f[1])?, lon: num(f[2])?, alt: num(f[3])? });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse() {
        let t = "camF/a.jpg 37.5 127.0 50\n\n  camR/b.jpg 37.5001 127.0001 60.5  \n";
        let r = parse_gps(t, Path::new("x")).unwrap();
        assert_eq!(r.len(), 2);
        assert_eq!(r[1], GpsRecord { name: "camR/b.jpg".into(), lat: 37.5001, lon: 127.0001, alt: 60.5 });
        assert!(parse_gps("# c\n", Path::new("x")).is_err());
        assert!(parse_gps("a 1 2\n", Path::new("x")).is_err());
    }
}
