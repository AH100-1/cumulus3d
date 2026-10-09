//! skyrecon 실행 파일의 라이브러리 부분: 점진 스트림 파이프라인과 단계별 호환 하위 명령([`interop`]).
//!
//! - [`session`]: 상태 값 [`session::Session`] 과 리듀서 함수 [`session::step`] / [`session::poll`] /
//!   [`session::command`] / [`session::finish`].
//! - [`pipeline`]: 리듀서를 감싸 훅·큐 정책·패닉 격리·구독 채널을 제공하는 [`pipeline::Pipeline`].
//! - [`events`]: 단계별 결과 [`events::Event`] 와 명령 [`events::Command`].
//! - [`sinks`]: `skyrecon stream` 의 파일 출력(timeline.txt, run.log, PLY, 스냅샷)을 만드는 기본 훅.
//! - [`stream`]: 입력 폴더 → 세션 → 파이프라인 조립([`stream::run_stream`]).
//!
//! # 사용 예
//!
//! 영상 폴더(`images/camF|camR|camL/*.jpg`, `gps_ref.txt`)를 위치 단위로 넣어 점진 재구성하고,
//! 기본 출력 훅으로 `skyrecon stream` 과 같은 파일을 만든다.
//!
//! ```no_run
//! use skyrecon_cli::events::{Event, EventKind};
//! use skyrecon_cli::pipeline::Pipeline;
//! use skyrecon_cli::session::{Input, Session, SessionConfig};
//! use skyrecon_cli::sinks::{self, SinkOptions};
//! use skyrecon_cli::stream::{frame_set, Layout};
//! use skyrecon_core::io::read_gps_file;
//! use std::path::Path;
//!
//! fn main() -> Result<(), String> {
//!     let src = Path::new("data");
//!     let layout = Layout::discover(src, 3)?;
//!     let mut cfg = SessionConfig::new(src.join("images"));
//!     cfg.gps = read_gps_file(src.join("gps_ref.txt")).map_err(|e| e.to_string())?;
//!     cfg.total_positions = Some(layout.frames.len());
//!
//!     let pipeline = Pipeline::new(Session::new(cfg)).on(EventKind::ZonePreview, |e: &Event| {
//!         if let Event::ZonePreview { range, cloud, .. } = e {
//!             println!("초벌 구역 {}: 점 {}", range.zone, cloud.len());
//!         }
//!     });
//!     let mut pipeline = sinks::attach(pipeline, Path::new("out"), &SinkOptions::default()).map_err(|e| e.to_string())?;
//!     for p in 0..layout.frames.len() {
//!         pipeline.push(Input::Frames(frame_set(&layout, p)));
//!     }
//!     let (_session, summary) = pipeline.finish();
//!     println!("이벤트 {} 개", summary.events);
//!     Ok(())
//! }
//! ```
//!
//! 훅 없이 상태 값만 주고받는 함수형 사용:
//!
//! ```no_run
//! use skyrecon_cli::session::{finish, step, FrameSet, Session, SessionConfig};
//!
//! let s0 = Session::new(SessionConfig::new("data/images"));
//! let frames = FrameSet::new([("camF", "camF/camF_0000.jpg"), ("camR", "camR/camR_0000.jpg")]);
//! let (s1, events) = step(&s0, frames);
//! for e in &events {
//!     if let Some(t) = e.timeline_text() {
//!         println!("{t}");
//!     }
//! }
//! let (_done, _summary_events) = finish(&s1);
//! ```

#![warn(missing_docs)]

pub mod interop;
pub mod densewrap;
pub mod events;
pub mod fusion_variants;
pub mod pipeline;
pub mod post;
pub mod session;
pub mod sinks;
pub mod stream;
pub mod util;
