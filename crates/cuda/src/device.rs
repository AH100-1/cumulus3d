/*
 * device.rs
 *
 * Copyright (c) 2026 ParkSangWoo
 *
 * Author(s):
 *
 *      ParkSangWoo <dev.parksangwoo@gmail.com>
 *
 *
 * This file is part of cumulus3d.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//! Shared part for device, context and NVRTC compilation.

use cudarc::driver::{CudaContext, CudaModule, CudaStream};
use cudarc::nvrtc::{compile_ptx_with_opts, CompileOptions};
use std::sync::Arc;

/// CUDA backend error.
#[derive(Debug, thiserror::Error)]
pub enum GpuError {
    #[error("CUDA 사용 불가: {0}")]
    /// Driver/NVRTC library or device is absent.
    NotAvailable(String),
    #[error("CUDA 드라이버 오류: {0:?}")]
    /// CUDA driver call error.
    Driver(#[from] cudarc::driver::DriverError),
    #[error("NVRTC 컴파일 실패: {0}")]
    /// NVRTC kernel compilation failure.
    Compile(String),
}

/// True if the driver and NVRTC libraries exist and there is at least one device.
pub fn is_available() -> bool {
    // Dynamic loading: cudarc panics if the library is missing, so check its presence first.
    // SAFETY: only tries to open the library.
    let libs = unsafe { cudarc::driver::sys::is_culib_present() && cudarc::nvrtc::sys::is_culib_present() };
    if !libs {
        return false;
    }
    std::panic::catch_unwind(|| CudaContext::device_count().map(|n| n > 0).unwrap_or(false)).unwrap_or(false)
}

/// Context and dedicated stream of one GPU.
pub struct CudaDevice {
    pub(crate) ctx: Arc<CudaContext>,
    pub(crate) stream: Arc<CudaStream>,
    arch: Option<&'static str>,
    name: String,
    /// Texture base address and row pitch alignment (bytes).
    pub(crate) tex_align: usize,
    pub(crate) pitch_align: usize,
}

impl CudaDevice {
    /// Opens device `ordinal`.
    pub fn new(ordinal: usize) -> Result<Arc<Self>, GpuError> {
        if !is_available() {
            return Err(GpuError::NotAvailable("드라이버/NVRTC 라이브러리 또는 장치 없음".into()));
        }
        let ctx =
            std::panic::catch_unwind(|| CudaContext::new(ordinal)).map_err(|_| GpuError::NotAvailable("문맥 생성 중 panic".into()))??;
        let stream = ctx.new_stream()?;
        let cc = ctx.compute_capability()?;
        let arch = match cc {
            (7, 0) => Some("compute_70"),
            (7, 2) => Some("compute_72"),
            (7, 5) => Some("compute_75"),
            (8, 0) => Some("compute_80"),
            (8, 6) => Some("compute_86"),
            (8, 7) => Some("compute_87"),
            (8, 9) => Some("compute_89"),
            (9, 0) => Some("compute_90"),
            _ => None,
        };
        let name = ctx.name().unwrap_or_default();
        use cudarc::driver::sys::CUdevice_attribute as A;
        let tex_align = ctx.attribute(A::CU_DEVICE_ATTRIBUTE_TEXTURE_ALIGNMENT).unwrap_or(512).max(4) as usize;
        let pitch_align = ctx.attribute(A::CU_DEVICE_ATTRIBUTE_TEXTURE_PITCH_ALIGNMENT).unwrap_or(32).max(4) as usize;
        Ok(Arc::new(Self { ctx, stream, arch, name, tex_align, pitch_align }))
    }

    /// Device name (e.g. "Tesla V100-PCIE-16GB").
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Compiles CUDA C source with NVRTC and loads it. With `fmad = false`, fused multiply-add is prevented so rounding matches the CPU.
    pub(crate) fn compile(&self, src: &str, fmad: bool, defines: &[String]) -> Result<Arc<CudaModule>, GpuError> {
        self.compile_with(src, fmad, false, defines)
    }

    /// With `fast = true`: approximate division/square root and denormal flush to 0 (for speed-first kernels).
    pub(crate) fn compile_with(&self, src: &str, fmad: bool, fast: bool, defines: &[String]) -> Result<Arc<CudaModule>, GpuError> {
        let opts = compile_options(self.arch, fmad, fast, defines);
        let ptx = compile_ptx_with_opts(src, opts).map_err(|e| GpuError::Compile(format!("{e:?}")))?;
        Ok(self.ctx.load_module(ptx)?)
    }
}

/// NVRTC options (device compilation and [`crate::check_kernels`] use the same values).
pub(crate) fn compile_options(arch: Option<&'static str>, fmad: bool, fast: bool, defines: &[String]) -> CompileOptions {
    let mut options = vec!["--std=c++14".to_string()];
    options.extend(defines.iter().map(|d| format!("-D{d}")));
    CompileOptions { arch, fmad: Some(fmad), ftz: Some(fast), prec_div: Some(!fast), prec_sqrt: Some(!fast), options, ..Default::default() }
}
