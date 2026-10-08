// 기술자 정수 내적 top-2. 스레드 하나가 질의 기술자 하나를 맡아 대상 기술자를 0번부터 차례로 훑는다.
// 순차 갱신(엄격히 큼)이라 CPU Top2::push 를 순서대로 부른 것과 같은 결과다.

typedef unsigned int u32;

#define TILE 128

extern "C" __global__ void top2_rows(const u32* __restrict__ q, int nq, const u32* __restrict__ t, int nt, u32* out) {
    __shared__ u32 tile[TILE * 32];
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    u32 a[32];
    if (i < nq) {
#pragma unroll
        for (int k = 0; k < 32; ++k) a[k] = q[(size_t)i * 32 + k];
    } else {
#pragma unroll
        for (int k = 0; k < 32; ++k) a[k] = 0;
    }
    u32 best = 0, idx = 0xffffffffu, second = 0;
    for (int c0 = 0; c0 < nt; c0 += TILE) {
        int cn = min(TILE, nt - c0);
        __syncthreads();
        for (int e = threadIdx.x; e < cn * 32; e += blockDim.x) tile[e] = t[(size_t)c0 * 32 + e];
        __syncthreads();
        for (int c = 0; c < cn; ++c) {
            const u32* b = tile + c * 32;
            u32 s = 0;
#pragma unroll
            for (int k = 0; k < 32; ++k) s = __dp4a(a[k], b[k], s);
            if (s > best) { second = best; best = s; idx = (u32)(c0 + c); }
            else if (s > second) { second = s; }
        }
    }
    if (i < nq) {
        out[3 * i] = best;
        out[3 * i + 1] = idx;
        out[3 * i + 2] = second;
    }
}
