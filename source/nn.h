// nn.h - header-only, zero-dependency, allocation-free MLP inference.
//
// Topology: IN -> H (tanh) -> OUT (tanh). All parameters are const arrays
// (see weights.h) so they live in .rodata and nothing is ever allocated.
//
// Layout (row-major, matches tools/train_export.py):
//   W1[H][IN], B1[H], W2[OUT][H], B2[OUT]
#pragma once
#include <cmath>

namespace nn {

// Cheap tanh is unnecessary on Broadway for 5->8->2, but keep a hook for it.
static inline float act(float x) { return tanhf(x); }

template <int IN, int H, int OUT>
static inline void forward(const float (&in)[IN],
                           const float (&W1)[H][IN], const float (&B1)[H],
                           const float (&W2)[OUT][H], const float (&B2)[OUT],
                           float (&out)[OUT])
{
    float hid[H];  // stack only

    for (int j = 0; j < H; ++j) {
        float acc = B1[j];
        for (int i = 0; i < IN; ++i) acc += W1[j][i] * in[i];
        hid[j] = act(acc);
    }
    for (int k = 0; k < OUT; ++k) {
        float acc = B2[k];
        for (int j = 0; j < H; ++j) acc += W2[k][j] * hid[j];
        out[k] = act(acc);
    }
}

} // namespace nn
