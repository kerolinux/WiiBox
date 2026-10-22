// weights.h - 5 -> 8 -> 2 MLP (tanh / tanh)
//
// This default set is HAND-DESIGNED so the project works out of the box.
// Overwrite it with `python tools/train_export.py` to use trained weights.
//
// Inputs  : s[0..4] = normalized ray distance, sensor angles -45,-22.5,0,22.5,45 deg
//           (1.0 = nothing in range, 0.0 = touching). +angle = robot's LEFT.
// Outputs : [0] = left wheel command, [1] = right wheel command, in [-1, 1].
//
// Hidden units:
//   h0..h4  per-sensor proximity detectors  tanh(2.4 - 4 s_i)
//   h5      left-vs-right clearance          tanh(2 (s3+s4-s0-s1))
//   h6      front very close (spin/escape)   tanh(2 - 8 s2)
//   h7      boxed in (all sensors low)       tanh(4 - 2 sum(s))
#pragma once

namespace weights {

constexpr int N_IN  = 5;
constexpr int N_HID = 8;
constexpr int N_OUT = 2;

static const float W1[N_HID][N_IN] = {
    { -4.0f,  0.0f,  0.0f,  0.0f,  0.0f },
    {  0.0f, -4.0f,  0.0f,  0.0f,  0.0f },
    {  0.0f,  0.0f, -4.0f,  0.0f,  0.0f },
    {  0.0f,  0.0f,  0.0f, -4.0f,  0.0f },
    {  0.0f,  0.0f,  0.0f,  0.0f, -4.0f },
    { -2.0f, -2.0f,  0.0f,  2.0f,  2.0f },
    {  0.0f,  0.0f, -8.0f,  0.0f,  0.0f },
    { -2.0f, -2.0f, -2.0f, -2.0f, -2.0f },
};

static const float B1[N_HID] = { 2.4f, 2.4f, 2.4f, 2.4f, 2.4f, 0.0f, 2.0f, 4.0f };

static const float W2[N_OUT][N_HID] = {
    { -0.7f, -0.9f,  0.15f,  0.9f,  0.7f, -0.5f,  0.8f, -0.3f },  // left wheel
    {  0.7f,  0.9f, -1.05f, -0.9f, -0.7f,  0.5f, -0.8f, -0.3f },  // right wheel
};

static const float B2[N_OUT] = { 1.74f, -0.97f };

} // namespace weights
