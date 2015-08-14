/*
 * Simplex Perlin Noise
 *
 * Originally by Casey Duncan, from https://github.com/caseman/noise.
 * Repackaged as a plain C header by Micah Elizabeth Scott.
 *
 * Copyright (c) 2008 Casey Duncan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once
#include <math.h>
#include <float.h>

const float GRAD3[][3] = {
	{1,1,0},{-1,1,0},{1,-1,0},{-1,-1,0}, 
	{1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1}, 
	{0,1,1},{0,-1,1},{0,1,-1},{0,-1,-1},
	{1,0,-1},{-1,0,-1},{0,-1,1},{0,1,1}};

const float GRAD4[][4] = {
    {0,1,1,1}, {0,1,1,-1}, {0,1,-1,1}, {0,1,-1,-1},
    {0,-1,1,1}, {0,-1,1,-1}, {0,-1,-1,1}, {0,-1,-1,-1},
    {1,0,1,1}, {1,0,1,-1}, {1,0,-1,1}, {1,0,-1,-1},
    {-1,0,1,1}, {-1,0,1,-1}, {-1,0,-1,1}, {-1,0,-1,-1},
    {1,1,0,1}, {1,1,0,-1}, {1,-1,0,1}, {1,-1,0,-1},
    {-1,1,0,1}, {-1,1,0,-1}, {-1,-1,0,1}, {-1,-1,0,-1},
    {1,1,1,0}, {1,1,-1,0}, {1,-1,1,0}, {1,-1,-1,0},
    {-1,1,1,0}, {-1,1,-1,0}, {-1,-1,1,0}, {-1,-1,-1,0}};

// At the possible cost of unaligned access, we use char instead of
// int here to try to ensure that this table fits in L1 cache
const unsigned char PERM[] = {
  151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140,
  36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120,
  234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33,
  88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71,
  134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133,
  230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161,
  1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130,
  116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250,
  124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227,
  47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44,
  154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98,
  108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34,
  242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14,
  239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121,
  50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243,
  141, 128, 195, 78, 66, 215, 61, 156, 180, 151, 160, 137, 91, 90, 15, 131,
  13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37,
  240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252,
  219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125,
  136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158,
  231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245,
  40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132, 187,
  208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198,
  173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126,
  255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223,
  183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167,
  43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185,
  112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179,
  162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199,
  106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236,
  205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156,
  180};

const unsigned char SIMPLEX[][4] = {
    {0,1,2,3},{0,1,3,2},{0,0,0,0},{0,2,3,1},{0,0,0,0},{0,0,0,0},{0,0,0,0},
    {1,2,3,0},{0,2,1,3},{0,0,0,0},{0,3,1,2},{0,3,2,1},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{1,3,2,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{1,2,0,3},{0,0,0,0},{1,3,0,2},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{2,3,0,1},{2,3,1,0},{1,0,2,3},{1,0,3,2},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{2,0,3,1},{0,0,0,0},{2,1,3,0},{0,0,0,0},{0,0,0,0},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{2,0,1,3},
    {0,0,0,0},{0,0,0,0},{0,0,0,0},{3,0,1,2},{3,0,2,1},{0,0,0,0},{3,1,2,0}, 
    {2,1,0,3},{0,0,0,0},{0,0,0,0},{0,0,0,0},{3,1,0,2},{0,0,0,0},{3,2,0,1},
    {3,2,1,0}};

#define fastfloor(n) (int)(n) - (((n) < 0.0f) & ((n) != (int)(n)))

// Fast sine/cosine functions from
// http://devmaster.net/forums/topic/4648-fast-and-accurate-sinecosine/page__st__80
// Note the input to these functions is not radians
// instead x = [0, 2] for r = [0, 2*PI]

static inline float fast_sin(float x)
{
    // Convert the input value to a range of -1 to 1
    // x = x * (1.0f / PI);

    // Wrap around
    volatile float z = (x + 25165824.0f);
    x = x - (z - 25165824.0f);

    #if LOW_SINE_PRECISION
        return 4.0f * (x - x * fabsf(x));
    #else
    {
        float y = x - x * fabsf(x);
        const float Q = 3.1f;
        const float P = 3.6f;
        return y * (Q + P * fabsf(y));
    }
    #endif
}

static inline float fast_cos(float x)
{
    return fast_sin(x + 0.5f);
}

// 2D simplex skew factors
#define F2 0.3660254037844386f  // 0.5 * (sqrt(3.0) - 1.0)
#define G2 0.21132486540518713f // (3.0 - sqrt(3.0)) / 6.0

static inline float 
noise2(float x, float y) 
{
  int i1, j1, I, J, c;
  float s = (x + y) * F2;
  float i = floorf(x + s);
  float j = floorf(y + s);
  float t = (i + j) * G2;

  float xx[3], yy[3], f[3];
  float noise[3] = {0.0f, 0.0f, 0.0f};
  int g[3];

  xx[0] = x - (i - t);
  yy[0] = y - (j - t);

  i1 = xx[0] > yy[0];
  j1 = xx[0] <= yy[0];

  xx[2] = xx[0] + G2 * 2.0f - 1.0f;
  yy[2] = yy[0] + G2 * 2.0f - 1.0f;
  xx[1] = xx[0] - i1 + G2;
  yy[1] = yy[0] - j1 + G2;

  I = (int) i & 255;
  J = (int) j & 255;
  g[0] = PERM[I + PERM[J]] % 12;
  g[1] = PERM[I + i1 + PERM[J + j1]] % 12;
  g[2] = PERM[I + 1 + PERM[J + 1]] % 12;

  for (c = 0; c <= 2; c++)
    f[c] = 0.5f - xx[c]*xx[c] - yy[c]*yy[c];
  
  for (c = 0; c <= 2; c++)
    if (f[c] > 0)
      noise[c] = f[c]*f[c]*f[c]*f[c] * (GRAD3[g[c]][0]*xx[c] + GRAD3[g[c]][1]*yy[c]);
  
  return (noise[0] + noise[1] + noise[2]) * 70.0f;
}

#define dot3(v1, v2) ((v1)[0]*(v2)[0] + (v1)[1]*(v2)[1] + (v1)[2]*(v2)[2])

#define ASSIGN(a, v0, v1, v2) (a)[0] = v0; (a)[1] = v1; (a)[2] = v2;

#define F3 (1.0f / 3.0f)
#define G3 (1.0f / 6.0f)

static inline float 
noise3(float x, float y, float z) 
{
  int c, o1[3], o2[3], g[4], I, J, K;
  float f[4], noise[4];
  noise[0] = 0.0f;
  noise[1] = 0.0f;
  noise[2] = 0.0f;
  noise[3] = 0.0f;
  float s = (x + y + z) * F3;
  float i = floorf(x + s);
  float j = floorf(y + s);
  float k = floorf(z + s);
  float t = (i + j + k) * G3;

  float pos[4][3];

  pos[0][0] = x - (i - t);
  pos[0][1] = y - (j - t);
  pos[0][2] = z - (k - t);

  if (pos[0][0] >= pos[0][1]) {
    if (pos[0][1] >= pos[0][2]) {
      ASSIGN(o1, 1, 0, 0);
      ASSIGN(o2, 1, 1, 0);
    } else if (pos[0][0] >= pos[0][2]) {
      ASSIGN(o1, 1, 0, 0);
      ASSIGN(o2, 1, 0, 1);
    } else {
      ASSIGN(o1, 0, 0, 1);
      ASSIGN(o2, 1, 0, 1);
    }
  } else {
    if (pos[0][1] < pos[0][2]) {
      ASSIGN(o1, 0, 0, 1);
      ASSIGN(o2, 0, 1, 1);
    } else if (pos[0][0] < pos[0][2]) {
      ASSIGN(o1, 0, 1, 0);
      ASSIGN(o2, 0, 1, 1);
    } else {
      ASSIGN(o1, 0, 1, 0);
      ASSIGN(o2, 1, 1, 0);
    }
  }
  
  for (c = 0; c <= 2; c++) {
    pos[3][c] = pos[0][c] - 1.0f + 3.0f * G3;
    pos[2][c] = pos[0][c] - o2[c] + 2.0f * G3;
    pos[1][c] = pos[0][c] - o1[c] + G3;
  }

  I = (int) i & 255; 
  J = (int) j & 255; 
  K = (int) k & 255;
  g[0] = PERM[I + PERM[J + PERM[K]]] % 12;
  g[1] = PERM[I + o1[0] + PERM[J + o1[1] + PERM[o1[2] + K]]] % 12;
  g[2] = PERM[I + o2[0] + PERM[J + o2[1] + PERM[o2[2] + K]]] % 12;
  g[3] = PERM[I + 1 + PERM[J + 1 + PERM[K + 1]]] % 12; 

  for (c = 0; c <= 3; c++) {
    f[c] = 0.6f - pos[c][0]*pos[c][0] - pos[c][1]*pos[c][1] - pos[c][2]*pos[c][2];
  }
  
  for (c = 0; c <= 3; c++) {
    if (f[c] > 0) {
      noise[c] = f[c]*f[c]*f[c]*f[c] * dot3(pos[c], GRAD3[g[c]]);
    }
  }
  
  return (noise[0] + noise[1] + noise[2] + noise[3]) * 32.0f;
}

static inline float
fbm_noise3(float x, float y, float z, int octaves, float persistence, float lacunarity) {
    float freq = 1.0f;
    float amp = 1.0f;
    float max = 1.0f;
    float total = noise3(x, y, z);
    int i;

    for (i = 1; i < octaves; ++i) {
        freq *= lacunarity;
        amp *= persistence;
        max += amp;
        total += noise3(x * freq, y * freq, z * freq) * amp;
    }
    return total / max;
}

#define dot4(v1, x, y, z, w) ((v1)[0]*(x) + (v1)[1]*(y) + (v1)[2]*(z) + (v1)[3]*(w))

#define F4 0.30901699437494745f /* (sqrt(5.0) - 1.0) / 4.0 */
#define G4 0.1381966011250105f /* (5.0 - sqrt(5.0)) / 20.0 */

static inline float 
noise4(float x, float y, float z, float w) {
    float noise[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float s = (x + y + z + w) * F4;
    float i = floorf(x + s);
    float j = floorf(y + s);
    float k = floorf(z + s);
    float l = floorf(w + s);
    float t = (i + j + k + l) * G4;

    float x0 = x - (i - t);
    float y0 = y - (j - t);
    float z0 = z - (k - t);
    float w0 = w - (l - t);

    int c = (x0 > y0)*32 + (x0 > z0)*16 + (y0 > z0)*8 + (x0 > w0)*4 + (y0 > w0)*2 + (z0 > w0);
    int i1 = SIMPLEX[c][0]>=3;
    int j1 = SIMPLEX[c][1]>=3;
    int k1 = SIMPLEX[c][2]>=3;
    int l1 = SIMPLEX[c][3]>=3;
    int i2 = SIMPLEX[c][0]>=2;
    int j2 = SIMPLEX[c][1]>=2;
    int k2 = SIMPLEX[c][2]>=2;
    int l2 = SIMPLEX[c][3]>=2;
    int i3 = SIMPLEX[c][0]>=1;
    int j3 = SIMPLEX[c][1]>=1;
    int k3 = SIMPLEX[c][2]>=1;
    int l3 = SIMPLEX[c][3]>=1;

    float x1 = x0 - i1 + G4;
    float y1 = y0 - j1 + G4;
    float z1 = z0 - k1 + G4;
    float w1 = w0 - l1 + G4;
    float x2 = x0 - i2 + 2.0f*G4;
    float y2 = y0 - j2 + 2.0f*G4;
    float z2 = z0 - k2 + 2.0f*G4;
    float w2 = w0 - l2 + 2.0f*G4;
    float x3 = x0 - i3 + 3.0f*G4;
    float y3 = y0 - j3 + 3.0f*G4;
    float z3 = z0 - k3 + 3.0f*G4;
    float w3 = w0 - l3 + 3.0f*G4;
    float x4 = x0 - 1.0f + 4.0f*G4;
    float y4 = y0 - 1.0f + 4.0f*G4;
    float z4 = z0 - 1.0f + 4.0f*G4;
    float w4 = w0 - 1.0f + 4.0f*G4;

    int I = (int)i & 255;
    int J = (int)j & 255;
    int K = (int)k & 255;
    int L = (int)l & 255;
    int gi0 = PERM[I + PERM[J + PERM[K + PERM[L]]]] & 0x1f;
    int gi1 = PERM[I + i1 + PERM[J + j1 + PERM[K + k1 + PERM[L + l1]]]] & 0x1f; 
    int gi2 = PERM[I + i2 + PERM[J + j2 + PERM[K + k2 + PERM[L + l2]]]] & 0x1f; 
    int gi3 = PERM[I + i3 + PERM[J + j3 + PERM[K + k3 + PERM[L + l3]]]] & 0x1f; 
    int gi4 = PERM[I + 1 + PERM[J + 1 + PERM[K + 1 + PERM[L + 1]]]] & 0x1f;
    float t0, t1, t2, t3, t4;

    t0 = 0.6f - x0*x0 - y0*y0 - z0*z0 - w0*w0;
    if (t0 >= 0.0f) {
        t0 *= t0;
        noise[0] = t0 * t0 * dot4(GRAD4[gi0], x0, y0, z0, w0);
    }
    t1 = 0.6f - x1*x1 - y1*y1 - z1*z1 - w1*w1;
    if (t1 >= 0.0f) {
        t1 *= t1;
        noise[1] = t1 * t1 * dot4(GRAD4[gi1], x1, y1, z1, w1);
    }
    t2 = 0.6f - x2*x2 - y2*y2 - z2*z2 - w2*w2;
    if (t2 >= 0.0f) {
        t2 *= t2;
        noise[2] = t2 * t2 * dot4(GRAD4[gi2], x2, y2, z2, w2);
    }
    t3 = 0.6f - x3*x3 - y3*y3 - z3*z3 - w3*w3;
    if (t3 >= 0.0f) {
        t3 *= t3;
        noise[3] = t3 * t3 * dot4(GRAD4[gi3], x3, y3, z3, w3);
    }
    t4 = 0.6f - x4*x4 - y4*y4 - z4*z4 - w4*w4;
    if (t4 >= 0.0f) {
        t4 *= t4;
        noise[4] = t4 * t4 * dot4(GRAD4[gi4], x4, y4, z4, w4);
    }

    return 27.0 * (noise[0] + noise[1] + noise[2] + noise[3] + noise[4]);
}

static inline float
fbm_noise4(float x, float y, float z, float w, int octaves, float persistence, float lacunarity) {
    float freq = 1.0f;
    float amp = 1.0f;
    float max = 1.0f;
    float total = noise4(x, y, z, w);
    int i;

    for (i = 1; i < octaves; ++i) {
        freq *= lacunarity;
        amp *= persistence;
        max += amp;
        total += noise4(x * freq, y * freq, z * freq, w * freq) * amp;
    }
    return total / max;
}
