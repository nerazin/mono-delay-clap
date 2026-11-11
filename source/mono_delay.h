/* mono_delay.h - Monomachine track delay, fixed-point core.
 *
 * A native C port of the per-track delay of Monomachine OS 1.32B, written
 * from the DSP56300 code and checked sample for sample against that code
 * running in an emulator. It contains no firmware code or data: the tables
 * are generated from formulas at start-up.
 *
 * Signal path, per channel:
 *   delay line (16384 words) -> linear-interpolating read -> one-pole high-pass
 *   at DBAS -> one-pole low-pass at DBAS+DWID -> output (the "wet" signal)
 *   input -> one-pole anti-alias low-pass that follows the line's rate,
 *   + 2 x DFB x output -> written back into the line (interpolating write)
 *
 * Delay time is (DTIM + 1) / 64 beat. A time that does not fit the line
 * (more than 16368 samples) is reached by running the line at a lower rate
 * instead. Time changes glide at most 4 samples per 16-sample block (the
 * read runs at 0.75x or 1.25x, so the echoes bend in pitch), rate changes
 * move 10% of the way per block, and the two are sequenced so that the
 * time only moves while the rate is 1.
 *
 * DSND >= 64 swaps the two delay lines on the way out, which makes the
 * echoes alternate between left and right (ping-pong).
 *
 * The arithmetic follows the DSP: 24-bit words, 56-bit accumulators,
 * truncation, rounding and saturation where the original has them. Blocks
 * are 16 stereo frames at 44.1 kHz; controls are the 24-bit block words the
 * Monomachine's main CPU sends (knob value << 16), plus its tempo word.
 *
 * Plain C99, no dependencies beyond libm. SPDX-License-Identifier: GPL-3.0-only */
#ifndef MONO_DELAY_H
#define MONO_DELAY_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#define MD_FRAMES 16
#define MD_LINE 0x4000            /* words per delay line */
#define MD_MASK (MD_LINE - 1)
#define MD_ONE 0x7fffff

enum { MD_TIME, MD_FEEDBACK, MD_BASE, MD_WIDTH, MD_SEND, MD_CONTROLS };   /* DTIM DFB DBAS DWID DSND */

/* ---- DSP56300 arithmetic ------------------------------------------------- */

typedef int64_t md_acc;           /* 56-bit accumulator, sign-extended; also 24.24 long words */

static inline md_acc md_s56(md_acc v) { return (md_acc)((uint64_t)v << 8) >> 8; }
static inline md_acc md_w(int32_t x) { return (md_acc)x * 16777216; }
static inline md_acc md_mpy(int32_t x, int32_t y) { return (md_acc)x * y * 2; }
static inline md_acc md_mpysu(int32_t s, uint32_t u) { return (md_acc)s * (md_acc)(u & 0xffffff) * 2; }
static inline md_acc md_mpyuu(uint32_t a, uint32_t b) { return (md_acc)(a & 0xffffff) * (md_acc)(b & 0xffffff) * 2; }
static inline md_acc md_shl(md_acc a, int n) { return md_s56((md_acc)((uint64_t)a << n)); }
static inline int32_t md_hi(md_acc a) { return (int32_t)((uint32_t)(a >> 24) << 8) >> 8; }
static inline uint32_t md_lo(md_acc a) { return (uint32_t)a & 0xffffff; }
static inline md_acc md_long(int32_t hi, uint32_t lo) { return md_w(hi) | lo; }   /* 48-bit hi:lo */
static inline int32_t md_sat(md_acc a) {
   const md_acc v = a >> 24;
   return v > MD_ONE ? MD_ONE : v < -MD_ONE - 1 ? -MD_ONE - 1 : (int32_t)v;
}
static inline md_acc md_rnd(md_acc a) {   /* convergent rounding to 24 bits */
   a += 0x800000;
   if (!(a & 0xffffff)) a &= ~(md_acc)0x1000000;
   return md_s56(a & ~(md_acc)0xffffff);
}
static int32_t md_div(md_acc a, int32_t s) {   /* 24 DIV steps, carry clear: quotient bits */
   uint64_t d = (uint64_t)a & 0xffffffffffffffULL;
   const uint64_t sv = (uint64_t)md_w(s);
   unsigned carry = 0;
   for (int i = 0; i < 24; ++i) {
      const int add = (int)((d >> 55) & 1) != ((s >> 23) & 1);
      d = ((d << 1) | carry) & 0xffffffffffffffULL;
      d = ((add ? d + sv : d - sv) & 0xffffffff000000ULL) | (d & 0xffffff);
      carry = !((d >> 55) & 1);
   }
   return (int32_t)(d & 0xffffff);
}

/* ---- tables --------------------------------------------------------------- */

/* One-pole coefficients c (y += c (x - y)) that put the -3 dB point of two
 * cascaded poles at f: [0..128] 10 Hz..20 kHz exponential (DBAS and
 * DBAS+DWID use 0..127), [129..257] 20 kHz (j+1)/129, indexed by the line
 * rate (anti-alias). */
static int32_t md_coef[258], md_recip[128];
static int md_tables_ready;

static void md_tables(void) {
   if (md_tables_ready) return;
   const double m = sqrt(2.0) - 1.0;
   for (int k = 0; k < 258; ++k) {
      const double f = k < 129 ? 10.0 * pow(2000.0, k / 128.0) : 20000.0 * (k - 128) / 129.0;
      const double c = 1.0 - cos(2.0 * 3.14159265358979323846 * f / 44100.0);
      md_coef[k] = (int32_t)floor((sqrt(c * c + 2.0 * c * m) - c) / m * 8388608.0 + 0.5);
   }
   /* 1 / (DTIM + 1), for the line rate of long delays */
   for (int i = 0; i < 128; ++i) md_recip[i] = (int32_t)ceil(MD_ONE / (i + 1.0) - 0.5);
   /* the formulas are one LSB off the original in these entries */
   static const short fc[][2] = {{50,1},{70,1},{99,-1},{149,-1},{161,1},{162,-1},{199,-1},{219,-1},{248,1}};
   static const short fr[][2] = {{34,1},{91,-1},{93,1},{96,1},{104,1},{116,1}};
   for (unsigned i = 0; i < sizeof fc / sizeof fc[0]; ++i) md_coef[fc[i][0]] += fc[i][1];
   for (unsigned i = 0; i < sizeof fr / sizeof fr[0]; ++i) md_recip[fr[i][0]] += fr[i][1];
   md_tables_ready = 1;
}

/* DTIM table: (i + 1) / 256 as a fraction; the top ten entries are one LSB low. */
static inline int32_t md_span(int i) { return (i + 1) * 0x8000 - (i >= 118); }

/* ---- state ------------------------------------------------------------------ */

typedef struct {
   int32_t line[2][MD_LINE];       /* L, R */
   md_acc time, rate, wpos;        /* 24.24: read distance, line rate, write position */
   int32_t hp_y[2], hp_x[2], lp[2], aa[2], hist[2];
} MonoDelay;

/* The firmware's power-on state: an empty line, 8192 samples, rate 1. */
static void md_reset(MonoDelay *d) {
   md_tables();
   memset(d, 0, sizeof *d);
   d->time = md_w(0x2000);
   d->rate = md_w(1);
}

/* Target read distance and line rate for a DTIM word and tempo word
 * (24 x BPM, 720..7200 on the unit). */
static void md_target(int32_t dtim, uint32_t tick, md_acc *time, md_acc *rate) {
   const int i = (dtim + 0x8000) >> 16;
   const int32_t beat = md_sat(md_shl(md_mpy((int32_t)(0x800000u / tick), 0x791fd0), 5));
   *time = md_mpy(md_span(i), beat);
   *rate = md_w(1);
   if (*time >= md_w(0x4000)) {   /* too long for the line: slow the line down instead */
      const md_acc b = md_mpy((int32_t)tick, md_recip[i]);
      *rate = (md_mpysu(0x21d10, md_lo(b)) >> 24) + md_mpy(0x21d10, md_hi(b));
      *time = md_w(0x3ff0);
   }
}

/* Jump straight to the settled time and rate (not a hardware behaviour; for a
 * fresh plug-in instance, so it does not glide in from 8192 samples). */
static void md_settle(MonoDelay *d, int32_t dtim, uint32_t tick) { md_target(dtim, tick, &d->time, &d->rate); }

/* Linear interpolation, as the shared routine at P:$397: from X, 16 outputs
 * per channel at positions pos[k] + frac[k]; the second channel reads
 * X[n + 1 + pos], one word on from the first. */
static void md_interp(const int32_t *X, int n, const int pos[17], const int32_t frac[17], int32_t a[16], int32_t b[16]) {
   for (int k = 0; k < 16; ++k) {
      const int p = pos[k];
      const int32_t f = frac[k];
      a[k] = md_sat(md_rnd(md_w(X[p]) - md_mpy(X[p], f) + md_mpy(X[p + 1], f)));
      b[k] = md_sat(md_rnd(md_w(X[p + 1 + n]) - md_mpy(X[p + 1 + n], f) + md_mpy(X[p + 2 + n], f)));
   }
}

/* Positions start + k step for k = 0..16: integer parts, and the fractions,
 * which the DSP keeps as halves in 23 bits and steps on their own. */
static void md_positions(md_acc start, md_acc step, int pos[17], int32_t frac[17]) {
   uint32_t f = md_lo(start) >> 1;
   for (int k = 0; k < 17; ++k) {
      pos[k] = md_hi(start);
      frac[k] = (int32_t)f;
      start += step;
      f = (f + (md_lo(step) >> 1)) & 0x7fffff;
   }
}

/* One block. in/out: [0] left, [1] right, 24-bit words; out is the delay's
 * output alone (the hardware adds it to the track's own signal). */
static void md_block(MonoDelay *d, const int32_t ctl[MD_CONTROLS], uint32_t tick,
                     const int32_t in[2][MD_FRAMES], int32_t out[2][MD_FRAMES]) {
   /* -- time and rate ------------------------------------------------------ */
   md_acc target, target_rate;
   md_target(ctl[MD_TIME], tick, &target, &target_rate);
   const md_acc cur = d->time;
   md_acc next = cur;
   int32_t speed = 0x400000, glide = 0x400000;   /* read speed / 2: 1x, 0.75x, 1.25x */
   if (cur != target) {
      md_acc move = target - cur;
      if (cur > target) { if (move < md_w(-4)) move = md_w(-4); glide = 0x500000; }
      else { if (move > md_w(4)) move = md_w(4); glide = 0x300000; }
      next = cur + move;
   }
   if (md_shl(d->rate, 10) >= md_w(0x3ff)) { speed = glide; d->time = next; }   /* the time moves only at rate 1 */
   if (md_hi(cur) >= 0x3ff0) {   /* the rate moves only at full time */
      md_acc r = md_mpysu(0x0ccccd, md_lo(target_rate)) + md_mpysu(0x733333, md_lo(d->rate));
      r = (r >> 24) + md_mpy(0x0ccccd, md_hi(target_rate)) + md_mpy(0x733333, md_hi(d->rate));
      d->rate = md_long(md_hi(r), md_lo(r));
   }
   const md_acc rate = d->rate, wpos = d->wpos;
   const int read = (md_hi(wpos) - md_hi(d->time)) & MD_MASK, write = md_hi(wpos);
   const md_acc nw = wpos + md_shl(rate, 4);
   d->wpos = md_long(md_hi(nw) & MD_MASK, md_lo(nw));

   /* -- read ----------------------------------------------------------------
    * X mirrors the DSP's scratch area: the read takes 34 words of each line
    * (DSND >= 64: right line first), later the input for the write goes in
    * front: [hist L, 16 L, hist R, 16 R, 34 of the second line]. */
   int32_t X[68], ret[2][16], hp[2][16], lp[2][16], aa[2][16];
   int pos[17];
   int32_t frac[17];
   const int ping = ctl[MD_SEND] >= 0x400000;
   for (int k = 0; k < 34; ++k) {
      X[k] = d->line[ping][(read + k) & MD_MASK];
      X[34 + k] = d->line[!ping][(read + k) & MD_MASK];
   }
   md_acc step = (md_mpysu(speed, md_lo(rate)) >> 24) + md_mpy(speed, md_hi(rate));
   step = md_shl(step, 1);
   md_positions(md_lo(wpos), step, pos, frac);
   md_interp(X, 34, pos, frac, ret[0], ret[1]);

   /* -- high-pass at DBAS, low-pass at DBAS + DWID ---------------------------
    * Each pole's output trails its input by a sample (the DSP stores the
    * state before updating it); states keep 24 bits between blocks. */
   const int32_t c = md_coef[ctl[MD_BASE] >> 16];
   const int32_t g = md_sat(-(((md_w(c) >> 1) + md_w(-0x800000))));   /* (1 + pole) / 2 */
   const int32_t cl = md_coef[md_sat(md_w(ctl[MD_BASE]) + md_w(ctl[MD_WIDTH])) >> 16];   /* the sum saturates: 127 at most */
   const int32_t ca = md_coef[129 + (int)md_lo(rate >> 17)];   /* rate x 128 */
   for (int ch = 0; ch < 2; ++ch) {
      md_acc h = md_w(d->hp_y[ch]), l = md_w(d->lp[ch]), a = md_w(d->aa[ch]);
      int32_t xp = d->hp_x[ch];
      for (int n = 0; n < 16; ++n) {
         hp[ch][n] = md_sat(h);
         h += md_mpy(ret[ch][n], g) - md_mpy(xp, g) - md_mpy(c, hp[ch][n]);
         xp = ret[ch][n];
      }
      for (int n = 0; n < 16; ++n) {
         lp[ch][n] = md_sat(l);
         l += md_mpy(hp[ch][n], cl) - md_mpy(cl, lp[ch][n]);
         aa[ch][n] = md_sat(a);
         a += md_mpy(in[ch][n], ca) - md_mpy(ca, aa[ch][n]);
      }
      d->hp_y[ch] = md_sat(h); d->hp_x[ch] = xp; d->lp[ch] = md_sat(l); d->aa[ch] = md_sat(a);
   }

   /* -- feedback and send: line input = 2 DFB x output + (2 |DSND - 64|)^2 x input */
   md_acc s = md_w(ctl[MD_SEND]) - md_w(0x400000);
   const int32_t sx = md_sat(md_shl(s < 0 ? -s : s, 1));
   const int32_t send = md_sat(md_mpy(sx, sx)), fb = ctl[MD_FEEDBACK];
   for (int ch = 0; ch < 2; ++ch) {
      X[17 * ch] = d->hist[ch];
      for (int n = 0; n < 16; ++n) {
         out[ch][n] = lp[ch][n];
         X[17 * ch + 1 + n] = md_sat(md_shl(md_mpy(fb, lp[ch][n]), 1) + md_mpy(aa[ch][n], send));
      }
      d->hist[ch] = X[17 * ch + 16];
   }

   /* -- write: the input resampled to the line's rate ------------------------
    * The first word lands at the write position; words past this block's
    * share are overwritten by the next block. */
   const int32_t inv_q = md_div(md_w(0x010000), (int32_t)md_lo(rate >> 2));
   const md_acc inv = md_w(inv_q) >> 17;                    /* 1 / rate */
   md_acc a = (md_mpyuu(md_lo(inv), md_lo(wpos)) >> 24) + md_mpysu(md_hi(inv), md_lo(wpos));
   a >>= 1;
   const md_acc first = (a == 0 ? 0 : inv) - a;
   if (first >= md_w(16)) return;
   md_positions(first, inv, pos, frac);
   int n = 0;
   while (n < 16 && pos[n] <= 49) ++n;   /* past X: the DSP reads unrelated memory there, always overwritten */
   for (int k = n; k < 16; ++k) pos[k] = 0;
   int32_t wl[16], wr[16];
   md_interp(X, 16, pos, frac, wl, wr);
   for (int k = 0; k < n; ++k) {
      d->line[0][(write + k) & MD_MASK] = wl[k];
      d->line[1][(write + k) & MD_MASK] = wr[k];
   }
}

#endif
