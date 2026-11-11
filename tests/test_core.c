/* Regression test for mono_delay.h that needs no firmware.
 *
 * The expected hashes come from Monomachine OS 1.32B itself: the same
 * stimulus was fed into the firmware's delay section running in an emulator
 * (see the README). The stimulus moves every control, changes tempo, reaches
 * the slowed line of long delays, switches ping-pong and drives the feedback
 * into saturation. */
#include <stdio.h>
#include "../source/mono_delay.h"

static uint32_t seed = 1;
static uint32_t rnd(uint32_t n) { seed = seed * 1664525u + 1013904223u; return (seed >> 8) % n; }

/* Controls and input of one block. */
static void stimulus(int block, int32_t ctl[MD_CONTROLS], uint32_t *tick, int32_t in[2][MD_FRAMES]) {
   static int level = 8;
   const int phase = block % 6000;                  /* 4500 blocks on a long time, then free */
   if (block == 0) {
      ctl[MD_FEEDBACK] = 80 << 16; ctl[MD_BASE] = 0; ctl[MD_WIDTH] = 127 << 16; ctl[MD_SEND] = 0;
   }
   if (phase == 0) {                                /* the time glides out, then the line slows down */
      ctl[MD_TIME] = (int32_t)(64 + rnd(64)) << 16;
      *tick = 2880 - rnd(1440);
   } else if (block % 97 == 0) {                    /* knob-like moves, also between steps */
      const int k = (int)rnd(MD_CONTROLS);
      if (k != MD_TIME || phase >= 4500)
         ctl[k] = k == MD_SEND ? (rnd(2) ? 0x7f0000 : (int32_t)rnd(0x800000)) : (int32_t)rnd(0x7f0000);
      if (k == MD_TIME && phase >= 4500 && rnd(2)) ctl[k] = (int32_t)rnd(20) << 16;
      if (rnd(4) == 0) *tick = 720 + rnd(6481);     /* 30..300 BPM */
      level = 8 + (int)rnd(8);                      /* full scale down to -42 dB */
   }
   for (int c = 0; c < 2; ++c)
      for (int n = 0; n < MD_FRAMES; ++n) {
         seed = seed * 1664525u + 1013904223u;
         in[c][n] = block % 400 < 300 ? (int32_t)seed >> level : 0;
      }
}

static uint64_t hash = 1469598103934665603ULL;
static void mix(const int32_t *v, int n) {
   for (int i = 0; i < n; ++i)
      for (int b = 0; b < 4; ++b) hash = (hash ^ (((uint32_t)v[i] >> (8 * b)) & 0xff)) * 1099511628211ULL;
}

#define TEST_BLOCKS 12000

#ifndef TEST_CORE_NO_MAIN
int main(void) {
   static MonoDelay d;
   md_reset(&d);
   mix(md_coef, 258); mix(md_recip, 128);
   const uint64_t tables = hash;
   hash = 1469598103934665603ULL;
   int32_t ctl[MD_CONTROLS], in[2][MD_FRAMES], out[2][MD_FRAMES];
   uint32_t tick;
   for (int block = 0; block < TEST_BLOCKS; ++block) {
      stimulus(block, ctl, &tick, in);
      md_block(&d, ctl, tick, in, out);
      mix(out[0], MD_FRAMES); mix(out[1], MD_FRAMES);
   }
   const uint64_t want_tables = 0x0f058407acc05da2ULL, want_audio = 0xe5b66ddc42559155ULL;
   printf("tables %016llx audio %016llx\n", (unsigned long long)tables, (unsigned long long)hash);
   if (tables != want_tables || hash != want_audio) { puts("FAIL: output differs from the firmware"); return 1; }
   puts("PASS core tables and audio match the firmware");
   return 0;
}
#endif
