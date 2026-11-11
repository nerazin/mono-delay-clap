/* Mono Delay: the Monomachine's track delay as a CLAP effect.
 *
 * The delay itself is mono_delay.h, a fixed-point port of OS 1.32B. This file
 * supplies what surrounds it on the hardware and in a DAW:
 *  - the main CPU's handling of a knob: every 384 frames the word moves a
 *    quarter of the way to the knob's value; modulation is added on top,
 *    unsmoothed, every 48 frames (as the Monomachine's LFOs are);
 *  - the tempo word from the host's transport (30..300 BPM, as the unit);
 *  - Mix instead of the send: the input goes into the delay at full level
 *    (DSND -64, or +63 with ping-pong), and Mix fades from dry to the delay;
 *  - sample-rate conversion when the host does not run at 44.1 kHz.
 * SPDX-License-Identifier: GPL-3.0-only */
#include <clap/clap.h>
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mono_delay.h"

/* The first four parameters are the core's controls, in its order. */
enum { TIME, FEEDBACK, BASE, WIDTH, PINGPONG, MIX, NPARAM };
static const struct { const char *name; double min, max, def; } info[NPARAM] = {
   {"Time", 0, 127, 64}, {"Feedback", 0, 127, 28}, {"Filter Base", 0, 127, 0},
   {"Filter Width", 0, 127, 127}, {"Ping-Pong", 0, 1, 0}, {"Mix", 0, 100, 50},
};

static const char *const features[] = {
   CLAP_PLUGIN_FEATURE_AUDIO_EFFECT, CLAP_PLUGIN_FEATURE_DELAY, CLAP_PLUGIN_FEATURE_STEREO, NULL
};
static const clap_plugin_descriptor_t descriptor = {
   .clap_version = CLAP_VERSION_INIT,
   .id = "works.reconstruction.monodelay",
   .name = "Mono Delay",
   .vendor = "Independent DSP",
   .url = "", .manual_url = "", .support_url = "",
   .version = "0.1.0",
   .description = "The Monomachine track delay, ported from its DSP code",
   .features = features,
};

#define NATIVE 44100.0
#define HISTORY 4096        /* resampler input ring, also the longest dry delay */
#define PHASES 128
#define QUEUE 8192

static double clamp(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }

/* ---- sample-rate conversion (windowed sinc, as in Mono Filter Distortion) -- */

typedef struct { double x[2]; } Frame;
typedef struct {
   float *kernel;
   Frame history[HISTORY];
   double ratio;
   int half, taps;
   int64_t last;
   uint64_t index;
} Resampler;

static bool rs_prepare(Resampler *r, double from, double to) {
   r->ratio = from / to;
   r->half = (int)ceil(40.0 * fmax(1.0, r->ratio));
   r->taps = 2 * r->half;
   if (r->taps + 2 >= HISTORY) return false;
   r->kernel = calloc((size_t)(PHASES + 1) * r->taps, sizeof(float));
   if (!r->kernel) return false;
   const double cutoff = 0.95 * fmin(1.0, to / from), pi = 3.14159265358979323846;
   for (int p = 0; p <= PHASES; ++p) {
      float *k = r->kernel + (size_t)p * r->taps;
      double sum = 0.0;
      for (int i = 0; i < r->taps; ++i) {
         const double d = (double)(i - r->half + 1) - (double)p / PHASES, x = d / r->half, u = pi * cutoff * d;
         const double window = fabs(x) <= 1.0 ? 0.42 + 0.5 * cos(pi * x) + 0.08 * cos(2.0 * pi * x) : 0.0;
         k[i] = (float)(cutoff * (fabs(u) < 1e-14 ? 1.0 : sin(u) / u) * window);
         sum += k[i];
      }
      for (int i = 0; i < r->taps; ++i) k[i] /= (float)sum;
   }
   return true;
}
static void rs_reset(Resampler *r) { memset(r->history, 0, sizeof r->history); r->last = -1; r->index = 0; }
static void rs_push(Resampler *r, Frame f) { r->history[(size_t)(++r->last) & (HISTORY - 1)] = f; }
static bool rs_ready(const Resampler *r) { return (int64_t)floor((double)r->index * r->ratio) + r->half <= r->last; }
static Frame rs_pop(Resampler *r) {
   const double pos = (double)(r->index++) * r->ratio, pp = (pos - floor(pos)) * PHASES;
   const int64_t base = (int64_t)floor(pos);
   const int phase = (int)fmin(PHASES - 1, (int)pp);
   const float *a = r->kernel + (size_t)phase * r->taps, *b = a + r->taps;
   Frame out = {{0, 0}};
   for (int i = 0; i < r->taps; ++i) {
      const int64_t at = base - r->half + 1 + i;
      if (at < 0) continue;
      const Frame *f = &r->history[(size_t)at & (HISTORY - 1)];
      const double w = a[i] + (b[i] - a[i]) * (pp - phase);
      out.x[0] += f->x[0] * w;
      out.x[1] += f->x[1] * w;
   }
   return out;
}

/* ---- plug-in --------------------------------------------------------------- */

typedef struct {
   clap_plugin_t plugin;
   const clap_host_t *host;
   _Atomic uint64_t value[NPARAM];    /* parameter values (double bits): host thread and audio thread */
   _Atomic uint64_t bpm;              /* last transport tempo, for the Time display */
   double mod[NPARAM];
   /* audio thread */
   bool active, direct, settled;
   uint32_t latency, warmup, blocks, tick, dry_pos;
   uint64_t frames, qread, qwrite;
   int pos;
   int32_t knob[MD_SEND], word[MD_CONTROLS], in[2][MD_FRAMES];
   double wet[2][MD_FRAMES], mix;
   MonoDelay core;
   Resampler up, down;
   Frame queue[QUEUE];
   double dry[2][HISTORY];
} Plugin;

static double bits_to(uint64_t u) { double x; memcpy(&x, &u, sizeof x); return x; }
static uint64_t to_bits(double x) { uint64_t u; memcpy(&u, &x, sizeof u); return u; }
static double value(const Plugin *s, int id) { return bits_to(atomic_load_explicit(&s->value[id], memory_order_relaxed)); }
static void set_value(Plugin *s, int id, double x) { atomic_store_explicit(&s->value[id], to_bits(x), memory_order_relaxed); }
static uint32_t tick_of(double bpm) { return (uint32_t)clamp(lround(24.0 * bpm), 720, 7200); }

/* Start of a control frame (three blocks): the words the main CPU sends. */
static void control_frame(Plugin *s) {
   const uint32_t frame = s->blocks / 3;
   for (int i = 0; i < MD_SEND; ++i) {
      const int32_t target = (int32_t)lrint(clamp(value(s, i), 0, 127) * 65536.0);
      if (!s->settled) s->knob[i] = target;
      else if ((frame & 7) == 1) s->knob[i] = (3 * s->knob[i] + target) >> 2;
      s->word[i] = s->mod[i] == 0 ? s->knob[i] : (int32_t)lrint(clamp(s->knob[i] + s->mod[i] * 65536.0, 0, 0x7effff));
   }
   s->word[MD_SEND] = value(s, PINGPONG) + s->mod[PINGPONG] >= 0.5 ? 0x7f0000 : 0;
   s->tick = tick_of(bits_to(atomic_load_explicit(&s->bpm, memory_order_relaxed)));
   if (!s->settled) md_settle(&s->core, s->word[MD_TIME], s->tick);   /* start on time, not gliding in */
   s->settled = true;
}

/* One frame at 44.1 kHz: in goes into the core's block, out is the delay's
 * output from the previous block (16 frames of latency). */
static void native_frame(Plugin *s, const double in[2], double out[2]) {
   if (s->pos == 0 && s->blocks++ % 3 == 0) control_frame(s);
   for (int c = 0; c < 2; ++c) {
      s->in[c][s->pos] = (int32_t)lrint(clamp(in[c], -1.0, 0.99999988) * 8388608.0);
      out[c] = s->wet[c][s->pos];
   }
   if (++s->pos < MD_FRAMES) return;
   int32_t wet[2][MD_FRAMES];
   md_block(&s->core, s->word, s->tick, (const int32_t (*)[MD_FRAMES])s->in, wet);
   for (int c = 0; c < 2; ++c)
      for (int i = 0; i < MD_FRAMES; ++i) s->wet[c][i] = wet[c][i] / 8388608.0;
   s->pos = 0;
}

/* The delay's output for one host frame, latency frames late. */
static void wet_frame(Plugin *s, const double in[2], double out[2]) {
   if (s->direct) { native_frame(s, in, out); return; }
   rs_push(&s->up, (Frame){{in[0], in[1]}});
   while (rs_ready(&s->up)) {
      const Frame f = rs_pop(&s->up);
      Frame w;
      native_frame(s, f.x, w.x);
      rs_push(&s->down, w);
      while (rs_ready(&s->down))
         if (s->qwrite - s->qread < QUEUE) s->queue[(s->qwrite++) & (QUEUE - 1)] = rs_pop(&s->down);
         else rs_pop(&s->down);
   }
   out[0] = out[1] = 0;
   if (s->frames >= s->warmup && s->qread != s->qwrite) {
      const Frame f = s->queue[(s->qread++) & (QUEUE - 1)];
      out[0] = f.x[0];
      out[1] = f.x[1];
   }
}

static void reset(Plugin *s) {
   md_reset(&s->core);
   s->settled = false;
   s->blocks = s->dry_pos = 0;
   s->pos = 0;
   s->frames = s->qread = s->qwrite = 0;
   memset(s->in, 0, sizeof s->in);
   memset(s->wet, 0, sizeof s->wet);
   memset(s->dry, 0, sizeof s->dry);
   rs_reset(&s->up);
   rs_reset(&s->down);
   s->mix = clamp(value(s, MIX) / 100.0, 0, 1);
}

static bool plug_init(const clap_plugin_t *p) { (void)p; return true; }
static void plug_destroy(const clap_plugin_t *p) {
   Plugin *s = p->plugin_data;
   free(s->up.kernel);
   free(s->down.kernel);
   free(s);
}
static bool plug_activate(const clap_plugin_t *p, double rate, uint32_t min, uint32_t max) {
   (void)min; (void)max;
   Plugin *s = p->plugin_data;
   if (s->active || !isfinite(rate) || rate < 8000 || rate > 768000) return false;
   s->direct = rate == NATIVE;
   uint32_t latency = MD_FRAMES, warmup = 0;
   if (!s->direct) {
      if (!rs_prepare(&s->up, rate, NATIVE) || !rs_prepare(&s->down, NATIVE, rate)) {
         free(s->up.kernel); free(s->down.kernel); s->up.kernel = s->down.kernel = NULL;
         return false;
      }
      warmup = (uint32_t)(s->up.half + ceil(s->down.half * rate / NATIVE) + 2);
      latency = warmup + (uint32_t)llround(MD_FRAMES * rate / NATIVE);
   }
   s->warmup = warmup;
   reset(s);
   s->active = true;
   if (latency != s->latency) {
      s->latency = latency;
      const clap_host_latency_t *hl = s->host->get_extension ? s->host->get_extension(s->host, CLAP_EXT_LATENCY) : NULL;
      if (hl && hl->changed) hl->changed(s->host);
   }
   return true;
}
static void plug_deactivate(const clap_plugin_t *p) {
   Plugin *s = p->plugin_data;
   free(s->up.kernel); free(s->down.kernel);
   s->up.kernel = s->down.kernel = NULL;
   s->active = false;
}
static bool plug_start(const clap_plugin_t *p) { (void)p; return true; }
static void plug_stop(const clap_plugin_t *p) { (void)p; }
static void plug_reset(const clap_plugin_t *p) { reset(p->plugin_data); }
static void plug_on_main(const clap_plugin_t *p) { (void)p; }

static void event(Plugin *s, const clap_event_header_t *h) {
   if (!h || h->space_id != CLAP_CORE_EVENT_SPACE_ID) return;
   if (h->type == CLAP_EVENT_PARAM_VALUE && h->size >= sizeof(clap_event_param_value_t)) {
      const clap_event_param_value_t *e = (const void *)h;
      if (e->param_id < NPARAM && e->note_id == -1 && e->port_index == -1 && e->channel == -1 && e->key == -1 && isfinite(e->value))
         set_value(s, (int)e->param_id, clamp(e->value, info[e->param_id].min, info[e->param_id].max));
   } else if (h->type == CLAP_EVENT_PARAM_MOD && h->size >= sizeof(clap_event_param_mod_t)) {
      const clap_event_param_mod_t *e = (const void *)h;
      if (e->param_id < NPARAM && e->note_id == -1 && e->port_index == -1 && e->channel == -1 && e->key == -1 && isfinite(e->amount))
         s->mod[e->param_id] = clamp(e->amount, -info[e->param_id].max, info[e->param_id].max);
   }
}

static clap_process_status plug_process(const clap_plugin_t *p, const clap_process_t *pr) {
   Plugin *s = p->plugin_data;
   if (!s->active || !pr || pr->audio_inputs_count < 1 || pr->audio_outputs_count < 1) return CLAP_PROCESS_ERROR;
   const clap_audio_buffer_t *in = &pr->audio_inputs[0];
   clap_audio_buffer_t *out = &pr->audio_outputs[0];
   if (in->channel_count < 2 || out->channel_count < 2 || ((!in->data32 || !out->data32) && (!in->data64 || !out->data64)))
      return CLAP_PROCESS_ERROR;
   if (pr->transport && (pr->transport->flags & CLAP_TRANSPORT_HAS_TEMPO) && isfinite(pr->transport->tempo))
      atomic_store_explicit(&s->bpm, to_bits(pr->transport->tempo), memory_order_relaxed);
   const clap_input_events_t *ev = pr->in_events;
   const uint32_t count = ev && ev->size && ev->get ? ev->size(ev) : 0;
   uint32_t next = 0;
   const uint32_t dry_len = s->latency;
   for (uint32_t n = 0; n < pr->frames_count; ++n) {
      for (; next < count; ++next) {
         const clap_event_header_t *h = ev->get(ev, next);
         if (h && h->time > n) break;
         event(s, h);
      }
      double x[2], wet[2];
      for (int c = 0; c < 2; ++c) {
         x[c] = in->data32 ? in->data32[c][n] : in->data64[c][n];
         if (!isfinite(x[c])) x[c] = 0;
      }
      wet_frame(s, x, wet);
      s->mix += (clamp((value(s, MIX) + s->mod[MIX]) / 100.0, 0, 1) - s->mix) * 0.01;
      for (int c = 0; c < 2; ++c) {
         const double dry = s->dry[c][s->dry_pos];
         s->dry[c][s->dry_pos] = x[c];
         const double y = dry * (1.0 - s->mix) + wet[c] * s->mix;   /* exact dry at 0, exact delay at 1 */
         if (out->data32) out->data32[c][n] = (float)y; else out->data64[c][n] = y;
      }
      s->dry_pos = (s->dry_pos + 1) % dry_len;
      ++s->frames;
   }
   out->constant_mask = 0;
   return CLAP_PROCESS_CONTINUE;
}

/* ---- extensions ------------------------------------------------------------ */

static uint32_t ports_count(const clap_plugin_t *p, bool input) { (void)p; (void)input; return 1; }
static bool ports_get(const clap_plugin_t *p, uint32_t i, bool input, clap_audio_port_info_t *port) {
   (void)p;
   if (i || !port) return false;
   memset(port, 0, sizeof *port);
   snprintf(port->name, sizeof port->name, "%s", input ? "Stereo In" : "Stereo Out");
   port->flags = CLAP_AUDIO_PORT_IS_MAIN | CLAP_AUDIO_PORT_SUPPORTS_64BITS | CLAP_AUDIO_PORT_REQUIRES_COMMON_SAMPLE_SIZE;
   port->channel_count = 2;
   port->port_type = CLAP_PORT_STEREO;
   port->in_place_pair = 0;
   return true;
}
static const clap_plugin_audio_ports_t ports = {ports_count, ports_get};

static uint32_t params_count(const clap_plugin_t *p) { (void)p; return NPARAM; }
static bool params_info(const clap_plugin_t *p, uint32_t i, clap_param_info_t *pi) {
   (void)p;
   if (i >= NPARAM || !pi) return false;
   memset(pi, 0, sizeof *pi);
   pi->id = i;
   pi->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE | CLAP_PARAM_REQUIRES_PROCESS |
               (i == PINGPONG ? CLAP_PARAM_IS_STEPPED : 0);
   snprintf(pi->name, sizeof pi->name, "%s", info[i].name);
   pi->min_value = info[i].min;
   pi->max_value = info[i].max;
   pi->default_value = info[i].def;
   return true;
}
static bool params_get(const clap_plugin_t *p, clap_id id, double *v) {
   if (id >= NPARAM || !v) return false;
   *v = value(p->plugin_data, (int)id);
   return true;
}
static double cutoff(int index) { return 10.0 * pow(2000.0, (index < 127 ? index : 127) / 128.0); }
static bool params_to_text(const clap_plugin_t *p, clap_id id, double v, char *out, uint32_t cap) {
   const Plugin *s = p->plugin_data;
   if (id >= NPARAM || !out || !cap || !isfinite(v)) return false;
   if (id == TIME) {   /* (DTIM + 1) / 256 of a whole note, and what the unit makes of it at this tempo */
      const int i = (int)lrint(clamp(v, 0, 127)), n = i + 1, g = n & -n;   /* n <= 128: g = gcd(n, 256) */
      md_acc time, rate;
      md_target(i << 16, tick_of(bits_to(atomic_load_explicit(&s->bpm, memory_order_relaxed))), &time, &rate);
      const double samples = md_hi(rate) ? (double)time / 16777216.0 : 16368.0 * 16777216.0 / (double)rate;
      snprintf(out, cap, "%d/%d (%.1f ms)", n / g, 256 / g, samples * 1000.0 / NATIVE);
   } else if (id == BASE) snprintf(out, cap, "%.1f (%.0f Hz)", v, cutoff((int)v));
   else if (id == WIDTH) snprintf(out, cap, "%.1f (%.0f Hz)", v, cutoff((int)(value(s, BASE) + v)));
   else if (id == PINGPONG) snprintf(out, cap, "%s", v >= 0.5 ? "On" : "Off");
   else if (id == MIX) snprintf(out, cap, "%.0f %%", v);
   else snprintf(out, cap, "%.1f", v);
   return true;
}
static bool params_from_text(const clap_plugin_t *p, clap_id id, const char *text, double *v) {
   (void)p;
   if (id >= NPARAM || !text || !v) return false;
   if (id == PINGPONG && (!strcmp(text, "On") || !strcmp(text, "Off"))) { *v = text[1] == 'n'; return true; }
   char *end = NULL;
   double x = strtod(text, &end);
   if (end == text || !isfinite(x)) return false;
   if (id == TIME && *end == '/') {   /* a note length: "1/16" */
      const double d = strtod(end + 1, NULL);
      if (!(d > 0)) return false;
      x = 256.0 * x / d - 1.0;
   }
   *v = clamp(x, info[id].min, info[id].max);
   return true;
}
static void params_flush(const clap_plugin_t *p, const clap_input_events_t *in, const clap_output_events_t *out) {
   (void)out;
   const uint32_t n = in && in->size && in->get ? in->size(in) : 0;
   for (uint32_t i = 0; i < n; ++i) event(p->plugin_data, in->get(in, i));
}
static const clap_plugin_params_t params = {params_count, params_info, params_get, params_to_text, params_from_text, params_flush};

static const uint8_t magic[8] = {'M', 'D', 'L', 'Y', 0, 0, 0, 1};
static bool state_save(const clap_plugin_t *p, const clap_ostream_t *st) {
   uint8_t buf[8 + 8 * NPARAM];
   memcpy(buf, magic, 8);
   for (int i = 0; i < NPARAM; ++i)
      for (int b = 0; b < 8; ++b) buf[8 + 8 * i + b] = (uint8_t)(to_bits(value(p->plugin_data, i)) >> (8 * b));
   for (uint64_t done = 0; done < sizeof buf;) {
      const int64_t n = st && st->write ? st->write(st, buf + done, sizeof buf - done) : -1;
      if (n <= 0) return false;
      done += (uint64_t)n;
   }
   return true;
}
static bool state_load(const clap_plugin_t *p, const clap_istream_t *st) {
   Plugin *s = p->plugin_data;
   uint8_t buf[8 + 8 * NPARAM];
   for (uint64_t done = 0; done < sizeof buf;) {
      const int64_t n = st && st->read ? st->read(st, buf + done, sizeof buf - done) : -1;
      if (n <= 0) return false;
      done += (uint64_t)n;
   }
   if (memcmp(buf, magic, 8)) return false;
   double v[NPARAM];
   for (int i = 0; i < NPARAM; ++i) {
      uint64_t u = 0;
      for (int b = 0; b < 8; ++b) u |= (uint64_t)buf[8 + 8 * i + b] << (8 * b);
      v[i] = bits_to(u);
      if (!isfinite(v[i]) || v[i] < info[i].min || v[i] > info[i].max) return false;
   }
   for (int i = 0; i < NPARAM; ++i) set_value(s, i, v[i]);
   const clap_host_params_t *hp = s->host->get_extension ? s->host->get_extension(s->host, CLAP_EXT_PARAMS) : NULL;
   if (hp && hp->rescan) hp->rescan(s->host, CLAP_PARAM_RESCAN_VALUES);
   return true;
}
static const clap_plugin_state_t state = {state_save, state_load};

static uint32_t remote_count(const clap_plugin_t *p) { (void)p; return 1; }
static bool remote_get(const clap_plugin_t *p, uint32_t i, clap_remote_controls_page_t *page) {
   (void)p;
   if (i || !page) return false;
   memset(page, 0, sizeof *page);
   snprintf(page->section_name, sizeof page->section_name, "Delay");
   snprintf(page->page_name, sizeof page->page_name, "Main");
   for (int k = 0; k < CLAP_REMOTE_CONTROLS_COUNT; ++k) page->param_ids[k] = k < NPARAM ? (clap_id)k : CLAP_INVALID_ID;
   return true;
}
static const clap_plugin_remote_controls_t remote = {remote_count, remote_get};
static uint32_t tail_get(const clap_plugin_t *p) { (void)p; return UINT32_MAX; }   /* feedback can ring forever */
static const clap_plugin_tail_t tail = {tail_get};
static uint32_t latency_get(const clap_plugin_t *p) { return ((const Plugin *)p->plugin_data)->latency; }
static const clap_plugin_latency_t latency = {latency_get};

static const void *plug_extension(const clap_plugin_t *p, const char *id) {
   (void)p;
   if (!id) return NULL;
   if (!strcmp(id, CLAP_EXT_AUDIO_PORTS)) return &ports;
   if (!strcmp(id, CLAP_EXT_PARAMS)) return &params;
   if (!strcmp(id, CLAP_EXT_STATE)) return &state;
   if (!strcmp(id, CLAP_EXT_REMOTE_CONTROLS) || !strcmp(id, CLAP_EXT_REMOTE_CONTROLS_COMPAT)) return &remote;
   if (!strcmp(id, CLAP_EXT_TAIL)) return &tail;
   if (!strcmp(id, CLAP_EXT_LATENCY)) return &latency;
   return NULL;
}

/* ---- factory and entry ----------------------------------------------------- */

static uint32_t factory_count(const clap_plugin_factory_t *f) { (void)f; return 1; }
static const clap_plugin_descriptor_t *factory_desc(const clap_plugin_factory_t *f, uint32_t i) { (void)f; return i ? NULL : &descriptor; }
static const clap_plugin_t *factory_create(const clap_plugin_factory_t *f, const clap_host_t *host, const char *id) {
   (void)f;
   if (!host || !id || strcmp(id, descriptor.id) || !clap_version_is_compatible(host->clap_version)) return NULL;
   Plugin *s = calloc(1, sizeof *s);
   if (!s) return NULL;
   s->host = host;
   s->latency = MD_FRAMES;
   for (int i = 0; i < NPARAM; ++i) atomic_init(&s->value[i], to_bits(info[i].def));
   atomic_init(&s->bpm, to_bits(120.0));
   s->plugin = (clap_plugin_t){
      .desc = &descriptor, .plugin_data = s, .init = plug_init, .destroy = plug_destroy,
      .activate = plug_activate, .deactivate = plug_deactivate, .start_processing = plug_start,
      .stop_processing = plug_stop, .reset = plug_reset, .process = plug_process,
      .get_extension = plug_extension, .on_main_thread = plug_on_main,
   };
   return &s->plugin;
}
static const clap_plugin_factory_t factory = {factory_count, factory_desc, factory_create};

static bool entry_init(const char *path) { (void)path; md_tables(); return true; }   /* read-only from here on */
static void entry_deinit(void) {}
static const void *entry_factory(const char *id) { return id && !strcmp(id, CLAP_PLUGIN_FACTORY_ID) ? &factory : NULL; }
CLAP_EXPORT const clap_plugin_entry_t clap_entry = {CLAP_VERSION_INIT, entry_init, entry_deinit, entry_factory};
