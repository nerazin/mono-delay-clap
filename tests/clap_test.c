/* Loads the built plug-in like a host and checks what the core test cannot:
 * the wrapper around mono_delay.h. usage: clap_test "build/Mono Delay.clap"
 *  - at 44.1 kHz with Mix 100 % the output is the core's, 16 frames late, exactly
 *  - with Mix 0 % the output is the input, latency frames late, exactly (any rate)
 *  - at 48 kHz the echo comes at the same time in milliseconds
 *  - ping-pong sends a left-only input to the right first
 *  - with Tempo Sync off the time ignores the host's tempo
 *  - state round trip, a 0.1.0 state, a rejected bad state, parameter text */
#include <clap/clap.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include "../source/mono_delay.h"

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); puts(""); exit(1); } } while (0)
#define N (192 * 512)   /* whole process calls */

typedef struct { clap_input_events_t api; clap_event_param_value_t v[16]; uint32_t n; } Events;
static uint32_t ev_size(const clap_input_events_t *e) { return ((const Events *)e->ctx)->n; }
static const clap_event_header_t *ev_get(const clap_input_events_t *e, uint32_t i) { return &((const Events *)e->ctx)->v[i].header; }
static const void *host_ext(const clap_host_t *h, const char *id) { (void)h; (void)id; return NULL; }
static void host_cb(const clap_host_t *h) { (void)h; }
static const clap_host_t host = {CLAP_VERSION_INIT, NULL, "test", "", "", "1", host_ext, host_cb, host_cb, host_cb};
static const clap_plugin_factory_t *factory;

typedef struct { uint8_t data[256]; uint64_t size, pos; } Buffer;
static int64_t buf_write(const clap_ostream_t *s, const void *p, uint64_t n) { Buffer *b = s->ctx; if (b->size + n > sizeof b->data) return -1; memcpy(b->data + b->size, p, n); b->size += n; return (int64_t)n; }
static int64_t buf_read(const clap_istream_t *s, void *p, uint64_t n) { Buffer *b = s->ctx; if (n > b->size - b->pos) n = b->size - b->pos; memcpy(p, b->data + b->pos, n); b->pos += n; return (int64_t)n; }

/* A plug-in with these parameter values (time, feedback, base, width, ping-pong, mix, sync), active at rate. */
static const clap_plugin_t *make(const double v[7], double rate) {
   const clap_plugin_t *p = factory->create_plugin(factory, &host, "works.reconstruction.monodelay");
   CHECK(p && p->init(p), "create");
   const clap_plugin_params_t *params = p->get_extension(p, CLAP_EXT_PARAMS);
   Events ev = {{&ev, ev_size, ev_get}, {{{0}}}, 7};
   for (uint32_t i = 0; i < 7; ++i) {
      ev.v[i] = (clap_event_param_value_t){{sizeof ev.v[i], 0, CLAP_CORE_EVENT_SPACE_ID, CLAP_EVENT_PARAM_VALUE, 0}, i, NULL, -1, -1, -1, -1, v[i]};
   }
   ev.api.ctx = &ev;
   params->flush(p, &ev.api, NULL);
   CHECK(p->activate(p, rate, 1, 512) && p->start_processing(p), "activate");
   return p;
}
/* Runs N frames of in through p at bpm, 512 at a time. */
static void run(const clap_plugin_t *p, double bpm, double in[2][N], double out[2][N]) {
   clap_event_transport_t tr = {{sizeof tr, 0, CLAP_CORE_EVENT_SPACE_ID, CLAP_EVENT_TRANSPORT, 0}, CLAP_TRANSPORT_HAS_TEMPO, 0, 0, bpm, 0, 0, 0, 0, 0, 0, 0, 0, 0};
   Events ev = {{NULL, ev_size, ev_get}, {{{0}}}, 0};
   ev.api.ctx = &ev;
   for (uint32_t at = 0; at < N; at += 512) {
      double *i[2] = {in[0] + at, in[1] + at}, *o[2] = {out[0] + at, out[1] + at};
      clap_audio_buffer_t ai = {NULL, i, 2, 0, 0}, ao = {NULL, o, 2, 0, 0};
      clap_process_t pr = {0};
      pr.frames_count = 512; pr.transport = &tr; pr.audio_inputs = &ai; pr.audio_outputs = &ao;
      pr.audio_inputs_count = pr.audio_outputs_count = 1; pr.in_events = &ev.api;
      CHECK(p->process(p, &pr) == CLAP_PROCESS_CONTINUE, "process");
   }
}
static uint32_t latency_of(const clap_plugin_t *p) { return ((const clap_plugin_latency_t *)p->get_extension(p, CLAP_EXT_LATENCY))->get(p); }
static void done(const clap_plugin_t *p) { p->stop_processing(p); p->deactivate(p); p->destroy(p); }

static double in[2][N], out[2][N];
int main(int argc, char **argv) {
   CHECK(argc == 2, "usage: clap_test plugin.clap");
   char path[1024];
   snprintf(path, sizeof path, "%s/Contents/MacOS/Mono Delay", argv[1]);
   void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
   CHECK(lib, "%s", dlerror());
   const clap_plugin_entry_t *entry = dlsym(lib, "clap_entry");
   CHECK(entry && entry->init(path), "entry");
   factory = entry->get_factory(CLAP_PLUGIN_FACTORY_ID);
   uint32_t seed = 7;
   for (int c = 0; c < 2; ++c)
      for (int n = 0; n < N; ++n) { seed = seed * 1664525u + 1013904223u; in[c][n] = (double)((int32_t)seed >> 10) / 8388608.0; }

   /* 44.1 kHz, Mix 100 %: the core itself */
   for (int ping = 0; ping < 2; ++ping) {
      const double v[7] = {15, 100, 10, 100, ping, 100, 1};
      const clap_plugin_t *p = make(v, 44100);
      CHECK(latency_of(p) == 16, "latency %u", latency_of(p));
      run(p, 133, in, out);
      done(p);
      static MonoDelay d;
      md_reset(&d);
      const int32_t ctl[MD_CONTROLS] = {15 << 16, 100 << 16, 10 << 16, 100 << 16, ping ? 0x7f0000 : 0};
      const uint32_t tick = 24 * 133;
      md_settle(&d, ctl[MD_TIME], tick);
      for (int at = 0; at + 32 <= N; at += 16) {
         int32_t x[2][16], y[2][16];
         for (int c = 0; c < 2; ++c) for (int n = 0; n < 16; ++n) x[c][n] = (int32_t)lrint(in[c][at + n] * 8388608.0);
         md_block(&d, ctl, tick, (const int32_t (*)[16])x, y);
         for (int c = 0; c < 2; ++c) for (int n = 0; n < 16; ++n)
            CHECK(out[c][at + 16 + n] == y[c][n] / 8388608.0, "ping %d frame %d: %.9f, core %.9f", ping, at + 16 + n, out[c][at + 16 + n], y[c][n] / 8388608.0);
      }
   }
   puts("PASS 44.1 kHz output is the core's, bit for bit (normal and ping-pong)");

   /* Mix 0 %: the input, untouched */
   const double rates[3] = {44100, 48000, 96000};
   for (int r = 0; r < 3; ++r) {
      const double v[7] = {15, 100, 10, 100, 0, 0, 1};
      const clap_plugin_t *p = make(v, rates[r]);
      const uint32_t lat = latency_of(p);
      run(p, 120, in, out);
      done(p);
      for (int c = 0; c < 2; ++c) for (uint32_t n = 0; n + lat < N; ++n) CHECK(out[c][n + lat] == in[c][n], "dry at %.0f Hz frame %u", rates[r], n);
      printf("PASS dry path exact at %.0f Hz (latency %u)\n", rates[r], lat);
   }

   /* 48 kHz: the echo time in ms, ping-pong, and Tempo Sync off at 90 BPM */
   const struct { int ping, sync; double bpm; } cases[3] = {{0, 1, 120}, {1, 1, 120}, {0, 0, 90}};
   for (int t = 0; t < 3; ++t) {
      const int ping = cases[t].ping;
      memset(in, 0, sizeof in);
      in[0][1000] = 0.5;
      const double v[7] = {15, 0, 0, 127, ping, 100, cases[t].sync};
      const clap_plugin_t *p = make(v, 48000);
      const uint32_t lat = latency_of(p);
      run(p, cases[t].bpm, in, out);
      done(p);
      int peak = 0, side = 0;
      for (int c = 0; c < 2; ++c) for (int n = 2000; n < N; ++n) if (fabs(out[c][n]) > fabs(out[side][peak])) { peak = n; side = c; }
      md_acc time, rate;
      md_target(15 << 16, 24 * 120, &time, &rate);   /* 120 BPM: the host's here, or the free tempo */
      const double want = 1000 + lat + (time / 16777216.0 + 3) * 48000 / 44100;   /* +3: AA, high-pass and low-pass each trail a sample */
      CHECK(fabs(peak - want) <= 2 && side == ping, "echo at %d on %d, want %.1f on %d", peak, side, want, ping);
      printf("PASS 48 kHz echo at %.2f ms on the %s (host at %.0f BPM, sync %s)\n", (peak - 1000 - lat) / 48.0, side ? "right" : "left", cases[t].bpm, cases[t].sync ? "on" : "off");
   }

   /* state and text */
   {
      const double v[7] = {31, 64, 20, 90, 1, 30, 0};
      const clap_plugin_t *a = make(v, 44100), *b = make((const double[7]){0, 0, 0, 0, 0, 0, 0}, 44100);
      const clap_plugin_state_t *sa = a->get_extension(a, CLAP_EXT_STATE), *sb = b->get_extension(b, CLAP_EXT_STATE);
      Buffer buf = {{0}, 0, 0};
      clap_ostream_t os = {&buf, buf_write};
      clap_istream_t is = {&buf, buf_read};
      CHECK(sa->save(a, &os) && sb->load(b, &is), "state");
      const clap_plugin_params_t *pb = b->get_extension(b, CLAP_EXT_PARAMS);
      for (clap_id i = 0; i < 7; ++i) { double x; CHECK(pb->get_value(b, i, &x) && x == v[i], "param %u after load", i); }
      buf.size = 56; buf.data[7] = 1; buf.pos = 0;   /* the same six values as a 0.1.0 state: Tempo Sync comes back on */
      CHECK(sb->load(b, &is), "0.1.0 state");
      for (clap_id i = 0; i < 7; ++i) { double x; CHECK(pb->get_value(b, i, &x) && x == (i == 6 ? 1 : v[i]), "param %u after 0.1.0 load", i); }
      buf.data[0] = 'X'; buf.pos = 0;
      CHECK(!sb->load(b, &is), "a bad state must be refused");
      char text[64];
      double x;
      CHECK(pb->value_to_text(b, 0, 15, text, sizeof text) && !strcmp(text, "125.0 ms"), "time text %s", text);
      CHECK(pb->text_to_value(b, 0, "250 ms", &x) && x == 31, "time from text %f", x);
      done(a); done(b);
      printf("PASS state round trip; Time 15 shows \"%s\"\n", text);
   }
   entry->deinit();
   return 0;
}
