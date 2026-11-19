/*
 * Host-side unit tests for ws2812_decoder.h
 *
 * Build & run (from the repository root):
 *   c++ -std=c++17 -O2 -Wall -Wextra -o /tmp/ws2812_decoder_test test/decoder_test.cpp && /tmp/ws2812_decoder_test
 *
 * The tests synthesize WS2812 waveforms (as level/duration segments in ns), sample them at a given rate
 * with a random phase, pack the samples MSB-first into 32-bit words, and feed them to the decoder in
 * random chunk sizes. Recovered frames are compared byte-for-byte with the source data.
 */
#include "../ws2812_decoder.h"
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

struct Seg { uint8_t level; double ns; };

struct Timing {
  double t0h = 400, t0l = 850, t1h = 800, t1l = 450; // nominal WS2812B datasheet values
  double jitter = 0;                                // each high/low duration varies uniformly by +-jitter ns
};

static std::mt19937 rng(12345);

static double jit(double v, double j) {
  if (j <= 0) return v;
  std::uniform_real_distribution<double> d(-j, j);
  return v + d(rng);
}

static void appendFrame(std::vector<Seg>& segs, const std::vector<uint8_t>& bytes, const Timing& t) {
  for (uint8_t b : bytes) {
    for (int i = 7; i >= 0; i--) {
      bool one = (b >> i) & 1;
      segs.push_back({1, jit(one ? t.t1h : t.t0h, t.jitter)});
      segs.push_back({0, jit(one ? t.t1l : t.t0l, t.jitter)});
    }
  }
}

static void appendLow(std::vector<Seg>& segs, double ns) { segs.push_back({0, ns}); }

// sample the waveform at `rateHz`, starting at a random phase within the first sample period
static std::vector<uint32_t> sample(const std::vector<Seg>& segs, double rateHz) {
  const double period = 1e9 / rateHz;
  std::uniform_real_distribution<double> ph(0, period);
  double t = ph(rng);
  std::vector<uint32_t> words;
  uint32_t acc = 0; int n = 0;
  double segEnd = 0;
  for (const Seg& s : segs) {
    segEnd += s.ns;
    while (t < segEnd) {
      acc = (acc << 1) | s.level;
      if (++n == 32) { words.push_back(acc); acc = 0; n = 0; }
      t += period;
    }
  }
  while (n) { acc <<= 1; if (++n == 32) { words.push_back(acc); n = 0; } } // pad with low
  return words;
}

struct Capture {
  std::vector<std::vector<uint8_t>> frames;
  uint8_t bufA[8192], bufB[8192];
};

static uint8_t* onFrame(void* ctx, uint8_t* data, size_t len) {
  Capture* c = static_cast<Capture*>(ctx);
  c->frames.emplace_back(data, data + len);
  return data == c->bufA ? c->bufB : c->bufA; // exercise buffer swapping
}

static void feed(Ws2812Decoder& dec, const std::vector<uint32_t>& words, size_t maxChunk) {
  std::uniform_int_distribution<size_t> d(1, maxChunk);
  size_t i = 0;
  while (i < words.size()) {
    size_t n = std::min(d(rng), words.size() - i);
    dec.decode(&words[i], n);
    i += n;
  }
}

static Ws2812Decoder::Config makeCfg(double rateHz, double thresholdNs = 600, double gapUs = 50) {
  Ws2812Decoder::Config c;
  c.highThreshold = (uint16_t)(thresholdNs * rateHz / 1e9 + 0.5);
  c.maxHigh = (uint16_t)(2000 * rateHz / 1e9 + 0.5);
  c.minHigh = 1;
  c.gapSamples = (uint32_t)(gapUs * rateHz / 1e6 + 0.5);
  return c;
}

static std::vector<uint8_t> randomBytes(size_t n) {
  std::uniform_int_distribution<int> d(0, 255);
  std::vector<uint8_t> v(n);
  for (auto& b : v) b = (uint8_t)d(rng);
  return v;
}

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void setup(Ws2812Decoder& dec, Capture& cap, double rate, size_t cap_bytes = sizeof(Capture::bufA)) {
  dec.setConfig(makeCfg(rate));
  dec.setOutput(cap.bufA, cap_bytes, onFrame, &cap);
  dec.reset();
}

// N frames of random data, compare byte-for-byte; returns number of mismatching frames
static int runFrames(double rate, const Timing& t, int nFrames, size_t pixels, size_t maxChunk, bool quiet = false) {
  Ws2812Decoder dec; Capture cap;
  setup(dec, cap, rate);
  std::vector<Seg> segs;
  std::vector<std::vector<uint8_t>> sent;
  std::uniform_real_distribution<double> gapD(55e3, 300e3);
  appendLow(segs, 100e3); // idle line before the first frame
  for (int f = 0; f < nFrames; f++) {
    sent.push_back(randomBytes(pixels * 3));
    appendFrame(segs, sent.back(), t);
    appendLow(segs, gapD(rng));
  }
  feed(dec, sample(segs, rate), maxChunk);
  int bad = 0;
  if (cap.frames.size() != sent.size()) {
    if (!quiet) printf("  got %zu frames, expected %zu (dropped %u, malformed %u)\n", cap.frames.size(), sent.size(), dec.getStats().droppedFrames, dec.getStats().malformedPulses);
    return nFrames;
  }
  for (size_t i = 0; i < sent.size(); i++) if (cap.frames[i] != sent[i]) bad++;
  return bad;
}

int main() {
  printf("clean frame, 316 px @ 8 MHz\n");
  CHECK(runFrames(8e6, Timing{}, 1, 316, 1000) == 0, "clean frame mismatch");

  printf("50 frames, 316 px @ 8 MHz, +-50 ns jitter, random gaps, chunks of 1..64 words\n");
  { Timing t; t.jitter = 50; CHECK(runFrames(8e6, t, 50, 316, 64) == 0, "jitter 50ns @8MHz"); }

  printf("50 frames @ 8 MHz, +-150 ns jitter, WS2812B-V5 style timing (T0H 300, T1H 900)\n");
  { Timing t; t.t0h = 300; t.t0l = 900; t.t1h = 900; t.t1l = 300; t.jitter = 150;
    CHECK(runFrames(8e6, t, 50, 316, 64) == 0, "jitter 150ns @8MHz, V5 timing"); }

  printf("50 frames @ 16 MHz, +-150 ns jitter, datasheet timing (T0H 400, T1H 800)\n");
  { Timing t; t.jitter = 150; CHECK(runFrames(16e6, t, 50, 316, 64) == 0, "jitter 150ns @16MHz"); }

  printf("single-word chunks @ 8 MHz\n");
  CHECK(runFrames(8e6, Timing{}, 5, 50, 1) == 0, "single-word chunks");

  printf("varying pixel counts\n");
  for (size_t px : {1, 2, 3, 10, 100, 316, 500}) CHECK(runFrames(8e6, Timing{}, 3, px, 128) == 0, "pixel count %zu", px);

  printf("capture starting mid-frame: partial first frame must be discarded\n");
  {
    Ws2812Decoder dec; Capture cap; setup(dec, cap, 8e6);
    std::vector<Seg> segs;
    auto first = randomBytes(300), second = randomBytes(300);
    appendFrame(segs, first, Timing{});
    appendLow(segs, 80e3);
    appendFrame(segs, second, Timing{});
    appendLow(segs, 80e3);
    auto words = sample(segs, 8e6);
    std::vector<uint32_t> tail(words.begin() + 300, words.end()); // start ~1.2 ms into the 3 ms first frame
    feed(dec, tail, 50);
    CHECK(cap.frames.size() == 1 && cap.frames[0] == second, "mid-frame start: got %zu frames", cap.frames.size());
  }

  printf("garbage between frames: malformed pulses drop that frame, next frame recovers\n");
  {
    Ws2812Decoder dec; Capture cap; setup(dec, cap, 8e6);
    std::vector<Seg> segs;
    auto a = randomBytes(90), b = randomBytes(90);
    appendLow(segs, 60e3);
    appendFrame(segs, a, Timing{});
    appendLow(segs, 60e3);
    // noise burst: random pulses including ones far too wide to be WS2812 bits
    std::uniform_real_distribution<double> d(50, 5000);
    for (int i = 0; i < 200; i++) { segs.push_back({1, d(rng)}); segs.push_back({0, d(rng)}); }
    appendLow(segs, 60e3);
    appendFrame(segs, b, Timing{});
    appendLow(segs, 60e3);
    feed(dec, sample(segs, 8e6), 33);
    CHECK(cap.frames.size() == 2 && cap.frames[0] == a && cap.frames[1] == b, "garbage: got %zu frames", cap.frames.size());
    CHECK(dec.getStats().droppedFrames >= 1, "garbage frame should be counted as dropped");
  }

  printf("truncated frame (ends mid-byte) is dropped and counted\n");
  {
    Ws2812Decoder dec; Capture cap; setup(dec, cap, 8e6);
    std::vector<Seg> segs;
    appendLow(segs, 60e3);
    auto a = randomBytes(30);
    appendFrame(segs, a, Timing{});
    segs.resize(segs.size() - 6); // remove the last 3 bits
    appendLow(segs, 60e3);
    feed(dec, sample(segs, 8e6), 16);
    CHECK(cap.frames.empty(), "truncated frame delivered");
    CHECK(dec.getStats().partialBytes == 1, "partialBytes=%u", dec.getStats().partialBytes);
  }

  printf("partial pixel (byte count not a multiple of 3) is delivered and counted\n");
  {
    Ws2812Decoder dec; Capture cap; setup(dec, cap, 8e6);
    std::vector<Seg> segs;
    appendLow(segs, 60e3);
    auto a = randomBytes(10);
    appendFrame(segs, a, Timing{});
    appendLow(segs, 60e3);
    feed(dec, sample(segs, 8e6), 16);
    CHECK(cap.frames.size() == 1 && cap.frames[0] == a, "partial pixel frame");
    CHECK(dec.getStats().partialPixels == 1, "partialPixels=%u", dec.getStats().partialPixels);
  }

  printf("oversize frame is truncated to buffer capacity\n");
  {
    Ws2812Decoder dec; Capture cap; setup(dec, cap, 8e6, 30);
    std::vector<Seg> segs;
    appendLow(segs, 60e3);
    auto a = randomBytes(60);
    appendFrame(segs, a, Timing{});
    appendLow(segs, 60e3);
    feed(dec, sample(segs, 8e6), 16);
    CHECK(cap.frames.size() == 1 && cap.frames[0].size() == 30 && std::equal(a.begin(), a.begin() + 30, cap.frames[0].begin()), "oversize");
    CHECK(dec.getStats().oversizeFrames == 1, "oversizeFrames");
  }

  printf("gap just below threshold does not split a frame; gap just above does\n");
  {
    Ws2812Decoder dec; Capture cap; setup(dec, cap, 8e6);
    std::vector<Seg> segs;
    appendLow(segs, 60e3);
    auto a = randomBytes(30), b = randomBytes(30);
    appendFrame(segs, a, Timing{});
    appendLow(segs, 40e3);   // 40 us + T0L/T1L < 50 us: same frame
    appendFrame(segs, b, Timing{});
    appendLow(segs, 60e3);
    feed(dec, sample(segs, 8e6), 16);
    std::vector<uint8_t> ab(a); ab.insert(ab.end(), b.begin(), b.end());
    CHECK(cap.frames.size() == 1 && cap.frames[0] == ab, "short gap split frame (%zu frames)", cap.frames.size());
  }

  printf("pulse histogram matches nominal timing\n");
  {
    Ws2812Decoder dec; Capture cap; setup(dec, cap, 8e6);
    std::vector<Seg> segs;
    appendLow(segs, 60e3);
    appendFrame(segs, std::vector<uint8_t>(10, 0x00), Timing{}); // 80 zero bits, 400 ns = 3.2 samples
    appendFrame(segs, std::vector<uint8_t>(10, 0xFF), Timing{}); // 80 one bits, 800 ns = 6.4 samples
    appendLow(segs, 60e3);
    feed(dec, sample(segs, 8e6), 16);
    const auto& h = dec.getStats().pulseHist;
    CHECK(h[3] + h[4] == 80 && h[6] + h[7] == 80, "hist 3:%u 4:%u 6:%u 7:%u", h[3], h[4], h[6], h[7]);
  }

  // informational: how often does the datasheet worst case break at 8 MHz?
  {
    Timing t; t.jitter = 150;
    int bad = runFrames(8e6, t, 50, 316, 64, true);
    printf("info: datasheet timing +-150 ns @ 8 MHz: %d/50 frames not recovered exactly (expected > 0: 125 ns quantization)\n", bad);
  }

  printf(failures ? "\n%d FAILURE(S)\n" : "\nall tests passed\n", failures);
  return failures ? 1 : 0;
}
