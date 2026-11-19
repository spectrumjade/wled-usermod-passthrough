#pragma once
/*
 * Hardware-independent WS2812 bitstream decoder.
 *
 * Input is a continuous stream of line samples packed into 32-bit words, earliest sample in the MSB.
 * The decoder measures the width of every high pulse (in samples): short = 0, long = 1.
 * Bits are packed MSB-first into bytes in wire order. A low period of at least `gapSamples`
 * ends a frame. Pixel/color interpretation is left to the caller.
 *
 * No Arduino/ESP-IDF dependencies so it can be unit-tested on the host (see test/).
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

class Ws2812Decoder {
  public:
    static constexpr unsigned HIST_BINS = 32;            // pulse width histogram bins (1 sample each), last bin collects everything wider
    static constexpr uint32_t RUN_CAP   = 0x40000000u;   // saturation for run-length counters (idle line)

    struct Config {
      uint16_t highThreshold = 5;    // high pulse >= this many samples decodes as "1"
      uint16_t minHigh       = 1;    // narrower pulses are malformed (glitches)
      uint16_t maxHigh       = 16;   // wider pulses are malformed
      uint32_t gapSamples    = 400;  // low run >= this ends a frame
    };

    struct Stats {
      uint32_t frames;           // frames delivered to the callback
      uint32_t droppedFrames;    // frames discarded because of decode errors
      uint32_t malformedPulses;  // in-frame high pulses outside [minHigh, maxHigh]
      uint32_t partialBytes;     // frames that ended with an incomplete byte (dropped)
      uint32_t partialPixels;    // delivered frames whose byte count is not a multiple of 3
      uint32_t oversizeFrames;   // frames larger than the buffer (delivered truncated)
      uint32_t pulseHist[HIST_BINS]; // histogram of all measured high pulse widths, in samples
    };

    // Called for every complete, error-free frame. `data` is the buffer the frame was decoded into.
    // Returns the buffer to decode the next frame into (same capacity), e.g. to implement double buffering.
    typedef uint8_t* (*FrameCallback)(void* ctx, uint8_t* data, size_t len);

    Ws2812Decoder() { memset(&_stats, 0, sizeof(_stats)); }

    void setConfig(const Config& cfg) { _cfg = cfg; }
    const Config& getConfig() const { return _cfg; }
    void setOutput(uint8_t* buf, size_t capacity, FrameCallback cb, void* ctx) { _buf = buf; _cap = capacity; _cb = cb; _ctx = ctx; }
    const Stats& getStats() const { return _stats; }
    void clearStats() { memset(&_stats, 0, sizeof(_stats)); }

    // Forget line state. The first frame after reset() is only accepted after a frame gap has been seen,
    // so a capture that starts in the middle of a frame does not produce a bogus partial frame.
    void reset() {
      _level = 0; _run = 0; _synced = false; _inFrame = false;
      resetFrame();
    }

    // Decode `count` sample words. Can be called with arbitrary chunk sizes; state carries over.
    void decode(const uint32_t* words, size_t count) {
      uint32_t level = _level;
      uint32_t run   = _run;
      const uint32_t gap = _cfg.gapSamples;
      for (size_t i = 0; i < count; i++) {
        const uint32_t w = words[i];
        // fast path: whole word at the current level (idle line or the inside of a long pulse)
        if (w == (0u - level)) {
          run += 32;
          if (run > RUN_CAP) run = RUN_CAP;
          if (!level && run >= gap) gapReached();
          continue;
        }
        // bit k (from MSB) of t is set if sample k differs from the sample before it
        uint32_t t = w ^ ((w >> 1) | (level << 31));
        uint32_t consumed = 0;
        while (t) {
          const uint32_t pos = __builtin_clz(t);
          run += pos - consumed;
          consumed = pos;
          if (level) highPulse(run);  // falling edge: high pulse of `run` samples ended
          else       risingEdge(run); // rising edge: low run of `run` samples ended
          level ^= 1;
          run = 0;
          t &= 0x7FFFFFFFu >> pos;    // clear the transition just handled (and everything before it)
        }
        run += 32 - consumed;
        if (run > RUN_CAP) run = RUN_CAP;
        if (!level && run >= gap) gapReached();
      }
      _level = level;
      _run = run;
    }

    bool inFrame() const { return _inFrame; }

  private:
    Config   _cfg;
    Stats    _stats;
    uint8_t* _buf = nullptr;
    size_t   _cap = 0;
    FrameCallback _cb = nullptr;
    void*    _ctx = nullptr;

    uint32_t _level = 0;      // current line level (0/1)
    uint32_t _run = 0;        // samples spent at the current level so far
    bool     _synced = false; // a frame gap has been seen since reset()
    bool     _inFrame = false;
    bool     _frameError = false;
    uint8_t  _acc = 0;        // bits of the byte being assembled
    uint8_t  _bitCount = 0;
    size_t   _len = 0;        // bytes in the current frame (may exceed _cap)

    void resetFrame() { _frameError = false; _acc = 0; _bitCount = 0; _len = 0; }

    // a low period long enough to end a frame
    inline void gapReached() {
      if (_inFrame) endFrame();
      _synced = true;
    }

    inline void risingEdge(uint32_t lowRun) {
      if (lowRun >= _cfg.gapSamples) gapReached();
      if (_synced && !_inFrame) { _inFrame = true; resetFrame(); }
    }

    inline void highPulse(uint32_t width) {
      _stats.pulseHist[width < HIST_BINS ? width : HIST_BINS - 1]++;
      if (!_inFrame) return;
      if (width < _cfg.minHigh || width > _cfg.maxHigh) {
        _stats.malformedPulses++;
        _frameError = true;
      }
      _acc = (uint8_t)((_acc << 1) | (width >= _cfg.highThreshold ? 1 : 0));
      if (++_bitCount == 8) {
        if (_len < _cap) _buf[_len] = _acc;
        _len++;
        _bitCount = 0;
      }
    }

    void endFrame() {
      _inFrame = false;
      if (_bitCount) {
        _stats.partialBytes++;
        _frameError = true;
      }
      if (_frameError) {
        _stats.droppedFrames++;
      } else if (_len > 0) {
        size_t len = _len;
        if (len > _cap) { _stats.oversizeFrames++; len = _cap; }
        if (_len % 3) _stats.partialPixels++;
        _stats.frames++;
        if (_cb && len) _buf = _cb(_ctx, _buf, len);
      }
      resetFrame();
    }
};
