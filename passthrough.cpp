/*
 * Passthrough usermod
 *
 * Captures a WS2812-style data stream from an upstream controller on an input GPIO (I2S peripheral used as
 * a 1-bit sampler with continuous DMA), decodes it to pixel bytes and re-transmits them through WLED's
 * normal LED output using the realtime path (same as E1.31/DDP). Gamma and brightness are bypassed so the
 * output bytes match the input bytes. See README.md for wiring, settings and diagnostics.
 *
 * Supported on classic ESP32 with ESP-IDF >= 5 only; other targets compile to a stub that reports "unsupported".
 */
#include "wled.h"
#include "ws2812_decoder.h"

#if defined(CONFIG_IDF_TARGET_ESP32) && defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  #define PASSTHROUGH_SUPPORTED
  #include <atomic>
  #include <driver/i2s_std.h>
#endif

static const char _ptName[]  PROGMEM = "Passthrough";  // config section and Info panel label
static const char _ptState[] PROGMEM = "passthrough";  // JSON state/info key

#ifndef PASSTHROUGH_SUPPORTED

// Stub for unsupported targets: keeps builds working and tells the user why nothing happens.
class UsermodPassthrough : public Usermod {
  public:
    void setup() override {}
    void loop() override {}
    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      JsonArray arr = user.createNestedArray(FPSTR(_ptName));
      arr.add(F("unsupported (classic ESP32 with ESP-IDF 5 only)"));
    }
};

#else

#ifndef PASSTHROUGH_MAX_PIXELS
  #define PASSTHROUGH_MAX_PIXELS 1024   // largest input frame accepted (longer frames are truncated)
#endif
#ifndef PASSTHROUGH_DMA_BUFFERS
  #define PASSTHROUGH_DMA_BUFFERS 8     // DMA ring: 8 x 4088 bytes = 32 KB = 33 ms of samples at 8 MHz
#endif

static constexpr UBaseType_t TASK_PRIORITY = 12;                // above the AsyncTCP web server task (10), below lwIP (18) and Wi-Fi (23)
static constexpr BaseType_t  TASK_CORE     = 0;                 // Arduino loop() runs on core 1
static constexpr uint32_t DMA_FRAME_NUM    = 511;               // stereo 32-bit frames per DMA buffer (8 bytes each, max 4092 bytes)
static constexpr size_t   DMA_BUF_BYTES    = DMA_FRAME_NUM * 8;
static constexpr size_t   FRAME_BUF_BYTES  = PASSTHROUGH_MAX_PIXELS * 3;
static constexpr uint32_t MAX_HIGH_NS      = 2000;              // longer high pulses are malformed
static constexpr uint8_t  FRESH            = 0x04;              // triple buffer: "new frame ready" flag

// I2S ISR callback: the DMA receive queue overflowed, i.e. the oldest buffer was dropped.
// The I2S driver is built with CONFIG_I2S_ISR_IRAM_SAFE and rejects callbacks outside IRAM. This must be a
// non-inline free function: IRAM_ATTR on a member function defined inside the class body was dropped by LTO.
static bool IRAM_ATTR passthroughOnRecvOverflow(i2s_chan_handle_t, i2s_event_data_t*, void* ctx) {
  static_cast<std::atomic<uint32_t>*>(ctx)->fetch_add(1, std::memory_order_relaxed);
  return false;
}

// position of R, G, B within a wire triplet, indexed by WLED COL_ORDER_* value
static const uint8_t ORDER_MAP[COL_ORDER_MAX + 1][3] = {
  {1, 0, 2}, // GRB
  {0, 1, 2}, // RGB
  {1, 2, 0}, // BRG
  {0, 2, 1}, // RBG
  {2, 1, 0}, // BGR
  {2, 0, 1}, // GBR
};

class UsermodPassthrough : public Usermod {
  private:
    // ---- persistent settings ----
    bool     _enabled = false;          // passthrough active after boot (off until the user enables it)
    int8_t   _inputPin = 33;
    uint8_t  _i2sPort = 1;
    uint8_t  _sampleRateMHz = 8;        // 8 or 16
    uint16_t _thresholdNs = 600;
    uint16_t _gapUs = 50;
    uint8_t  _inputOrder = COL_ORDER_GRB;
    bool     _noSignalBlack = false;    // false = hold last frame, true = go black
    uint16_t _noSignalMs = 1000;
    bool     _applyBri = false;

    // ---- requests from JSON/config handlers (web server context), applied in loop() ----
    std::atomic<bool> _ptOn{false};
    std::atomic<bool> _cfgDirty{true};
    std::atomic<bool> _suspended{false}; // OTA in progress
    bool _cfgRead = false;

    // ---- runtime state (loop context) ----
    int8_t      _allocatedPin = -1;
    const char* _error = nullptr;
    char        _errBuf[64] = "";   // formatted error text (e.g. with an ESP-IDF error name)
    bool        _capturing = false;
    uint32_t    _startFailedAt = 0;
    bool        _ownRealtime = false;
    bool        _overridden = false;  // UI "override live" was active in the last loop()
    bool        _blanked = false;
    uint32_t    _lastFrameMs = 0;
    size_t      _lastPixels = 0;
    uint32_t    _lastConflictCheck = 0;

    // input frame rate (updated once per second)
    uint32_t _rateTime = 0, _rateIn = 0;
    uint16_t _inFps = 0;

    // ---- capture (shared with the capture task) ----
    i2s_chan_handle_t  _rx = nullptr;
    TaskHandle_t       _task = nullptr;
    volatile bool      _taskStop = false;
    volatile bool      _taskRunning = false;
    uint32_t*          _readBuf = nullptr;
    uint8_t*           _frameBuf[3] = {nullptr, nullptr, nullptr};
    size_t             _frameLen[3] = {0, 0, 0};
    std::atomic<uint8_t> _ready{1};    // triple buffer: index of latest complete frame | FRESH
    uint8_t            _backIdx = 0;   // capture task only
    uint8_t            _frontIdx = 2;  // loop() only
    Ws2812Decoder      _decoder;
    std::atomic<uint32_t> _overruns{0};
    std::atomic<bool>  _clearStatsReq{false}; // decoder stats belong to the capture task, so it clears them
    uint8_t            _capRateMHz = 8;     // _sampleRateMHz snapshot of the running capture


    // ------------------------------------------------------------------------------------------
    // capture task and I2S
    // ------------------------------------------------------------------------------------------

    // decoder callback (capture task): publish the finished frame and continue in a free buffer
    static uint8_t* onFrame(void* ctx, uint8_t* data, size_t len) {
      auto* self = static_cast<UsermodPassthrough*>(ctx);
      self->_frameLen[self->_backIdx] = len;
      uint8_t old = self->_ready.exchange(self->_backIdx | FRESH, std::memory_order_acq_rel);
      self->_backIdx = old & 3;
      return self->_frameBuf[self->_backIdx];
    }

    // loop(): take the latest complete frame, if there is a new one
    bool fetchFrame(const uint8_t*& data, size_t& len) {
      if (!(_ready.load(std::memory_order_acquire) & FRESH)) return false;
      uint8_t old = _ready.exchange(_frontIdx, std::memory_order_acq_rel);
      _frontIdx = old & 3;
      data = _frameBuf[_frontIdx];
      len  = _frameLen[_frontIdx];
      return true;
    }

    static void captureTask(void* arg) {
      static_cast<UsermodPassthrough*>(arg)->captureLoop();
      vTaskDelete(nullptr);
    }

    void captureLoop() {
      uint32_t seenOverruns = _overruns.load();
      while (!_taskStop) {
        size_t got = 0;
        esp_err_t err = i2s_channel_read(_rx, _readBuf, DMA_BUF_BYTES, &got, 100);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) { vTaskDelay(1); continue; }
        if (_clearStatsReq.exchange(false)) _decoder.clearStats();
        uint32_t ovf = _overruns.load(std::memory_order_relaxed);
        if (ovf != seenOverruns) {
          // samples were lost somewhere in the queued data: drain the queue and resync on the next frame gap,
          // otherwise two unrelated parts of the stream could be stitched into a wrong but well-formed frame
          seenOverruns = ovf;
          size_t n;
          while (i2s_channel_read(_rx, _readBuf, DMA_BUF_BYTES, &n, 0) == ESP_OK && n) {}
          _decoder.reset();
          continue;
        }
        _decoder.decode(_readBuf, got / 4);
      }
      _taskRunning = false;
    }

    // returns a description of a peripheral conflict, or nullptr
    const char* findConflict() const {
      if (_i2sPort == 0 && UsermodManager::lookup(USERMOD_ID_AUDIOREACTIVE)) return PSTR("I2S0 is used by AudioReactive");
      if (_i2sPort == 1) {
        for (const auto& bus : BusManager::busses) {
          if (bus->isDigital() && !bus->is2Pin() && bus->getDriverType() == BUSDRV_PARHW) return PSTR("I2S1 is used by the LED output (parallel I2S driver)");
          if (Bus::isHub75(bus->getType())) return PSTR("I2S1 is used by the HUB75 output");
        }
      }
      return nullptr;
    }

    // free capture buffers and release the input pin (the pin is only held while capturing)
    void releaseResources() {
      for (auto& b : _frameBuf) { free(b); b = nullptr; }
      free(_readBuf); _readBuf = nullptr;
      if (_allocatedPin >= 0) {
        PinManager::deallocatePin(_allocatedPin, PinOwner::UM_Unspecified);
        _allocatedPin = -1;
      }
    }

    bool startCapture() {
      if ((_error = checkPin(_inputPin))) return false;
      if ((_error = findConflict())) return false;
      if (!PinManager::allocatePin(_inputPin, false, PinOwner::UM_Unspecified)) { _error = PSTR("input pin already in use"); return false; }
      _allocatedPin = _inputPin;

      _readBuf = static_cast<uint32_t*>(d_malloc(DMA_BUF_BYTES));
      for (auto& b : _frameBuf) b = static_cast<uint8_t*>(d_malloc(FRAME_BUF_BYTES));
      if (!_readBuf || !_frameBuf[0] || !_frameBuf[1] || !_frameBuf[2]) { releaseResources(); _error = PSTR("out of memory"); return false; }

      i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)_i2sPort, I2S_ROLE_MASTER);
      chanCfg.dma_desc_num  = PASSTHROUGH_DMA_BUFFERS;
      chanCfg.dma_frame_num = DMA_FRAME_NUM;
      if (i2s_new_channel(&chanCfg, nullptr, &_rx) != ESP_OK) {
        _rx = nullptr; releaseResources();
        _error = PSTR("I2S port busy or out of memory");
        return false;
      }

      // 32-bit stereo slots: every bit clock cycle samples the input once, so the sample stream has no gaps.
      // BCLK = sample_rate * 64. MCLK = 2 * BCLK keeps the divider from the 160 MHz PLL an integer for 8 and 16 MHz.
      // No clock pins are routed: the master clock only exists internally.
      _capRateMHz = _sampleRateMHz;
      i2s_std_config_t stdCfg = {};
      stdCfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)_capRateMHz * 1000000U / 64U);
      stdCfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_128;
      stdCfg.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
      stdCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
      stdCfg.gpio_cfg.bclk = I2S_GPIO_UNUSED;
      stdCfg.gpio_cfg.ws   = I2S_GPIO_UNUSED;
      stdCfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
      stdCfg.gpio_cfg.din  = (gpio_num_t)_allocatedPin;

      i2s_event_callbacks_t cbs = {};
      cbs.on_recv_q_ovf = passthroughOnRecvOverflow;
      esp_err_t err = i2s_channel_init_std_mode(_rx, &stdCfg);
      if (err == ESP_OK) err = i2s_channel_register_event_callback(_rx, &cbs, &_overruns);
      if (err == ESP_OK) err = i2s_channel_enable(_rx);
      if (err != ESP_OK) {
        i2s_del_channel(_rx); _rx = nullptr; releaseResources();
        snprintf_P(_errBuf, sizeof(_errBuf), PSTR("I2S setup failed: %s"), esp_err_to_name(err));
        _error = _errBuf;
        DEBUG_PRINTF_P(PSTR("Passthrough: %s\n"), _error);
        return false;
      }

      // idle level is low; the pull-down keeps an unconnected input quiet (GPIO34-39 have none, the external divider does the job)
      if (_allocatedPin < 34) gpio_pulldown_en((gpio_num_t)_allocatedPin);

      Ws2812Decoder::Config dc;
      const uint32_t rateMHz = _capRateMHz;
      uint32_t thr = ((uint32_t)_thresholdNs * rateMHz + 500U) / 1000U;
      uint32_t maxHigh = (MAX_HIGH_NS * rateMHz) / 1000U;
      if (thr < 2) thr = 2;
      if (maxHigh <= thr) maxHigh = thr + 1;
      dc.highThreshold = (uint16_t)thr;
      dc.minHigh       = 1;
      dc.maxHigh       = (uint16_t)maxHigh;
      dc.gapSamples    = (uint32_t)_gapUs * rateMHz;
      _decoder.setConfig(dc);
      _decoder.clearStats();
      _decoder.reset();
      _backIdx = 0; _frontIdx = 2; _ready.store(1);
      _decoder.setOutput(_frameBuf[_backIdx], FRAME_BUF_BYTES, onFrame, this);

      _taskStop = false;
      _taskRunning = true;
      if (xTaskCreatePinnedToCore(captureTask, "passthrough", 4096, this, TASK_PRIORITY, &_task, TASK_CORE) != pdPASS) {
        _taskRunning = false; _task = nullptr;
        i2s_channel_disable(_rx); i2s_del_channel(_rx); _rx = nullptr; releaseResources();
        _error = PSTR("cannot create capture task");
        return false;
      }
      _capturing = true;
      _lastFrameMs = millis();
      _blanked = false;
      DEBUG_PRINTF_P(PSTR("Passthrough: capture started on I2S%u, GPIO%d, %u MHz\n"), _i2sPort, _allocatedPin, _capRateMHz);
      return true;
    }

    void stopCapture() {
      if (_task) {
        _taskStop = true;
        uint32_t t0 = millis();
        while (_taskRunning && millis() - t0 < 500) vTaskDelay(1); // read timeout is 100 ms
        if (_taskRunning) { vTaskDelete(_task); _taskRunning = false; } // should never happen
        _task = nullptr;
      }
      if (_rx) {
        i2s_channel_disable(_rx);
        i2s_del_channel(_rx);
        _rx = nullptr;
      }
      releaseResources();
      if (_capturing) DEBUG_PRINTLN(F("Passthrough: capture stopped"));
      _capturing = false;
    }

    // ------------------------------------------------------------------------------------------
    // output
    // ------------------------------------------------------------------------------------------

    // Keep WLED in realtime mode, (re)acquiring the lock if another source released or replaced it, and keep
    // effect rendering suspended: with "use main segment only" WLED would otherwise keep rendering segments and
    // overwrite the bus. Other code (JSON API, presets) may resume the strip briefly, so this runs every loop().
    void ensureRealtime() {
      if (!realtimeMode || realtimeTimeout != UINT32_MAX) {
        realtimeLock(65000, REALTIME_MODE_GENERIC); // 65000 = no timeout
        interfaceUpdateCallMode = CALL_MODE_WS_SEND; // realtimeLock() doesn't notify the UI ("live" state)
      }
      _ownRealtime = true;
      if (realtimeOverride) strip.resume(); // UI "override live": WLED's own effects are shown
      else                  strip.suspend();
      // keepPowered() and render() left strip and bus at full brightness; WLED's effects need its own brightness
      // back, and nothing in WLED re-applies it when override starts
      const bool overridden = realtimeOverride;
      if (overridden && !_overridden) restoreWledBrightness();
      _overridden = overridden;
    }

    // Hand strip and bus brightness back to WLED (after keepPowered()/render() set them to full)
    static void restoreWledBrightness() {
      applyBri(); // strip brightness = WLED's current brightness (applies during override or outside realtime mode)
      // render() wrote the bus brightness directly; if the strip value didn't change, setBrightness16() returned
      // early and left it at full, so set the value WLED expects (including the brightness multiplier)
      BusManager::setBrightness16(scaledBri16(strip.getBrightness16()));
      strip.trigger(); // repaint WLED's own content right away
    }

    // Keep strip brightness at full so the relay stays closed (handleOnOff() opens it at zero brightness).
    // Runs every loop() between handleTransitions() and handleIO().
    void keepPowered() {
      if (!realtimeOverride && strip.getBrightness() != 255) strip.setBrightness(255, true);
    }

    void releaseRealtime() {
      if (!_ownRealtime) return;
      _ownRealtime = false;
      strip.resume();
      _overridden = false;
      if (realtimeMode == REALTIME_MODE_GENERIC) exitRealtime();
      restoreWledBrightness();
      // schedule a WebSocket update so the UI drops its "live data" overlay: the JSON response to the toggle request
      // was sent before this loop() ran, and exitRealtime() only sends an update if one is already pending
      interfaceUpdateCallMode = CALL_MODE_WS_SEND;
    }

    // Write wire-order pixel bytes straight to the LED outputs and show them: the input stream fills output 1, then
    // output 2, and so on (configured start indexes are ignored, so gaps or overlaps can't drop or mirror pixels).
    // This bypasses WS2812FX entirely (segments, 2D/matrix layout, LED maps, gamma), so WLED's layout configuration
    // has no effect on passthrough.
    void render(const uint8_t* data, size_t len, bool scaleBri) {
      if (realtimeOverride) return;
      while (!BusManager::canAllShow()) yield(); // previous frame still being sent (same wait as WS2812FX::show())
      const size_t n = len / 3;
      const uint8_t* ord = ORDER_MAP[_inputOrder];
      const uint8_t fade = scaleBri ? bri : 255;
      // CCT -1 disables white balance correction in BusDigital::setPixelColor() (only applied for 1900..10060 K)
      const int16_t oldCCT = Bus::getCCT();
      Bus::setCCT(-1);
      size_t i = 0; // input pixel index
      for (const auto& bus : BusManager::busses) {
        for (unsigned j = 0; j < bus->getLength(); j++, i++) {
          uint32_t c = 0;
          if (i < n) {
            const uint8_t* p = data + 3 * i;
            c = RGBW32(p[ord[0]], p[ord[1]], p[ord[2]], 0);
            if (fade != 255) c = color_fade(c, fade);
          }
          bus->setPixelColor(j, c);
        }
      }
      Bus::setCCT(oldCCT);
      _lastPixels = n;
      // full brightness at the bus: color_fade() with 255 is a no-op, so bytes pass unchanged. The bus value is set
      // directly so the brightness multiplier (LED settings) does not scale the output.
      BusManager::setBrightness16(255U << 8);
      BusManager::show();
    }

    void passthroughLoop() {
      ensureRealtime();
      keepPowered();
      const uint8_t* data; size_t len;
      if (fetchFrame(data, len)) {
        _lastFrameMs = millis();
        _blanked = false;
        render(data, len, _applyBri);
      } else if (_noSignalBlack && !_blanked && millis() - _lastFrameMs > _noSignalMs) {
        render(nullptr, 0, false);
        _blanked = true;
      }
    }

    // ------------------------------------------------------------------------------------------
    // configuration
    // ------------------------------------------------------------------------------------------

    // strapping pins are rejected: the upstream signal could change the boot mode (GPIO12 high = 1.8 V flash, no boot)
    static const char* checkPin(int8_t pin) {
      if (pin < 0) return PSTR("no input pin configured");
      if (pin == 0 || pin == 2 || pin == 5 || pin == 12 || pin == 15) return PSTR("strapping pins (0, 2, 5, 12, 15) are not allowed");
      if (!PinManager::isPinOk(pin, false)) return PSTR("not a valid input pin");
      return nullptr;
    }

    void updateInputRate() {
      const uint32_t now = millis();
      const uint32_t dt = now - _rateTime;
      if (dt < 1000) return;
      const uint32_t in = _decoder.getStats().frames;
      _inFps = ((in - _rateIn) * 1000U + dt / 2) / dt;
      _rateIn = in; _rateTime = now;
    }

    // min/max widths (ns) of pulses decoded as 0 and as 1, from the histogram
    void pulseRange(bool ones, uint32_t& lo, uint32_t& hi) const {
      const auto& h = _decoder.getStats().pulseHist;
      const unsigned thr = _decoder.getConfig().highThreshold;
      int first = -1, last = -1;
      for (unsigned i = ones ? thr : 1; i < (ones ? Ws2812Decoder::HIST_BINS : thr) && i < Ws2812Decoder::HIST_BINS; i++) {
        if (h[i]) { if (first < 0) first = i; last = i; }
      }
      lo = first < 0 ? 0 : first * 1000U / _capRateMHz;
      hi = last  < 0 ? 0 : last  * 1000U / _capRateMHz;
    }

  public:
    void setup() override {}

    void loop() override {
      if (_cfgDirty.exchange(false)) {
        // settings changed: stop capture; it restarts below with the new settings if passthrough is on
        stopCapture();
        _startFailedAt = 0;
        _error = nullptr;
      }

      if (_suspended) {
        // OTA update running: free the capture resources, but leave the strip and the realtime lock alone. The OTA
        // handler has suspended the strip and is resetting the segments from the web server task; resuming or
        // showing the strip here would race with that. If the update fails, loop() continues where it left off.
        stopCapture();
        return;
      }

      const bool want = _ptOn;
      if (want && !_capturing) {
        // retry failed starts every 5 s (e.g. after an LED configuration change removed a conflict)
        if (!_startFailedAt || millis() - _startFailedAt > 5000) {
          if (startCapture()) _startFailedAt = 0;
          else { _startFailedAt = millis() | 1; DEBUG_PRINTF_P(PSTR("Passthrough: %s\n"), _error); }
        }
      } else if (!want) {
        // inactive: no pin, no I2S, no realtime lock
        if (_capturing) stopCapture();
        _error = nullptr;
        _startFailedAt = 0;
      }
      if (!want || !_capturing) {
        releaseRealtime();
        return;
      }

      // LED settings can change at runtime: stop if the output now needs our I2S peripheral
      if (millis() - _lastConflictCheck > 1000) {
        _lastConflictCheck = millis();
        const char* c = findConflict();
        if (c) { stopCapture(); _error = c; _startFailedAt = millis() | 1; releaseRealtime(); return; }
      }
      updateInputRate();
      passthroughLoop();
    }

    void onUpdateBegin(bool init) override { _suspended = init; }

    // ---- JSON state API: {"passthrough":{"on":true|false|"t", "reset":true}} ----

    void addToJsonState(JsonObject& root) override {
      JsonObject pt = root.createNestedObject(FPSTR(_ptState));
      pt["on"] = (bool)_ptOn;
    }

    void readFromJsonState(JsonObject& root) override {
      JsonObject pt = root[FPSTR(_ptState)];
      if (pt.isNull()) return;
      JsonVariant on = pt["on"];
      if (on.is<bool>()) _ptOn = on.as<bool>();
      else if (on.is<const char*>() && on.as<const char*>()[0] == 't') _ptOn = !_ptOn;
      if (pt[F("reset")] | false) { _clearStatsReq = true; _overruns = 0; }
    }

    // ---- Info panel / JSON info ----

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      const auto& st = _decoder.getStats();

      // state + toggle button
      JsonArray arr = user.createNestedArray(FPSTR(_ptName));
      String s;
      if (_error) { s = F("error: "); s += FPSTR(_error); s += ' '; } // otherwise the button state shows on/off (like AudioReactive)
      s += F("<button class=\"btn btn-xs\" onclick=\"requestJson({passthrough:{on:'t'}});\"><i class=\"icons ");
      s += _ptOn ? F("on") : F("off");
      s += F("\">&#xe08f;</i></button>");
      arr.add(s);

      if (!_capturing) return;
      char buf[64];
      arr = user.createNestedArray(F("PT input"));
      snprintf_P(buf, sizeof(buf), PSTR("%u fps, %u pixels"), _inFps, (unsigned)_lastPixels);
      arr.add(buf);

      // measured pulse widths: both ranges must stay clear of the threshold
      uint32_t z0, z1, o0, o1;
      pulseRange(false, z0, z1);
      pulseRange(true, o0, o1);
      arr = user.createNestedArray(F("PT bit widths"));
      snprintf_P(buf, sizeof(buf), PSTR("0: %u-%u ns, 1: %u-%u ns"), z0, z1, o0, o1);
      arr.add(buf);

      // dropped = frames with decode errors; overruns = capture fell behind (samples lost)
      arr = user.createNestedArray(F("PT stats"));
      snprintf_P(buf, sizeof(buf), PSTR("dropped %u, overruns %u"), st.droppedFrames, (unsigned)_overruns);
      arr.add(buf);
    }

    // ---- persistent settings ----

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_ptName));
      top[F("enabled")]     = _enabled;
      top[F("pin")]         = _inputPin;
      top[F("i2s")]         = _i2sPort;
      top[F("sampleRate")]  = _sampleRateMHz;
      top[F("thresholdNs")] = _thresholdNs;
      top[F("gapUs")]       = _gapUs;
      top[F("inputOrder")]  = _inputOrder;
      top[F("noSignalBlack")] = _noSignalBlack;
      top[F("noSignalMs")]  = _noSignalMs;
      top[F("applyBri")]    = _applyBri;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_ptName)];
      bool complete = !top.isNull();
      const bool wasEnabled = _enabled;
      complete &= getJsonValue(top[F("enabled")],     _enabled, false);
      complete &= getJsonValue(top[F("pin")],         _inputPin, (int8_t)33);
      complete &= getJsonValue(top[F("i2s")],         _i2sPort, (uint8_t)1);
      complete &= getJsonValue(top[F("sampleRate")],  _sampleRateMHz, (uint8_t)8);
      complete &= getJsonValue(top[F("thresholdNs")], _thresholdNs, (uint16_t)600);
      complete &= getJsonValue(top[F("gapUs")],       _gapUs, (uint16_t)50);
      complete &= getJsonValue(top[F("inputOrder")],  _inputOrder, (uint8_t)COL_ORDER_GRB);
      complete &= getJsonValue(top[F("noSignalBlack")], _noSignalBlack, false);
      complete &= getJsonValue(top[F("noSignalMs")],  _noSignalMs, (uint16_t)1000);
      complete &= getJsonValue(top[F("applyBri")],    _applyBri, false);

      if (_i2sPort > 1) _i2sPort = 1;
      if (_sampleRateMHz != 16) _sampleRateMHz = 8;
      _thresholdNs = constrain(_thresholdNs, 100, 1500);
      _gapUs       = constrain(_gapUs, 10, 1000);
      _noSignalMs  = constrain(_noSignalMs, 100, 60000);
      if (_inputOrder > COL_ORDER_MAX) _inputOrder = COL_ORDER_GRB;

      // the "enabled" setting sets the runtime state at boot and whenever it is changed in the settings page
      if (!_cfgRead || _enabled != wasEnabled) _ptOn = _enabled;
      _cfgRead = true;
      _cfgDirty = true;
      return complete;
    }

    void appendConfigData(Print& s) override {
      s.print(F("dd=addDropdown('Passthrough','i2s');addOption(dd,'I2S0 (conflicts with AudioReactive)',0);addOption(dd,'I2S1 (conflicts with parallel I2S LED output)',1);"));
      s.print(F("dd=addDropdown('Passthrough','sampleRate');addOption(dd,'8 MHz',8);addOption(dd,'16 MHz (tight upstream timing)',16);"));
      s.print(F("dd=addDropdown('Passthrough','inputOrder');addOption(dd,'GRB',0);addOption(dd,'RGB',1);addOption(dd,'BRG',2);addOption(dd,'RBG',3);addOption(dd,'BGR',4);addOption(dd,'GBR',5);"));
      s.print(F("addInfo('Passthrough:enabled',1,'passthrough active after boot (input: WS281x RGB only)');"));
      s.print(F("addInfo('Passthrough:pin',1,'input (not 0, 2, 5, 12, 15)');"));
      s.print(F("addInfo('Passthrough:thresholdNs',1,'ns high pulse: longer = 1');"));
      s.print(F("addInfo('Passthrough:gapUs',1,'&micro;s low = end of frame');"));
      s.print(F("addInfo('Passthrough:inputOrder',1,'set equal to the LED output color order for byte-identical output');"));
      s.print(F("addInfo('Passthrough:noSignalBlack',1,'go black when no frames arrive (otherwise hold the last frame)');"));
      s.print(F("addInfo('Passthrough:noSignalMs',1,'ms without frames before going black');"));
      s.print(F("addInfo('Passthrough:applyBri',1,'scale passthrough data with WLED brightness');"));
    }

};

#endif // PASSTHROUGH_SUPPORTED

static UsermodPassthrough passthroughUsermod;
REGISTER_USERMOD(passthroughUsermod);
