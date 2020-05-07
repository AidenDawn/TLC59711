//
// Arduino library for controlling TI's TLC59711
//
// 21 Feb 2016 by Ulrich Stern
//
// open source (see LICENSE file)
//

#include "TLC59711.h"
#include <SPI.h>

#define OPTIMIZE_SET 0   // whether to optimize setChannel() and setRGB()

TLC59711::TLC59711(uint16_t numTLC, uint8_t clkPin, uint8_t dataPin):
    numTLC(numTLC), bufferSz(14*numTLC), clkPin(clkPin), dataPin(dataPin),
    buffer((uint16_t*) calloc(bufferSz, 2)), buffer2(0),
    beginCalled(false) {
  setTmgrst();
}

TLC59711::~TLC59711() {
  free(buffer);
  free(buffer2);
}

void TLC59711::begin(bool useSpi, unsigned int postXferDelayMicros) {
  end();
  useSpi_ = useSpi;
  postXferDelayMicros_ = postXferDelayMicros;
  beginCalled = true;
}

void TLC59711::beginFast(bool bufferXfer, uint32_t spiClock,
    unsigned int postXferDelayMicros) {
  begin(true, postXferDelayMicros);
  bufferXfer_ = bufferXfer;
  SPI.begin();
  SPI.beginTransaction(SPISettings(spiClock, MSBFIRST, SPI_MODE0));
  if (bufferXfer && !buffer2)
    buffer2 = (uint16_t*) malloc(2*bufferSz);
}
void TLC59711::beginSlow(unsigned int postXferDelayMicros, bool interrupts) {
  begin(false, postXferDelayMicros);
  noInterrupts = !interrupts;
  pinMode(clkPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

void TLC59711::setTmgrst(bool val) {
  // OUTTMG = 1, EXTGCK = 0, TMGRST = 0, DSPRPT = 1, BLANK = 0 -> 0x12
  fc = 0x12 + (val ? 0x4 : 0);
  setBrightness();
}

void TLC59711::setBrightness(uint16_t tlcIdx,
    uint8_t bcr, uint8_t bcg, uint8_t bcb) {
  if (tlcIdx < numTLC) {
    uint32_t ms32 = (uint32_t)0x25 << 26 | (uint32_t)fc << 21 |
      (uint32_t)bcb << 14 | (uint32_t)bcg << 7 | bcr;
    uint16_t idx = 14*tlcIdx+12;
    buffer[idx] = ms32;
    buffer[++idx] = ms32 >> 16;
  }
}
void TLC59711::setBrightness(uint8_t bcr, uint8_t bcg, uint8_t bcb) {
  for (uint16_t i=0; i<numTLC; i++)
    setBrightness(i, bcr, bcg, bcb);
}

#if OPTIMIZE_SET
#pragma GCC optimize("O3")
#endif
void TLC59711::setChannel(uint16_t idx, uint16_t val) {
  idx = 14*(idx/12) + idx%12;
    // lookup table would likely give significant speedup
  if (idx < bufferSz)
    buffer[idx] = val;
}

void TLC59711::setRGB(uint16_t idx, uint16_t r, uint16_t g, uint16_t b) {
  idx = 3*idx;
  setChannel(idx, r);
  setChannel(++idx, g);
  setChannel(++idx, b);
}
void TLC59711::setRGB(uint16_t r, uint16_t g, uint16_t b) {
  for (uint16_t i=0, n=4*numTLC; i<n; i++)
    setRGB(i, r, g, b);
}
#if OPTIMIZE_SET
#pragma GCC reset_options
#endif

#ifdef __AVR__

#pragma GCC optimize("O3")
#define WAIT_SPIF while (!(SPSR & _BV(SPIF))) { }
void TLC59711::xferSpi() {
  cli();
  uint8_t* p = (uint8_t*)buffer + bufferSz*2;
  uint8_t out = *--p;
  while (true) {
    SPDR = out;
    if (p == (uint8_t*)buffer)
      break;
    out = *--p;
    WAIT_SPIF
  }
  WAIT_SPIF
}
#pragma GCC reset_options

#else

static void reverseMemcpy(void *dst, void *src, size_t count) {
  uint8_t* s = (uint8_t*)src;
  while (count-- > 0)
    *((uint8_t*)dst+count) = *s++;
}

void TLC59711::xferSpi() {
  reverseMemcpy(buffer2, buffer, bufferSz*2);
  cli();
  SPI.transfer(buffer2, bufferSz*2);
}

#endif

void TLC59711::xferSpi16() {
  cli();
  for (int i=bufferSz-1; i >= 0; i--)
    SPI.transfer16(buffer[i]);
}

void TLC59711::xferShiftOut() {
  if (noInterrupts)
    cli();
  for (int i=bufferSz-1; i >= 0; i--) {
    uint16_t val = buffer[i];
    shiftOut(dataPin, clkPin, MSBFIRST, val >> 8);
    shiftOut(dataPin, clkPin, MSBFIRST, val);
  }
}

void TLC59711::write() {
  if (!beginCalled)
    return;
  uint8_t oldSREG = SREG;
  if (useSpi_) {
    if (bufferXfer_)
      xferSpi();
    else
      xferSpi16();
  }
  else
    xferShiftOut();
  SREG = oldSREG;
  // delay to make sure the TLC59711s read (latch) their shift registers;
  // the delay required is 8 times the duration between the last two SCKI
  // rising edges (plus 1.34 us); see datasheet pg. 22 for details
  delayMicroseconds(postXferDelayMicros_);
}

void TLC59711::end() {
  if (beginCalled && useSpi_) {
    SPI.endTransaction();
    SPI.end();
  }
}

