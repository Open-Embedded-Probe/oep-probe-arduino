// Host tests: PadFilter (OepPadFilter.h) on the fake glitch filter driver - the pad's filter held by count: made at the
// first take, removed at the last give, so the spi-target's CS and a logic capture listening on that pin with its
// glitch filter each keep it on while they want it, whichever lets go first; a refused filter is not counted.
#include <stdio.h>

#include "OepPadFilter.h"

using namespace oep;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

int main() {
  const int cs = 54, other = 6;
  // the spi-target takes CS's filter, then a capture listening on CS takes it too: one filter, on
  CHECK(PadFilter::take(cs));
  FakeGlitchFilter *made = g_fake_glitch_last;
  CHECK(g_fake_glitch_filters == 1 && made && made->gpio == cs && made->enabled);
  CHECK(PadFilter::take(cs));
  CHECK(g_fake_glitch_filters == 1 && PadFilter::holders(cs) == 2);
  // the capture ends first: the spi-target's CS stays filtered
  PadFilter::give(cs);
  CHECK(g_fake_glitch_filters == 1 && made->enabled && PadFilter::holders(cs) == 1);
  // another pin is its own filter
  CHECK(PadFilter::take(other));
  CHECK(g_fake_glitch_filters == 2);
  PadFilter::give(other);
  CHECK(g_fake_glitch_filters == 1 && PadFilter::holders(other) == 0);
  // the spi-target stops: the pad as it was
  PadFilter::give(cs);
  CHECK(g_fake_glitch_filters == 0 && PadFilter::holders(cs) == 0);
  // a give with nothing held, a pin out of range: nothing
  PadFilter::give(cs);
  CHECK(g_fake_glitch_filters == 0 && PadFilter::holders(cs) == 0);
  CHECK(!PadFilter::take(-1) && !PadFilter::take(PadFilter::kPins) && g_fake_glitch_filters == 0);
  // the other order: the capture first, the spi-target after, the capture's end leaves it on
  CHECK(PadFilter::take(cs) && PadFilter::take(cs));
  PadFilter::give(cs);
  CHECK(g_fake_glitch_filters == 1 && g_fake_glitch_last && g_fake_glitch_last->enabled);
  PadFilter::give(cs);
  CHECK(g_fake_glitch_filters == 0);
  printf("pad filter: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
