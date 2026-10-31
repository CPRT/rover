#pragma once

#include <glib.h>

// PARIS reference timing: dit length = 1.2 / WPM seconds (18 WPM -> 66.7 ms).
constexpr guint MORSE_DEFAULT_WPM = 18;
constexpr gdouble MORSE_PARIS_SECONDS_PER_DIT = 1.2;

constexpr gdouble MORSE_ELEMENT_GAP_UNITS = 1.0;
constexpr gdouble MORSE_LETTER_GAP_UNITS = 3.0;
constexpr gdouble MORSE_WORD_GAP_UNITS = 7.0;
constexpr gdouble MORSE_DASH_UNITS = 3.0;

inline gdouble morse_paris_dit_seconds(guint wpm) {
  return MORSE_PARIS_SECONDS_PER_DIT / static_cast<gdouble>(wpm > 0 ? wpm : 1);
}
