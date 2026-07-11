#include "morse_decoder.hpp"
#include "morse_timing.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
constexpr gdouble kMinUnitSeconds = 0.015;
constexpr gdouble kMaxUnitSeconds = 1.2;

struct MorseMapEntry {
  const char *symbol;
  char decoded;
};

const MorseMapEntry kMorseTable[] = {
    {".-", 'A'},    {"-...", 'B'},  {"-.-.", 'C'},  {"-..", 'D'},
    {".", 'E'},     {"..-.", 'F'},  {"--.", 'G'},   {"....", 'H'},
    {"..", 'I'},    {".---", 'J'},  {"-.-", 'K'},   {".-..", 'L'},
    {"--", 'M'},    {"-.", 'N'},    {"---", 'O'},   {".--.", 'P'},
    {"--.-", 'Q'},  {".-.", 'R'},   {"...", 'S'},   {"-", 'T'},
    {"..-", 'U'},   {"...-", 'V'},  {".--", 'W'},   {"-..-", 'X'},
    {"-.--", 'Y'},  {"--..", 'Z'},  {"-----", '0'}, {".----", '1'},
    {"..---", '2'}, {"...--", '3'}, {"....-", '4'}, {".....", '5'},
    {"-....", '6'}, {"--...", '7'}, {"---..", '8'}, {"----.", '9'},
};

gdouble clamp_unit(gdouble value) {
  return std::max(kMinUnitSeconds, std::min(kMaxUnitSeconds, value));
}

char morse_symbol_to_char(const gchar *symbol) {
  for (const auto &entry : kMorseTable) {
    if (std::strcmp(entry.symbol, symbol) == 0) {
      return entry.decoded;
    }
  }
  return '?';
}

void append_char(MorseDecoder *decoder, char c) {
  if (decoder->decoded_text == nullptr) {
    return;
  }
  g_string_append_c(decoder->decoded_text, c);
  if (decoder->max_text_chars > 0 &&
      decoder->decoded_text->len > decoder->max_text_chars) {
    const guint trim = decoder->decoded_text->len - decoder->max_text_chars;
    g_string_erase(decoder->decoded_text, 0, trim);
  }
}

void append_space(MorseDecoder *decoder) {
  if (decoder->decoded_text == nullptr || decoder->decoded_text->len == 0) {
    return;
  }
  // keep a single separator even during long word gaps.
  if (decoder->decoded_text->str[decoder->decoded_text->len - 1] != ' ') {
    append_char(decoder, ' ');
  }
}

void finalize_symbol(MorseDecoder *decoder) {
  if (decoder->symbol_len == 0) {
    return;
  }
  decoder->symbol_buf[decoder->symbol_len] = '\0';
  append_char(decoder, morse_symbol_to_char(decoder->symbol_buf));
  decoder->symbol_len = 0;
  decoder->symbol_buf[0] = '\0';
}

void classify_on_duration(MorseDecoder *decoder, gdouble duration) {
  const gdouble unit = decoder->unit_sec;
  if (duration <= 0.0) {
    return;
  }

  const gdouble units = duration / unit;
  char sym = '.';
  if (units >= decoder->dash_min_units) {
    sym = '-';
  } else if (units <= decoder->dot_max_units) {
    sym = '.';
  } else {
    sym = (std::fabs(units - 1.0) <= std::fabs(units - 3.0)) ? '.' : '-';
  }
  if (decoder->symbol_len + 1 < G_N_ELEMENTS(decoder->symbol_buf)) {
    decoder->symbol_buf[decoder->symbol_len++] = sym;
    decoder->symbol_buf[decoder->symbol_len] = '\0';
  }
}

void classify_off_duration(MorseDecoder *decoder, gdouble off_duration) {
  const gdouble letter_gap_threshold =
      MORSE_LETTER_GAP_UNITS * decoder->gap_detect_ratio * decoder->unit_sec;
  const gdouble word_gap_threshold =
      MORSE_WORD_GAP_UNITS * decoder->gap_detect_ratio * decoder->unit_sec;
  if (decoder->symbol_pending_gap && off_duration >= letter_gap_threshold) {
    finalize_symbol(decoder);
    decoder->symbol_pending_gap = FALSE;
  }
  if (off_duration >= word_gap_threshold) {
    append_space(decoder);
    decoder->word_gap_emitted = TRUE;
  }
}

void commit_led_transition(MorseDecoder *decoder, gboolean next_led_on,
                           gdouble timestamp_sec) {
  const gdouble segment_duration =
      timestamp_sec - decoder->last_transition_sec;
  if (decoder->led_on) {
    classify_on_duration(decoder, segment_duration);
    decoder->symbol_pending_gap = decoder->symbol_len > 0;
    decoder->word_gap_emitted = FALSE;
  } else {
    classify_off_duration(decoder, segment_duration);
  }

  decoder->led_on = next_led_on;
  decoder->last_transition_sec = timestamp_sec;
  decoder->pending_led_on = next_led_on;
  decoder->pending_since_sec = timestamp_sec;
}
} // namespace

MorseDecoder *morse_decoder_new(void) {
  MorseDecoder *decoder = g_new0(MorseDecoder, 1);
  decoder->state = MORSE_DECODER_IDLE;
  decoder->wpm = MORSE_DEFAULT_WPM;
  decoder->unit_sec = morse_paris_dit_seconds(decoder->wpm);
  decoder->baseline = 0.0f;
  decoder->sample_initialized = FALSE;
  decoder->led_on = FALSE;
  decoder->pending_led_on = FALSE;
  decoder->pending_since_sec = 0.0;
  decoder->last_timestamp_sec = 0.0;
  decoder->last_transition_sec = 0.0;
  decoder->symbol_len = 0;
  decoder->symbol_pending_gap = FALSE;
  decoder->word_gap_emitted = FALSE;
  decoder->decoded_text = g_string_new("");
  decoder->max_text_chars = 256;
  decoder->on_margin = MORSE_DEFAULT_ON_MARGIN;
  decoder->min_transition_units = MORSE_DEFAULT_MIN_TRANSITION_UNITS;
  decoder->gap_detect_ratio = MORSE_DEFAULT_GAP_DETECT_RATIO;
  decoder->dot_max_units = MORSE_DEFAULT_DOT_MAX_UNITS;
  decoder->dash_min_units = MORSE_DEFAULT_DASH_MIN_UNITS;
  return decoder;
}

void morse_decoder_free(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return;
  }
  if (decoder->decoded_text != nullptr) {
    g_string_free(decoder->decoded_text, TRUE);
    decoder->decoded_text = nullptr;
  }
  g_free(decoder);
}

void morse_decoder_reset(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return;
  }
  decoder->sample_initialized = FALSE;
  decoder->led_on = FALSE;
  decoder->pending_led_on = FALSE;
  decoder->pending_since_sec = 0.0;
  decoder->last_timestamp_sec = 0.0;
  decoder->last_transition_sec = 0.0;
  decoder->symbol_len = 0;
  decoder->symbol_buf[0] = '\0';
  decoder->symbol_pending_gap = FALSE;
  decoder->word_gap_emitted = FALSE;
  if (decoder->decoded_text != nullptr) {
    g_string_assign(decoder->decoded_text, "");
  }
}

void morse_decoder_start(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return;
  }
  morse_decoder_reset(decoder);
  decoder->state = MORSE_DECODER_LOCKED;
}

void morse_decoder_process_sample(MorseDecoder *decoder, gfloat red_dominance,
                                  gdouble timestamp_sec) {
  if (decoder == nullptr || decoder->state != MORSE_DECODER_LOCKED) {
    return;
  }

  if (!decoder->sample_initialized) {
    decoder->sample_initialized = TRUE;
    decoder->baseline = red_dominance;
    decoder->last_timestamp_sec = timestamp_sec;
    decoder->last_transition_sec = timestamp_sec;
    decoder->pending_led_on = decoder->led_on;
    decoder->pending_since_sec = timestamp_sec;
    return;
  }
  if (timestamp_sec <= decoder->last_timestamp_sec) {
    timestamp_sec = decoder->last_timestamp_sec + 1e-3;
  }

  const gfloat dt =
      static_cast<gfloat>(timestamp_sec - decoder->last_timestamp_sec);

  // update the baseline for the signal.
  if (!decoder->led_on) {
    const gfloat alpha = std::min(0.25f, 0.1f + 2.0f * dt);
    decoder->baseline += alpha * (red_dominance - decoder->baseline);
  }

  const gfloat threshold_on = decoder->baseline + decoder->on_margin;
  const gfloat threshold_off = decoder->baseline + decoder->on_margin * 0.5f;

  gboolean next_led_on = decoder->led_on;
  if (!decoder->led_on && red_dominance >= threshold_on) {
    next_led_on = TRUE;
  } else if (decoder->led_on && red_dominance <= threshold_off) {
    next_led_on = FALSE;
  }

  const gdouble min_hold =
      decoder->min_transition_units * decoder->unit_sec;

  if (next_led_on == decoder->led_on) {
    decoder->pending_led_on = decoder->led_on;
    decoder->pending_since_sec = timestamp_sec;
  } else if (decoder->pending_led_on == decoder->led_on) {
    decoder->pending_led_on = next_led_on;
    decoder->pending_since_sec = timestamp_sec;
  } else if (decoder->pending_led_on == next_led_on) {
    if (timestamp_sec - decoder->pending_since_sec >= min_hold) {
      commit_led_transition(decoder, next_led_on, timestamp_sec);
    }
  } else {
    decoder->pending_led_on = next_led_on;
    decoder->pending_since_sec = timestamp_sec;
  }

  // check if the off duration is long enough to be a gap.
  if (!decoder->led_on) {
    const gdouble off_duration = timestamp_sec - decoder->last_transition_sec;
    classify_off_duration(decoder, off_duration);
  }

  decoder->last_timestamp_sec = timestamp_sec;
}

void morse_decoder_set_wpm(MorseDecoder *decoder, guint wpm) {
  if (decoder == nullptr) {
    return;
  }
  decoder->wpm = std::max(1u, wpm);
  decoder->unit_sec = clamp_unit(morse_paris_dit_seconds(decoder->wpm));
}

void morse_decoder_set_on_margin(MorseDecoder *decoder, gfloat on_margin) {
  if (decoder == nullptr) {
    return;
  }
  decoder->on_margin = std::max(0.0f, on_margin);
}

void morse_decoder_set_min_transition_units(MorseDecoder *decoder,
                                            gdouble min_transition_units) {
  if (decoder == nullptr) {
    return;
  }
  decoder->min_transition_units = std::max(0.0, min_transition_units);
}

void morse_decoder_set_gap_detect_ratio(MorseDecoder *decoder,
                                        gdouble gap_detect_ratio) {
  if (decoder == nullptr) {
    return;
  }
  decoder->gap_detect_ratio =
      std::max(0.01, std::min(1.0, gap_detect_ratio));
}

void morse_decoder_set_dot_max_units(MorseDecoder *decoder,
                                     gdouble dot_max_units) {
  if (decoder == nullptr) {
    return;
  }
  decoder->dot_max_units = std::max(0.0, dot_max_units);
}

void morse_decoder_set_dash_min_units(MorseDecoder *decoder,
                                      gdouble dash_min_units) {
  if (decoder == nullptr) {
    return;
  }
  decoder->dash_min_units = std::max(0.0, dash_min_units);
}

gfloat morse_decoder_get_on_margin(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return MORSE_DEFAULT_ON_MARGIN;
  }
  return decoder->on_margin;
}

gdouble morse_decoder_get_min_transition_units(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return MORSE_DEFAULT_MIN_TRANSITION_UNITS;
  }
  return decoder->min_transition_units;
}

gdouble morse_decoder_get_gap_detect_ratio(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return MORSE_DEFAULT_GAP_DETECT_RATIO;
  }
  return decoder->gap_detect_ratio;
}

gdouble morse_decoder_get_dot_max_units(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return MORSE_DEFAULT_DOT_MAX_UNITS;
  }
  return decoder->dot_max_units;
}

gdouble morse_decoder_get_dash_min_units(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return MORSE_DEFAULT_DASH_MIN_UNITS;
  }
  return decoder->dash_min_units;
}

const gchar *morse_decoder_get_text(MorseDecoder *decoder) {
  if (decoder == nullptr || decoder->decoded_text == nullptr) {
    return "";
  }
  return decoder->decoded_text->str;
}

MorseDecoderState morse_decoder_get_state(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return MORSE_DECODER_IDLE;
  }
  return decoder->state;
}

gboolean morse_decoder_get_led_on(MorseDecoder *decoder) {
  if (decoder == nullptr) {
    return FALSE;
  }
  return decoder->led_on;
}
