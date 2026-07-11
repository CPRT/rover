#pragma once

#include <glib.h>

G_BEGIN_DECLS

typedef enum {
  MORSE_DECODER_IDLE,
  MORSE_DECODER_LOCKED,
} MorseDecoderState;

#define MORSE_DEFAULT_ON_MARGIN 0.015f
#define MORSE_DEFAULT_MIN_TRANSITION_UNITS 0.08
#define MORSE_DEFAULT_GAP_DETECT_RATIO 0.8
#define MORSE_DEFAULT_DOT_MAX_UNITS 1.8
#define MORSE_DEFAULT_DASH_MIN_UNITS 2.2

typedef struct MorseDecoder MorseDecoder;

struct MorseDecoder {
  MorseDecoderState state;
  guint wpm;
  gdouble unit_sec;
  gfloat baseline;
  gboolean sample_initialized;
  gboolean led_on;
  gboolean pending_led_on;
  gdouble pending_since_sec;
  gdouble last_timestamp_sec;
  gdouble last_transition_sec;
  gchar symbol_buf[16];
  guint symbol_len;
  gboolean symbol_pending_gap;
  gboolean word_gap_emitted;
  GString *decoded_text;
  guint max_text_chars;

  gfloat on_margin;
  gdouble min_transition_units;
  gdouble gap_detect_ratio;
  gdouble dot_max_units;
  gdouble dash_min_units;
};

MorseDecoder *morse_decoder_new(void);
void morse_decoder_free(MorseDecoder *decoder);

void morse_decoder_reset(MorseDecoder *decoder);
void morse_decoder_start(MorseDecoder *decoder);

void morse_decoder_process_sample(MorseDecoder *decoder, gfloat red_dominance,
                                  gdouble timestamp_sec);

void morse_decoder_set_wpm(MorseDecoder *decoder, guint wpm);
void morse_decoder_set_on_margin(MorseDecoder *decoder, gfloat on_margin);
void morse_decoder_set_min_transition_units(MorseDecoder *decoder,
                                            gdouble min_transition_units);
void morse_decoder_set_gap_detect_ratio(MorseDecoder *decoder,
                                        gdouble gap_detect_ratio);
void morse_decoder_set_dot_max_units(MorseDecoder *decoder,
                                     gdouble dot_max_units);
void morse_decoder_set_dash_min_units(MorseDecoder *decoder,
                                      gdouble dash_min_units);

gfloat morse_decoder_get_on_margin(MorseDecoder *decoder);
gdouble morse_decoder_get_min_transition_units(MorseDecoder *decoder);
gdouble morse_decoder_get_gap_detect_ratio(MorseDecoder *decoder);
gdouble morse_decoder_get_dot_max_units(MorseDecoder *decoder);
gdouble morse_decoder_get_dash_min_units(MorseDecoder *decoder);

const gchar *morse_decoder_get_text(MorseDecoder *decoder);
MorseDecoderState morse_decoder_get_state(MorseDecoder *decoder);
gboolean morse_decoder_get_led_on(MorseDecoder *decoder);

G_END_DECLS
