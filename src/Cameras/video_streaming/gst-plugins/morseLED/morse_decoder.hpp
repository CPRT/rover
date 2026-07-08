#pragma once

#include <glib.h>

G_BEGIN_DECLS

typedef enum {
  MORSE_DECODER_IDLE,
  MORSE_DECODER_LOCKED,
} MorseDecoderState;

typedef struct MorseDecoder MorseDecoder;

struct MorseDecoder {
  MorseDecoderState state;
  guint wpm;
  gdouble unit_sec;
  gfloat baseline;
  gboolean sample_initialized;
  gboolean led_on;
  gdouble last_timestamp_sec;
  gdouble last_transition_sec;
  gchar symbol_buf[16];
  guint symbol_len;
  gboolean symbol_pending_gap;
  gboolean word_gap_emitted;
  GString *decoded_text;
  guint max_text_chars;
};

MorseDecoder *morse_decoder_new(void);
void morse_decoder_free(MorseDecoder *decoder);

void morse_decoder_reset(MorseDecoder *decoder);
void morse_decoder_start(MorseDecoder *decoder);

void morse_decoder_process_sample(MorseDecoder *decoder, gfloat red_dominance,
                                  gdouble timestamp_sec);

void morse_decoder_set_wpm(MorseDecoder *decoder, guint wpm);

const gchar *morse_decoder_get_text(MorseDecoder *decoder);
MorseDecoderState morse_decoder_get_state(MorseDecoder *decoder);
gboolean morse_decoder_get_led_on(MorseDecoder *decoder);

G_END_DECLS
