#pragma once

#include <glib.h>

G_BEGIN_DECLS

typedef enum {
  MORSE_DECODER_IDLE,
  MORSE_DECODER_CALIBRATING,
  MORSE_DECODER_LOCKED,
} MorseDecoderState;

typedef struct MorseDecoder MorseDecoder;

struct MorseDecoder {
  MorseDecoderState state;

  /* TODO: ROI (x, y, width, height) */
  /* TODO: threshold, off_level, on_level, hysteresis */
  /* TODO: Morse FSM state (current symbol, last transition time) */
  /* TODO: timing parameters (unit time in seconds, derived from wpm) */
  /* TODO: decoded text buffer */
};

MorseDecoder *morse_decoder_new(void);
void morse_decoder_free(MorseDecoder *decoder);

void morse_decoder_reset(MorseDecoder *decoder);

/* TODO: call once when detection is triggered */
void morse_decoder_start(MorseDecoder *decoder);

/* TODO: feed per-frame brightness sample and buffer timestamp (seconds) */
void morse_decoder_process_sample(MorseDecoder *decoder, gfloat brightness,
                                  gdouble timestamp_sec);

/* TODO: read-only accessors for debug / properties */
const gchar *morse_decoder_get_text(MorseDecoder *decoder);
MorseDecoderState morse_decoder_get_state(MorseDecoder *decoder);

G_END_DECLS
