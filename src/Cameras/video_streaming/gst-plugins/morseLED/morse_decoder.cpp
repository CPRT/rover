#include "morse_decoder.hpp"

MorseDecoder *morse_decoder_new(void) {
  /* TODO */
  return nullptr;
}

void morse_decoder_free(MorseDecoder *decoder) {
  /* TODO */
  (void)decoder;
}

void morse_decoder_reset(MorseDecoder *decoder) {
  /* TODO */
  (void)decoder;
}

void morse_decoder_start(MorseDecoder *decoder) {
  /* TODO */
  (void)decoder;
}

void morse_decoder_process_sample(MorseDecoder *decoder, gfloat brightness,
                                  gdouble timestamp_sec) {
  /* TODO */
  (void)decoder;
  (void)brightness;
  (void)timestamp_sec;
}

const gchar *morse_decoder_get_text(MorseDecoder *decoder) {
  /* TODO */
  (void)decoder;
  return "";
}

MorseDecoderState morse_decoder_get_state(MorseDecoder *decoder) {
  /* TODO */
  (void)decoder;
  return MORSE_DECODER_IDLE;
}
