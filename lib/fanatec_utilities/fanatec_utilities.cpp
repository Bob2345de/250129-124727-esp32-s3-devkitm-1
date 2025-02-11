#include "fanatec_utilities.h"
#include <Arduino.h>
#include <string.h>

void transposeBits(const uint8_t data[3], uint8_t dataout[3], const uint8_t transpose[24]) {
  memset(dataout, 0, 3);
  for (uint8_t i = 0; i < 24; i++) {
    uint8_t input_byte = i / 8;
    uint8_t input_bit = i % 8;
    if (data[input_byte] & (1 << input_bit)) {
      uint8_t output_pos = transpose[i];
      uint8_t output_byte = output_pos / 8;
      uint8_t output_bit = output_pos % 8;
      dataout[output_byte] |= (1 << output_bit);
    }
  }
}
