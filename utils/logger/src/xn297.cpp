
#include <stdint.h>

static const uint8_t xn297_scramble[] = {
  0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
  0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
  0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
  0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
  0x8e, 0xc5, 0x2f};

static uint8_t bit_reverse(uint8_t b_in)
{
    uint8_t b_out = 0;
    for (int i = 0; i < 8; ++i) {
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}

void xn297_scramble_data(uint8_t *p, int data_len, int address_len)
{
  for(uint8_t i = 0; i < data_len; i++) {
      p[i] = bit_reverse(p[i]) ^ bit_reverse(xn297_scramble[i+address_len]);
  }
}
                      
void xn297_scramble_address(uint8_t *adr, int address_len)
{
    for (int i = 0; i < address_len; ++i) {
        adr[i] = adr[i] ^ xn297_scramble[address_len-i-1];
    }
}
