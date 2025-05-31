// IR tools
#include <stdint.h>

enum
{
   IR_UNKNOWN,                  // Unknown coding, so raw data provided
   IR_IDLE,
   IR_ZERO,                     // All zeros
   IR_PDC,                      // Pulse Distance coding
   IR_PLC,                      // Pulse length coding
};
typedef void ir_callback_t (uint8_t coding, uint16_t lead0, uint16_t lead1, uint8_t len, uint8_t * data);

void ir_start(revk_gpio_t gpio,ir_callback_t *cb);
