// IR tools
static const char __attribute__((unused)) * TAG = "IR";

#include <revk.h>
#include <driver/rmt_rx.h>
#include <ir.h>

static const char *const ir_coding[] = { "UNKNOWN", "IDLE", "ZERO", "PDC", "PLC" };

typedef struct ir_config_s
{
   rmt_symbol_word_t ir_rx_symbols[256];
   rmt_rx_done_event_data_t ir_rx_data;
   revk_gpio_t gpio;
   ir_callback_t *cb;
   uint8_t raw[64];             // 2 bits per symbol
} ir_config_t;

static bool
ir_rx_done_callback (rmt_channel_handle_t channel, const rmt_rx_done_event_data_t * edata, void *user_data)
{
   BaseType_t high_task_wakeup = pdFALSE;
   QueueHandle_t receive_queue = (QueueHandle_t) user_data;
   xQueueSendFromISR (receive_queue, edata, &high_task_wakeup);
   return high_task_wakeup == pdTRUE;
}

static void
ir_task (void *arg)
{
   ir_config_t *c = arg;
   if (!c || !c->gpio.set)
   {
      vTaskDelete (NULL);
      return;
   }
   revk_gpio_input (c->gpio);
   rmt_rx_channel_config_t rx_channel_cfg = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 1000000,
      .mem_block_symbols = sizeof (c->ir_rx_symbols) / sizeof (*c->ir_rx_symbols),
      .gpio_num = c->gpio.num,
      .flags.invert_in = c->gpio.invert,
#ifdef	CONFIG_IDF_TARGET_ESP32S3
      .flags.with_dma = 1,
#endif
   };
   rmt_channel_handle_t rx_channel = NULL;
   REVK_ERR_CHECK (rmt_new_rx_channel (&rx_channel_cfg, &rx_channel));
   if (!rx_channel)
   {
      ESP_LOGE (TAG, "No RMT Rx");
      vTaskDelete (NULL);
      return;
   }
   QueueHandle_t receive_queue = xQueueCreate (1, sizeof (rmt_rx_done_event_data_t));
   if (!receive_queue)
   {
      ESP_LOGE (TAG, "No RMT Queue");
      vTaskDelete (NULL);
      return;
   }
   rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = ir_rx_done_callback,
   };
   REVK_ERR_CHECK (rmt_rx_register_event_callbacks (rx_channel, &cbs, receive_queue));

   rmt_receive_config_t receive_config = {
      .signal_range_min_ns = 2500,
      .signal_range_max_ns = 12500000,
   };

   REVK_ERR_CHECK (rmt_enable (rx_channel));
   REVK_ERR_CHECK (rmt_receive (rx_channel, c->ir_rx_symbols, sizeof (c->ir_rx_symbols), &receive_config));

   ESP_LOGE (TAG, "IR started %d", c->gpio.num);
   uint8_t idle = 0;
   while (1)
   {
      uint16_t bit = 0,
         lead0 = 0,
         lead1 = 0;
      jo_t j = NULL;
      if (xQueueReceive (receive_queue, &c->ir_rx_data, pdMS_TO_TICKS (50)) == pdPASS)
      {
         idle = 1;
         //ESP_LOGE (TAG, "Symbols %d %d/%d", c->ir_rx_data.num_symbols,c->ir_rx_symbols[0].level0,c->ir_rx_symbols[0].level1);
         int i = 0;
         if (c->ir_rx_symbols[i].duration0 > 2000)
         {
            lead0 = c->ir_rx_symbols[i].duration0;
            lead1 = c->ir_rx_symbols[i].duration1;
            i++;
         }
         if (!j && irlog)
         {
            j = jo_object_alloc ();
            jo_int (j, "gpio", c->gpio.num);
            if (lead0)
            {
               jo_array (j, "lead");
               jo_int (j, NULL, lead0);
               jo_int (j, NULL, lead1);
               jo_close (j);
            }
            if (irdebug)
               jo_array (j, "timing");
         }
         while (i < c->ir_rx_data.num_symbols)
         {
            if (irdebug)
            {
               jo_int (j, NULL, c->ir_rx_symbols[i].duration0);
               if (c->ir_rx_symbols[i].duration1 || i + 1 < c->ir_rx_data.num_symbols)
                  jo_int (j, NULL, c->ir_rx_symbols[i].duration1);
            }
            c->raw[bit / 8] = (c->raw[bit / 8] >> 1) | (c->ir_rx_symbols[i].duration0 >= 800 ? 0x80 : 0);
            bit++;
            if (c->ir_rx_symbols[i].duration1 || i + 1 < c->ir_rx_data.num_symbols)
            {
               c->raw[bit / 8] = (c->raw[bit / 8] >> 1) | (c->ir_rx_symbols[i].duration1 >= 800 ? 0x80 : 0);
               bit++;
            }
            i++;
         }
         // Next
         REVK_ERR_CHECK (rmt_receive (rx_channel, c->ir_rx_symbols, sizeof (c->ir_rx_symbols), &receive_config));
         if (bit)
         {
            if (irdebug)
               jo_close (j);
            if (bit & 7)
               c->raw[bit / 8] >>= (8 - (bit & 7));
            uint8_t byte = (bit + 7) / 8;
            // Work out coding
            uint8_t coding = 0;
            uint8_t b;
            if (!coding)
            {
               for (b = 0; b < byte && !c->raw[b]; b++);
               if (b == byte)
                  coding = IR_ZERO;
            }
            if (!coding)
            {
               for (b = 0; b < byte && !(c->raw[b] & 0xAA); b++);
               if (b == byte)
               {                // all 0 are short, so data in 1
                  coding = IR_PLC;
                  b = 0;
                  while (b < bit / 2)
                  {
                     if (c->raw[b / 4] & (1 << ((b & 3) * 2)))
                        c->raw[b / 8] |= (1 << (b & 7));
                     else
                        c->raw[b / 8] &= ~(1 << (b & 7));
                     b++;
                  }
                  bit = b;
               }
            }
            if (!coding)
            {
               for (b = 0; b < byte && !(c->raw[b] & 0x55); b++);
               if (b == byte)
               {                // all 1 are short so data in 0
                  coding = IR_PDC;
                  b = 0;
                  while (b < bit / 2)
                  {
                     if (c->raw[b / 4] & (2 << ((b & 3) * 2)))
                        c->raw[b / 8] |= (1 << (b & 7));
                     else
                        c->raw[b / 8] &= ~(1 << (b & 7));
                     b++;
                  }
                  bit = b;
               }
            }
            // Bi Phase coding for another day
            if (bit & 7)
               c->raw[bit / 8] >>= (8 - (bit & 7));
            byte = (bit + 7) / 8;
            if (c->cb)
               c->cb (coding, lead0, lead1, bit, c->raw);
            if (irlog && j)
            {
               jo_string (j, "coding", ir_coding[coding]);
               jo_array (j, "data");
               for (int i = 0; i < byte; i++)
                  jo_stringf (j, NULL, "%02X", c->raw[i]);
               jo_close (j);
               jo_int (j, "bits", bit);
               revk_info ("ir", &j);
            }
         }
         if (j)
            jo_free (&j);
      } else
      {
         if (idle && idle++ == 3)
         {
            //ESP_LOGE (TAG, "IR idle");
            idle = 0;
            if (c->cb)
               c->cb (IR_IDLE, 0, 0, 0, NULL);
         }
      }
   }
   vTaskDelete (NULL);
}

void
ir_start (revk_gpio_t gpio, ir_callback_t * cb)
{
   ir_config_t *c = mallocspi (sizeof (*c));
   c->gpio = gpio;
   c->cb = cb;
   revk_task ("ir", ir_task, c, 4);
}
