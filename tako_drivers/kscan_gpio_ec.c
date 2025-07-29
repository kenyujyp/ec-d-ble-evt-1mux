/*
 * Copyright (c) 2023 Sviatoslav Bulbakha
 *
 * SPDX-License-Identifier: MIT
 */

 #include "debounce.h"
 #include "kscan_gpio.h"
 
 #include <stdlib.h>
 #include <zephyr/device.h>
 #include <zephyr/drivers/adc.h>
 #include <zephyr/drivers/kscan.h>
 #include <zephyr/logging/log.h>
 #include <zmk/event_manager.h>
 #include <zmk/events/activity_state_changed.h>
 
 LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);
 
 #define DT_DRV_COMPAT zmk_kscan_gpio_ec
 
 #define INST_ROWS_LEN(n) DT_INST_PROP_LEN(n, row_gpios)
 #define INST_MUX_SELS_LEN(n) DT_INST_PROP_LEN(n, mux_sel_gpios)
 #define INST_COL_CHANNELS_LEN(n) DT_INST_PROP_LEN(n, col_channels)
 
 #define KSCAN_GPIO_ROW_CFG_INIT(idx, inst_idx)                                 \
   KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), row_gpios, idx)
 #define KSCAN_GPIO_MUX_SEL_CFG_INIT(idx, inst_idx)                             \
   KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), mux_sel_gpios, idx)
 
 #define INST_MATRIX_LEN(n) (INST_ROWS_LEN(n) * INST_COL_CHANNELS_LEN(n))
 
 // clang-format off
 
 const uint16_t actuation_threshold[] = {
   500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500,
   500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500,
   500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500,
   500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500,
   500, 500, 500, 500, 500,
 };
 
 const uint16_t release_threshold[] = {
   450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450,
   450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450,
   450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450,
   450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450, 450,
   450, 450, 450, 450, 450,
 };
 // clang-format on
 
 struct kscan_ec_data {
   kscan_callback_t callback;
   uint16_t poll_interval;
   struct k_work_delayable poll;
   bool *matrix_state;
   const struct device *dev;
 };
 
 struct kscan_ec_config {
   struct kscan_gpio_list row_gpios;
   struct kscan_gpio_list mux_sels;
   struct kscan_gpio power;
   struct kscan_gpio mux0_en; //mux enable GPIO pin
   struct kscan_gpio mux1_en; //mux enable GPIO pin
   struct kscan_gpio discharge;
   struct adc_dt_spec adc_channel;
 
   size_t rows;
   size_t cols;
 
   const uint16_t matrix_warm_up_ms;
   const uint16_t matrix_relax_us;
   const uint16_t adc_read_settle_us;
   const uint16_t active_polling_interval_ms;
   const uint16_t idle_polling_interval_ms;
   const uint16_t sleep_polling_interval_ms;

   const uint16_t col_channels[];
   const uint32_t row_input_masks[];
   
 };
 
 /**
  * Get the index of a matrix state array from a row and column.
  */
 static int state_index_rc(const struct kscan_ec_config *config, const int row,
                           const int col) {
   __ASSERT(row < config->rows, "Invalid row %i", row);
   __ASSERT(col < config->cols, "Invalid col %i", row);
   /* offset by column length, if row=1, col_len=14, offset= 1x14 */
   return (row * config->cols) + col;
 }
 
 static int kscan_ec_configure(const struct device *dev,
                               const kscan_callback_t callback) {
   LOG_DBG("KSCAN EC configure");
 
   struct kscan_ec_data *data = dev->data;
    if (!callback)
    {
        return -EINVAL;
    }
    data->callback = callback;
    LOG_DBG("Configured KSCAN");
    return 0;
 }
 
 static int kscan_ec_enable(const struct device *dev) {
   LOG_DBG("KSCAN EC enable");
 
   struct kscan_ec_data *data = dev->data;
   const struct kscan_ec_config *config = dev->config;
 
   k_work_schedule(&data->poll, K_MSEC(data->poll_interval));
   return 0;
 
   return 0;
 }
 
 static int kscan_ec_disable(const struct device *dev) {
  LOG_DBG("KSCAN API disable");
  struct kscan_ec_data *data = dev->data;
  k_work_cancel_delayable(&data->poll);
  return 0;
 }
 
 static void kscan_ec_work_handler(struct k_work *work) {

   struct k_work_delayable *d_work = k_work_delayable_from_work(work);
   struct kscan_ec_data *data = CONTAINER_OF(d_work, struct kscan_ec_data, poll);
   const struct kscan_ec_config *config = data->dev->config;
   const struct device *dev = data->dev;
 
   int rc;
 
   int16_t matrix_read[config->rows * config->cols];
 
   /* power on everything */
   gpio_pin_set_dt(&config->power.spec, 1);

   for (int i = 0; i < config->row_gpios.len; i++) {
    gpio_pin_set_dt(&config->row_gpios.gpios[i].spec, 1);
  }

  for (int i = 0; i < config->mux_sels.len; i++) {
    gpio_pin_set_dt(&config->mux_sels.gpios[i].spec, 1);
  }
 
   // The board needs some time to be operational after powering up
   k_sleep(K_MSEC(cfg->matrix_warm_up_ms));

   for (int col = 0; col < config->cols; col++) {
     uint16_t ch = config->col_channels[col];
     // activate mux based on column index (e.g., first 8 columns use mux1_en)
     if (col < 8){
      // momentarily disable current multiplexers
      gpio_pin_set_dt(&config->mux0_en.spec, 0);
      gpio_pin_set_dt(&config->mux1_en.spec, 0);
      /* MUX channel select */
      gpio_pin_set_dt(&config->mux_sels.gpios[0].spec, ch & (1 << 0));
      gpio_pin_set_dt(&config->mux_sels.gpios[1].spec, ch & (1 << 1));
      gpio_pin_set_dt(&config->mux_sels.gpios[2].spec, ch & (1 << 2));
      gpio_pin_set_dt(&config->mux0_en.spec, 1);
     } else{
      // momentarily disable current multiplexers
      gpio_pin_set_dt(&config->mux1_en.spec, 0);
      gpio_pin_set_dt(&config->mux0_en.spec, 0);
      /* MUX channel select */
      gpio_pin_set_dt(&config->mux_sels.gpios[0].spec, ch & (1 << 0));
      gpio_pin_set_dt(&config->mux_sels.gpios[1].spec, ch & (1 << 1));
      gpio_pin_set_dt(&config->mux_sels.gpios[2].spec, ch & (1 << 2));
      gpio_pin_set_dt(&config->mux1_en.spec, 1);
     }
     
     for (int row = 0; row < config->rows; row++) {
       
       /* check if it is masked for this row col, skip it if yes */
       /* if (config->row_input_masks && (config->row_input_masks[row] & BIT(col)) != 0) {} */
       
       const int index = state_index_rc(config, row, col);
       /* disable all rows */
       for (int row = 0; row < config->rows; row++) {
         gpio_pin_set_dt(&config->row_gpios.gpios[row].spec, 0);
       }
       
       // --- LOCK ---
       const unsigned int lock = irq_lock();
       // have capacitor charged and set the row pin to high state
       gpio_pin_configure_dt(&config->discharge.spec, GPIO_INPUT);
       gpio_pin_set_dt(&config->row_gpios.gpios[row].spec, 1);  // enable current row
 
       // WAIT_CHARGE(); try disabling this line
 
       rc = adc_read(config->adc_channel.dev, adc_seq);
       adc_seq->calibrate = false;
 
       if (rc == 0) {
         matrix_read[index] = data->adc_raw;
         /* Handle matrix reads */
         const bool pressed = data->matrix_state[index];
         if (!pressed && matrix_read[index] > actuation_threshold[index]) {
            data->matrix_state[index] = true;
            /* add console output here */
            data->callback(data->dev, row, col, true);
          } else if (pressed && matrix_read[index] < release_threshold[index]) {
            data->matrix_state[index] = false;
            data->callback(data->dev, row, col, false);
          }
       } else {
         LOG_ERR("Failed to read ADC: %d", rc);
         matrix_read[index] = -1;
       }
       irq_unlock(lock);
       // -- END LOCK --
 
       gpio_pin_set_dt(&config->discharge.spec, 0);
       gpio_pin_configure_dt(&config->discharge.spec, GPIO_OUTPUT);
       WAIT_DISCHARGE();
    }
  }
 
   /* Power off */
   gpio_pin_set_dt(&config->power.spec, 0);
 
   for (int i = 0; i < config->row_gpios.len; i++) {
     gpio_pin_set_dt(&config->row_gpios.gpios[i].spec, 0);
   }
 
   for (int i = 0; i < config->mux_sels.len; i++) {
     gpio_pin_set_dt(&config->mux_sels.gpios[i].spec, 0);
   }
 
   /* Print matrix reads, comment it first */
   /* static int cnt = 0;
    * if (cnt++ >= (300 / config->poll_period_ms)) {
    * cnt = 0;
    * 
    * for (int r = 0; r < config->rows; r++) {
    *   for (int c = 0; c < config->cols; c++) {
    *     const int index = state_index_rc(config, r, c);
    *     printk("%6d", matrix_read[index]);
    *     if (c < config->cols - 1) {
    *       printk(",");
    *     }
    *   }
    *   printk("\n");
    * }
    * printk("\n\n");
    * }
    */
 
 }
 
 static int kscan_ec_init(const struct device *dev) {
    LOG_DBG("KSCAN EC init");
  
    struct kscan_ec_data *data = dev->data;
    const struct kscan_ec_config *config = dev->config;
  
    int rc = 0;
  
    LOG_WRN("EC Channel: %d", config->adc_channel.channel_cfg.channel_id);
    LOG_WRN("EC Channel 2: %d", config->adc_channel.channel_id);
  
    gpio_pin_configure_dt(&config->power.spec, GPIO_OUTPUT_INACTIVE);
  
    data->dev = dev;
  
    data->adc_seq = (struct adc_sequence){
        .buffer = &data->adc_raw,
        .buffer_size = sizeof(data->adc_raw),
    };
  
    rc = adc_channel_setup_dt(&config->adc_channel);
    if (rc < 0) {
      LOG_ERR("ADC channel setup error %d", rc);
    }
  
    rc = adc_sequence_init_dt(&config->adc_channel, &data->adc_seq);
    if (rc < 0) {
      LOG_ERR("ADC sequence init error %d", rc);
    }
  
    gpio_pin_configure_dt(&config->discharge.spec, GPIO_OUTPUT_INACTIVE);
  
    // Init rows
    for (int i = 0; i < config->row_gpios.len; i++) {
      gpio_pin_configure_dt(&config->row_gpios.gpios[i].spec, GPIO_OUTPUT_INACTIVE);
    }
  
    // Init mux sel
    for (int i = 0; i < config->mux_sels.len; i++) {
      gpio_pin_configure_dt(&config->mux_sels.gpios[i].spec,
                            GPIO_OUTPUT_INACTIVE);
    }
  
    // Init both muxes
    gpio_pin_set_dt(&config->mux0_en.spec, GPIO_OUTPUT_INACTIVE);
    gpio_pin_set_dt(&config->mux1_en.spec, GPIO_OUTPUT_INACTIVE);
  
    data->poll_interval = cfg->active_polling_interval_ms;
    k_work_init_delayable(&data->poll, kscan_ec_timer_handler);
    k_timer_init(&data->work_timer, kscan_ec_timer_handler, NULL);
    k_work_init(&data->work, kscan_ec_work_handler);
  
    return 0;
 }
 
 static int kscan_ec_activity_event_handler(const struct device *dev,
                                            const zmk_event_t *eh) {
   struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
 
   if (ev == NULL) {
     return -ENOTSUP;
   }
 
   struct kscan_ec_data *data = dev->data;
   const struct kscan_ec_config *config = dev->config;
 
   uint16_t poll_period;
 
   switch (ev->state) {
   case ZMK_ACTIVITY_ACTIVE:
     poll_period = config->active_polling_interval_ms;
     break;
   case ZMK_ACTIVITY_IDLE:
     poll_period = config->idle_polling_interval_ms;
     break;
   case ZMK_ACTIVITY_SLEEP:
     poll_period = config->sleep_polling_interval_ms;
     break;
   default:
     LOG_WRN("Unsupported activity state: %d", ev->state);
     return -EINVAL;
   }
 
   LOG_DBG("Setting poll period to %dms", poll_period);
   k_timer_start(&data->work_timer, K_MSEC(poll_period), K_MSEC(poll_period));
 
   return 0;
 }
 
 static const struct kscan_driver_api kscan_ec_api = {
     .config = kscan_ec_configure,
     .enable_callback = kscan_ec_enable,
     .disable_callback = kscan_ec_disable};
 
 #define KSCAN_EC_INIT(n)                                                       \
   static struct kscan_gpio kscan_ec_row_gpios_##n[] = {                        \
       LISTIFY(INST_ROWS_LEN(n), KSCAN_GPIO_ROW_CFG_INIT, (, ), n)};            \
   static struct kscan_gpio kscan_ec_mux_sel_gpios_##n[] = {                    \
       LISTIFY(INST_MUX_SELS_LEN(n), KSCAN_GPIO_MUX_SEL_CFG_INIT, (, ), n)};    \
                                                                                \
   static bool kscan_ec_matrix_state_##n[INST_MATRIX_LEN(n)];                   \
                                                                                \
   static struct kscan_ec_data kscan_ec_data_##n = {                            \
       .matrix_state = kscan_ec_matrix_state_##n,                               \
   };                                                                           \
                                                                                \
   static struct kscan_ec_config kscan_ec_config_##n = {                        \
       .row_gpios = KSCAN_GPIO_LIST(kscan_ec_row_gpios_##n),                       \
       .mux_sels = KSCAN_GPIO_LIST(kscan_ec_mux_sel_gpios_##n),                 \
       .power = KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(n), power_gpios, 0),          \
       .mux0_en = KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(n), mux0_en_gpios, 0),      \
       .mux1_en = KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(n), mux1_en_gpios, 0),      \
       .row-input-masks = DT_INST_PROP(n, row-input-masks),                      \
       .discharge = KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(n), discharge_gpios, 0),  \
       .matrix_warm_up_ms = DT_INST_PROP(n, matrix_warm_up_ms),                 \
       .matrix_relax_us = DT_INST_PROP(n, matrix_relax_us),                     \
       .adc_read_settle_us = DT_INST_PROP(n, adc_read_settle_us),               \
       .active_polling_interval_ms = DT_INST_PROP(n, active_polling_interval_ms),      \
       .idle_polling_interval_ms = DT_INST_PROP(n, idle_polling_interval_ms),   \
       .sleep_polling_interval_ms = DT_INST_PROP(n, sleep_polling_interval_ms), \
       .col_channels = DT_INST_PROP(n, col_channels),                           \
       .rows = INST_ROWS_LEN(n),                                                \
       .cols = INST_COL_CHANNELS_LEN(n),                                        \
       .adc_channel = ADC_DT_SPEC_INST_GET(n),                                  \
   };                                                                           \
   static int kscan_ec_activity_event_handler_wrapper##n(                       \
       const zmk_event_t *eh) {                                                 \
     const struct device *dev = DEVICE_DT_INST_GET(n);                          \
     return kscan_ec_activity_event_handler(dev, eh);                           \
   }                                                                            \
   ZMK_LISTENER(kscan_ec##n, kscan_ec_activity_event_handler_wrapper##n);       \
   ZMK_SUBSCRIPTION(kscan_ec##n, zmk_activity_state_changed);                   \
                                                                                \
   DEVICE_DT_INST_DEFINE(n, &kscan_ec_init, NULL, &kscan_ec_data_##n,           \
                         &kscan_ec_config_##n, POST_KERNEL,                     \
                         CONFIG_KSCAN_INIT_PRIORITY, &kscan_ec_api);
 
 DT_INST_FOREACH_STATUS_OKAY(KSCAN_EC_INIT);