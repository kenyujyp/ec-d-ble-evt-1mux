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
 
 #define INST_MATRIX_LEN(n) (INST_ROWS_LEN(n) * INST_COL_CHANNELS_LEN(n))
 
 // clang-format off
 
 const uint16_t actuation_threshold[] = {
   800, 800, 800, 800,
   800, 800, 800, 800
 };
 
 const uint16_t release_threshold[] = {
   600, 600, 600, 600,
   600, 600, 600, 600
 };

 uint16_t noise_floor[] = {
  500, 500, 500, 500,
  500, 500, 500, 500
};
 // clang-format on
 
 struct kscan_ec_data {
  const struct device *dev;
  struct adc_sequence adc_seq;
  uint16_t adc_raw;   // could be 

  struct k_work work;
  struct k_timer work_timer;
  kscan_callback_t callback;
  bool *matrix_state;
};

 struct kscan_ec_config {
   struct gpio_dt_spec* row_gpios;
   struct gpio_dt_spec* mux_sels;
   struct gpio_dt_spec mux0_en; //mux enable GPIO pin
   struct gpio_dt_spec discharge;
   struct adc_dt_spec adc_channel;
 
   size_t rows;
   size_t cols;
 
   const uint16_t matrix_warm_up_ms;
   const uint16_t matrix_relax_us;
   const uint16_t active_polling_interval_ms;
   const uint16_t idle_polling_interval_ms;

   const uint32_t* row_input_masks;
   uint16_t col_channels[];
   
 };
 
 /*
  * Get the index of a matrix state array from a row and column.
  */
 static int state_index_rc(const struct kscan_ec_config *config, const int row,
                           const int col) {
   __ASSERT(row < config->rows, "Invalid row %i", row);
   __ASSERT(col < config->cols, "Invalid col %i", row);
   /* offset by column length, if row=1, col_len=15, offset= 1x15 */
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

  k_timer_start(&data->work_timer, K_MSEC(config->active_polling_interval_ms),
                K_MSEC(config->active_polling_interval_ms));

  return 0;
}

static int kscan_ec_disable(const struct device *dev) {
  LOG_DBG("KSCAN EC disable");

  struct kscan_ec_data *data = dev->data;
  k_timer_stop(&data->work_timer);

  return 0;
}

static void kscan_ec_timer_handler(struct k_timer *timer) {
  struct kscan_ec_data *data =
      CONTAINER_OF(timer, struct kscan_ec_data, work_timer);
  k_work_submit(&data->work);
}
 
static void kscan_ec_work_handler(struct k_work *work) {

  struct kscan_ec_data *data = CONTAINER_OF(work, struct kscan_ec_data, work);
  const struct kscan_ec_config *config = data->dev->config;
  struct adc_sequence *adc_seq = &data->adc_seq;

  /* adc read status, first check int type */
  int rc;
  uint16_t matrix_read = 0;
 
  /* power on everything */
  // gpio_pin_set_dt(&config->power.spec, 1);
 
  // The board needs some time to be operational after powering up
  k_sleep(K_MSEC(config->matrix_warm_up_ms));

  for (int col = 0; col < config->cols; col++) {
    uint8_t ch = config->col_channels[col];

    /* disable all rows */
    for (int r = 0; r < config->rows; r++){
      gpio_pin_set_dt(&config->row_gpios[r], 0);   // write pin low 
    }
     
    for (int row = 0; row < config->rows; row++) {
      /* check if it is masked for this row col, skip it if yes */
      if (config->row_input_masks && (config->row_input_masks[row] & (1 << col)) != 0) {
        continue;
      }

      gpio_pin_set_dt(&config->row_gpios[row], 0);   // disable current row

      /* disable both multiplexers, mux output is disabled when enable pin is high */
      gpio_pin_set_dt(&config->mux0_en, 0);  // drive high, active low
      //gpio_pin_configure_dt(&config->mux0_en, GPIO_INPUT);

      /* multiplexer channel select */
      for (uint8_t i = 0; i < 3; i++) {
        gpio_pin_set_dt(&config->mux_sels[i], (ch >> i) & 1);
      }

      /* adjusted position index in matrix */
      const uint16_t index = state_index_rc(config, row, col);

      // --- LOCK ---
      const uint32_t lock = irq_lock();
      // charge phase
      // set discharge pin to high impedance
      gpio_pin_configure_dt(&config->discharge, GPIO_INPUT);
      gpio_pin_set_dt(&config->discharge, 1);   // !!!! 2 or 1
      k_busy_wait(1);  // Ensure discharge is off

      // set current row pin high
      gpio_pin_set_dt(&config->row_gpios[row], 1);
      // wait for charge, 5us, need to define!!
      k_busy_wait(5);
      // reenable mux_0
      gpio_pin_set_dt(&config->mux0_en, 1);    // drive low, active low

      /* read adc */
      rc = adc_read(config->adc_channel.dev, adc_seq);
      //adc_seq->calibrate = false;

      irq_unlock(lock);
      // -- END LOCK --
      /* drive current row low */
      gpio_pin_set_dt(&config->row_gpios[row], 0);
      /* pull low discharge pin and configure pin to output to drain external circuit */
      gpio_pin_configure_dt(&config->discharge, GPIO_OUTPUT);
      gpio_pin_set_dt(&config->discharge, 0);   // drive low, active high


      /* handle matrix reads */
      const bool pressed = data->matrix_state[index];
      // shift 4 for correct range value
      if (rc == 0) {
        matrix_read = (data->adc_raw >> 4);
      } else {
        LOG_ERR("Failed to read ADC: %d", rc);
      }
      /* print reading for debugging, uncomment in final build */
      printk("reading: %d, %d, %u, %u\n", row, col, matrix_read, noise_floor[index]);
      
      /* real time noise floor calibration */
      if (matrix_read < noise_floor[index]) {
        /* update noise floor */
        noise_floor[index] = matrix_read;
        /* quick-and-dirty debugging, added more space to frimware size (remove before final build) */
        printk("noise floor changed: %d, %d, %u\n", row, col, matrix_read);
      }

      if (!pressed && matrix_read > (actuation_threshold[index] + noise_floor[index])) {
        /* key pressed */
        data->matrix_state[index] = true;
        /* quick-and-dirty debugging, added more space to frimware size (remove before final build) */
        printk("key pressed: %d, %d, %u, %u\n", row, col, matrix_read, noise_floor[index]);
        /* uncommment next line for final build */
        //data->callback(data->dev, row, col, true);
      } else if (pressed && matrix_read < (release_threshold[index] + noise_floor[index])) {
        /* key is released */
        data->matrix_state[index] = false;
        /* quick-and-dirty debugging, added more space to frimware size (remove before final build) */
        printk("key released: %d, %d, %u, %u\n", row, col, matrix_read, noise_floor[index]);
        /* uncommment next line for final build */
        //data->callback(data->dev, row, col, false);
      }
      
      // wait for discharge, 10us
      k_busy_wait(10);
    }
  }
}
 
 static int kscan_ec_init(const struct device *dev) {
    LOG_DBG("KSCAN EC init");
  
    struct kscan_ec_data *data = dev->data;
    const struct kscan_ec_config *config = dev->config;
  
    int rc = 0;
  
    LOG_WRN("EC Channel: %d", config->adc_channel.channel_cfg.channel_id);
    LOG_WRN("EC Channel 2: %d", config->adc_channel.channel_id);
  
    //gpio_pin_configure_dt(&config->power.spec, GPIO_OUTPUT_INACTIVE);
  
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
  
    gpio_pin_configure_dt(&config->discharge, GPIO_OUTPUT);
  
    // Init rows
    for (int i = 0; i < config->rows; i++) {
      gpio_pin_configure_dt(&config->row_gpios[i], GPIO_OUTPUT);
    }
  
    // Init mux sel
    for (int i = 0; i < 3; i++) {
      gpio_pin_configure_dt(&config->mux_sels[i],
                            GPIO_OUTPUT);
    }
  
    // Init both muxes
    gpio_pin_set_dt(&config->mux0_en, GPIO_OUTPUT);
  
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
     /* COMPLETELY STOP POLLING IN SLEEP STATE */
     k_timer_stop(&data->work_timer);  // Stop any existing timer
     return 0;  // Exit early without restarting timer
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
     .disable_callback = kscan_ec_disable
 };
 
 #define ZKEM_GPIO_DT_SPEC_ELEM(n, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(n, prop, idx),

 #define KSCAN_EC_INIT(n)                                                       \
                                                                                \
   static bool kscan_ec_matrix_state_##n[INST_MATRIX_LEN(n)];                   \
                                                                                \
   /* pointer point to address of first element in array */                     \
   static struct kscan_ec_data kscan_ec_data_##n = {                            \
       .matrix_state = kscan_ec_matrix_state_##n,                               \
   };                                                                           \
                                                                                \
   COND_CODE_1(                                                                 \
    DT_INST_NODE_HAS_PROP(n, row_input_masks),                                  \
    (static const uint32_t row_input_masks_##n[] = DT_INST_PROP(n, row_input_masks);),   \
    ())                                                                         \
                                                                                \
   static const struct gpio_dt_spec row_gpios_##n[] = {                         \
      DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), row_gpios, ZKEM_GPIO_DT_SPEC_ELEM)}; \
   static const struct gpio_dt_spec mux_sels_##n[] = {                         \
      DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), mux_sel_gpios, ZKEM_GPIO_DT_SPEC_ELEM)}; \
                                                                                \
   static struct kscan_ec_config kscan_ec_config_##n = {                        \
       .row_gpios = row_gpios_##n,                                              \
       .mux_sels = mux_sels_##n,                                                \
       .mux0_en = GPIO_DT_SPEC_INST_GET_OR(n, mux0_en_gpios, {0}),              \
       COND_CODE_1(DT_INST_NODE_HAS_PROP(n, row_input_masks),                   \
                    (.row_input_masks = row_input_masks_##n, ), ())             \
       .discharge = GPIO_DT_SPEC_INST_GET_OR(n, discharge_gpios, {0}),          \
       .matrix_warm_up_ms = DT_INST_PROP(n, matrix_warm_up_ms),                 \
       .matrix_relax_us = DT_INST_PROP(n, matrix_relax_us),                     \
       .active_polling_interval_ms = DT_INST_PROP(n, active_polling_interval_ms),      \
       .idle_polling_interval_ms = DT_INST_PROP(n, idle_polling_interval_ms),   \
       .col_channels = DT_INST_PROP(n, col_channels),                           \
       .rows = DT_INST_PROP_LEN(n, row_gpios),                                  \
       .cols = DT_INST_PROP_LEN(n, col_channels),                               \
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