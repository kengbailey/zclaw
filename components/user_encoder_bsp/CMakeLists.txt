idf_component_register(
  SRCS "src/bidi_switch_knob.c" "user_encoder_bsp.c"
  REQUIRES driver esp_timer
  INCLUDE_DIRS "./" "./src")
