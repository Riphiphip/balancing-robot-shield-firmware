cmake_minimum_required(VERSION 3.16.0)

if(CONFIG_QDEC_GPIO)
target_sources(app PRIVATE qdec_gpio.c)
endif()