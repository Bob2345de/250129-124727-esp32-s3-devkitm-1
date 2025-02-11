#pragma once
#include <stdint.h>
#include <SPIFFS.h>

#define MAX_CONFIGS 10
#define CONFIG_FILE "/fanatec_configs.bin"

typedef struct FanatecConfig {
    char configname[10];
    int config_number;
    uint8_t wheel_ident;
    uint8_t transpose_array[3];
} FanatecConfig;

void store_config(const char name[10], int config_num, uint8_t wheel_id, const uint8_t transpose[3]);
bool retrieve_config(int config_num, char name_out[10], uint8_t* wheel_id, uint8_t transpose_out[3]);
bool store_active_config(int active_config);
int retrieve_active_config();
void initialize_default_configs();
void transposeBits(const uint8_t data[3], uint8_t dataout[3], const uint8_t transpose[24]);
