#pragma once
#include <stdint.h>
#include <SPIFFS.h>

#define MAX_CONFIGS 10
#define CONFIG_FILE "/fanatec_configs.bin"

typedef struct {
    char configname[10];
    int config_number;
    uint8_t wheel_ident;
    uint8_t transpose_array[4]; // 32-bit (4 byte) transpose mapping
} FanatecConfig;

// Store/retrieve configuration with 4-byte transpose arrays
void store_config(const char name[10], int config_num, uint8_t wheel_id, const uint8_t transpose[4]);
bool retrieve_config(int config_num, char name_out[10], uint8_t* wheel_id, uint8_t transpose_out[4]);

// Active config management
bool store_active_config(int active_config);
int retrieve_active_config();

// Transpose function with 32-element mapping
void transposeBits(const uint8_t data[4], uint8_t dataout[4], const uint8_t transpose[32]);

// Initialization
void initialize_default_configs();
