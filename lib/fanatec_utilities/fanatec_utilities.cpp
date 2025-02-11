#include "fanatec_utilities.h"
#include <Arduino.h>
#include <string.h>
#include <SPIFFS.h>

// Store active config number at end of configs array (offset 180 = 10 configs * 18 bytes each)
bool store_active_config(int active_config) {
    File file = SPIFFS.open(CONFIG_FILE, "r+");
    if (!file) {
        file = SPIFFS.open(CONFIG_FILE, "w+");
    }
    
    if (file) {
        file.seek(MAX_CONFIGS * sizeof(FanatecConfig));
        file.write((uint8_t*)&active_config, sizeof(active_config));
        file.close();
        return true;
    }
    return false;
}

int retrieve_active_config() {
    File file = SPIFFS.open(CONFIG_FILE, "r");
    int active_config = 0;
    if (file) {
        file.seek(MAX_CONFIGS * sizeof(FanatecConfig));
        if(file.available() >= sizeof(active_config)) {
            file.read((uint8_t*)&active_config, sizeof(active_config));
        }
        file.close();
    }
    return active_config;
}

void store_config(const char name[10], int config_num, uint8_t wheel_id, const uint8_t transpose[3]) {
    FanatecConfig new_config;
    memcpy(new_config.configname, name, 10);
    new_config.config_number = config_num;
    new_config.wheel_ident = wheel_id;
    memcpy(new_config.transpose_array, transpose, 3);

    File file = SPIFFS.open(CONFIG_FILE, "r+");
    if (!file) {
        file = SPIFFS.open(CONFIG_FILE, "w+");
    }
    
    if (file) {
        FanatecConfig configs[MAX_CONFIGS];
        file.read((uint8_t*)configs, sizeof(configs));
        
        // Update existing or find empty slot
        bool updated = false;
        for (int i = 0; i < MAX_CONFIGS; i++) {
            if (configs[i].config_number == config_num || configs[i].config_number == 0) {
                configs[i] = new_config;
                updated = true;
                break;
            }
        }
        
        if (updated) {
            file.seek(0);
            file.write((uint8_t*)configs, sizeof(configs));
        }
        file.close();
    }
}

bool retrieve_config(int config_num, char name_out[10], uint8_t* wheel_id, uint8_t transpose_out[3]) {
    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (file) {
        FanatecConfig configs[MAX_CONFIGS];
        file.read((uint8_t*)configs, sizeof(configs));
        file.close();
        
        for (int i = 0; i < MAX_CONFIGS; i++) {
            if (configs[i].config_number == config_num) {
                memcpy(name_out, configs[i].configname, 10);
                *wheel_id = configs[i].wheel_ident;
                memcpy(transpose_out, configs[i].transpose_array, 3);
                return true;
            }
        }
    }
    return false;
}

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
