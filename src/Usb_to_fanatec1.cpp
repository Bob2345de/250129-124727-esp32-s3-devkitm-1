#include <Arduino.h>
       /*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "fanatec_utilities.h"
#include "hid_host.h"

/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN                GPIO_NUM_0

static const char *TAG = "example";
QueueHandle_t hid_host_event_queue;
bool user_shutdown = false;
int length2 =2;
const uint8_t transpose[24] = {
  23, 22, 21, 20, 19, 18, 17, 16,
  15, 14, 13, 12, 11, 10, 9, 8,
  7, 6, 5, 4, 3, 2, 1, 0
};



/**
 * @brief HID Host event
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
  hid_host_device_handle_t hid_device_handle;
  hid_host_driver_event_t event;
  void *arg;
} hid_host_event_queue_t;




/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {"NONE"};


/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto) {
  static hid_protocol_t prev_proto_output = HID_PROTOCOL_MAX;

  if (prev_proto_output != proto) {
    prev_proto_output = proto;
    printf("\r\n");
    {
      printf("Generic\r\n");
    }
    fflush(stdout);
  }
}


/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src, uint8_t key,
                             unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    if (src[i] == key) {
      return true;
    }
  }
  return false;
}


/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data,
                                             const int length) {
  hid_print_new_device_report_header(HID_PROTOCOL_NONE);
  //for (int i = 0; i < min(10, length); i++) {
    for (int i = 0; i < min(10, length); i++) {
      printf("%02X", data[i]);
    }
    /* printf("byte1");
    printf("%02X", data[0]);
    printf("byte2");
    printf("%02X", data[1]);
    printf("byte3");
    printf("%02X", data[2]);

  
     */
  printf("%d\n", length);
  putchar('|');
  

  switch (length) {
    // This assumes the HID is a Thrustmaster T16000M flight joystick
    case 9:
      {
        typedef struct __attribute__((packed)) {
          uint16_t buttons;
          uint8_t hat;
          uint16_t x;
          uint16_t y;
          uint8_t z;  // Twist
          uint8_t throttle;
        } t16k_t;

        const t16k_t *const joy = (const t16k_t *const)data;
        printf("buttons: %04x hat: %2x X: %04x Y: %04x Z: %02x throttle: %02x",
            joy->buttons, joy->hat, joy->x, joy->y, joy->z, joy->throttle);
      }
      break;
    case 7:
      // This assumes the HID is a Logitech Extreme 3D Pro flight joystick
      {
        typedef struct __attribute__((packed)) {
          uint32_t x : 10;
          uint32_t y : 10;
          uint32_t hat : 4;
          uint32_t z : 8;
          uint8_t buttons_a;
          uint8_t throttle;
          uint8_t buttons_b;
        } le3dp_t;

        const le3dp_t *const joy = (const le3dp_t *const)data;
        printf("X: %04x Y: %04x hat: %x Z: %02x buttons_a: %02x throttle: %02x buttons_a: %02x",
            joy->x, joy->y, joy->hat, joy->z, joy->buttons_a, joy->throttle,
            joy->buttons_b);
      }
      break;
    case 64:
      {
        // Assume the HID is a Sony Dual Shock 4 PlayStation 4 game controller
        // 14 Buttons, 6 Axes, 1 D-Pad
        typedef struct __attribute__((packed)) {
          uint8_t ReportID;   // always 0x01
          uint8_t leftXAxis;
          uint8_t leftYAxis;
          uint8_t rightXAxis;
          uint8_t rightYAxis;
          uint8_t dPad:4;     // dpad[3-0]
          uint8_t button1:4;  // Triangle[7], circle[6], cross[5], square[4]
          uint8_t button2;    // R3:7,L3:6,Options:5,share:4,R2:3,L2:2,R1:1,L1:0
          uint8_t button3:2;  // tpad click[1], logo[0]
          uint8_t reportCnt:6;
          uint8_t L2Axis;
          uint8_t R2Axis;
          uint16_t timestamp; // increment by 188 per report
          uint8_t batteryLvl;
          uint16_t gyroX;
          uint16_t gyroY;
          uint16_t gyroZ;
          int16_t accelX;
          int16_t accelY;
          int16_t accelZ;
          uint8_t filler[39];
        } DS4GamepadReport_t;
        const DS4GamepadReport_t *const ds4 =
          (const DS4GamepadReport_t *const)data;
        printf("Left X,Y: %3u,%3u Right X,Y: %3u,%3u "
            "DPad: %x Button1: 0x%x Button2: 0x%02x Button3: 0x%x "
            "Throttle Left,Right: %3u,%3u",
            ds4->leftXAxis, ds4->leftYAxis, ds4->rightXAxis, ds4->rightYAxis,
            ds4->dPad, ds4->button1, ds4->button2, ds4->button3,
            ds4->L2Axis, ds4->R2Axis);
        break;
      }
    default:
      break;
  }
  printf("\r\n");
  fflush(stdout);
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg) {
  uint8_t data[64] = {0};
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
        hid_device_handle, data, 64, &data_length));

    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      // Boot interface devices not supported after removing keyboard/mouse
    } else {
      hid_host_generic_report_callback(data, data_length);
    }

    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
             hid_proto_name_str[dev_params.proto]);
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    ESP_LOGI(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
             hid_proto_name_str[dev_params.proto]);
    break;
  default:
    ESP_LOGE(TAG, "HID Device, protocol '%s' Unhandled event",
             hid_proto_name_str[dev_params.proto]);
    break;
  }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event, void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
  const hid_host_device_config_t dev_config = {
      .callback = hid_host_interface_callback, .callback_arg = NULL};


  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    ESP_LOGI(TAG, "HID Device, protocol '%s' CONNECTED",
             hid_proto_name_str[dev_params.proto]);

    ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle,
                                                     HID_REPORT_PROTOCOL_BOOT));
    }
    ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
    break;
  default:
    break;
  }
}

/**
 * @brief Start USB Host install and handle common USB host library events while
 * app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg) {
  const gpio_config_t input_pin = {
      .pin_bit_mask = BIT64(APP_QUIT_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&input_pin));

  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive((TaskHandle_t)arg);

  while (gpio_get_level(APP_QUIT_PIN) != 0) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

    // Release devices once all clients has deregistered
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      usb_host_device_free_all();
      ESP_LOGI(TAG, "USB Event flags: NO_CLIENTS");
    }
    // All devices were removed
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      ESP_LOGI(TAG, "USB Event flags: ALL_FREE");
    }
  }
  // App Button was pressed, trigger the flag
  user_shutdown = true;
  ESP_LOGI(TAG, "USB shutdown");
  // Clean up USB Host
  vTaskDelay(10); // Short delay to allow clients clean-up
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}

/**
 * @brief HID Host main task
 *
 * Creates queue and get new event from the queue
 *
 * @param[in] pvParameters Not used
 */
void hid_host_task(void *pvParameters) {
  hid_host_event_queue_t evt_queue;
  // Create queue
  hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));

  // Wait queue
  while (!user_shutdown) {
    if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50))) {
      hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event,
                            evt_queue.arg);
    }
  }

  xQueueReset(hid_host_event_queue);
  vQueueDelete(hid_host_event_queue);
  vTaskDelete(NULL);
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event, void *arg) {
  const hid_host_event_queue_t evt_queue = {
      .hid_device_handle = hid_device_handle, .event = event, .arg = arg};
  xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

void setup (void) {
  BaseType_t task_created;
  ESP_LOGI(TAG, "HID Joystick");

  /*
   * Create usb_lib_task to:
   * - initialize USB Host library
   * - Handle USB Host events while APP pin in in HIGH state
   */
  task_created =
      xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                              xTaskGetCurrentTaskHandle(), 2, NULL, 0);
  assert(task_created == pdTRUE);

  // Wait for notification from usb_lib_task to proceed
  ulTaskNotifyTake(false, 1000);

  /*
   * HID host driver configuration
   * - create background task for handling low level event inside the HID driver
   * - provide the device callback to get new HID Device connection event
   */
  const hid_host_driver_config_t hid_host_driver_config = {
      .create_background_task = true,
      .task_priority = 5,
      .stack_size = 4096,
      .core_id = 0,
      .callback = hid_host_device_callback,
      .callback_arg = NULL};

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Task is working until the devices are gone (while 'user_shutdown' if false)
  user_shutdown = false;

  /*
   * Create HID Host task process for handle events
   * IMPORTANT: Task is necessary here while there is no possibility to interact
   * with USB device from the callback.
   */
  task_created =
      xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
  assert(task_created == pdTRUE);
}

void loop() {}
