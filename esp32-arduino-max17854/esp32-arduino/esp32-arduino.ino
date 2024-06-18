/*

  Todo:
        PEC validation       ✅
        Read balance shunts  ✅
        Manual balance       ✅
        Auto balance         ❌
        Slave temperatures   ❌
        Long reads           ❌

*/

#include "esp32-arduino.h"
#include "arduino.h"
#include "driver/twai.h"
#include <freertos/task.h>

QueueHandle_t tx_queue, rx_queue;
TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t twaiTaskHandle = NULL;
TaskHandle_t twaiprocessTaskHandle = NULL;

BMS_Data maxim_data, inverter_data;
CanBus can_bus;
bool can_tx_stop = false;
static const unsigned long starttime = millis();  // uptime counter
static unsigned long contactor_time = 0;

unsigned long previousMillis250 = 0;
unsigned long previousMillis1000 = 0;

const long interval250 = 250;    // Minimum balance shunt duration
const long interval1000 = 1000;  //

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  // Look in the api-reference for other speed sets.
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static bool driver_installed = false;

void setup() {
  digitalWrite(SHDNL_MAX17841_1, LOW);  // reset MAX17841b
  initialisation.Arduino_SPI_init();
  Serial.println("START");

  modules = initialisation.MAX178X_init_sequence(CELLS_PER_SLAVE);
  maxim.init(modules, &maxim_data);
  xTaskCreatePinnedToCore(SPI_Task, "SPI_Task", 4096, NULL, 1, &spiTaskHandle, 0);

  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.print("Waiting for good data");
  while (1) {
    Serial.print(".");
    if (!has_errors(&inverter_data)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  };
  Serial.println("Maxim interface -> Inverter data readings OK - Starting CAN");

  // esp_task_wdt_init(&twdt_config);
  // esp_task_wdt_reset();

  // queue

  rx_queue = xQueueCreate(32, sizeof(twai_message_t));
  tx_queue = xQueueCreate(32, sizeof(twai_message_t));
  xTaskCreatePinnedToCore(TWAI_Task, "TWAI_Task", 4096, NULL, 6, &twaiTaskHandle, 0);
  xTaskCreatePinnedToCore(TWAI_Processing_Task, "TWAI_Processing_Task", 4096, NULL, 4, &twaiprocessTaskHandle, 0);
}
void interrupt_pin() {
  Serial.println("******************** INT LOW ********************");
}

void loop() {
  Serial.println("Main");
  while (1) {
    Serial.println("Loop");
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.printf("Uptime %.2fs\n", ((millis() - starttime) * 0.001));
    if (Serial.available() > 0) {
      // Read the incoming byte
      char incomingByte = Serial.read();
      if (incomingByte == ' ' || incomingByte == 'd' || incomingByte == 'D' ) {
        cell_debug(&inverter_data);
        maxim.debug_balance();
      }
      if (incomingByte == 'c' || incomingByte == 'C') {
        if (!can_tx_stop) {
          Serial.println("Muting CAN BUS TX...");
          can_tx_stop = true;
        } else {
          Serial.println("Unmuting CAN BUS TX...");
          can_tx_stop = false;
        }
      }
      // Check if the incoming byte is 'r' or 'R'
      if (incomingByte == 'r' || incomingByte == 'R') {
        Serial.println("Killing spi...");
        vTaskDelete(spiTaskHandle);
        Serial.println("ISO bus down...");
        Serial.println("Killing twai...");
        vTaskDelay(pdMS_TO_TICKS(100));  // Give some time for the message to be sent
        vTaskDelete(twaiTaskHandle);
        vTaskDelay(pdMS_TO_TICKS(100));  // Give some time for the message to be sent
        vTaskDelete(twaiprocessTaskHandle);
        Serial.println("Rebooting the ESP in 10 seconds...");
        digitalWrite(SHDNL_MAX17841_1, LOW);
        delay(10000);
        ESP.restart();  // Reboot the ESP
      }
    }
  }
  // #endif
}  // App in threads only

void TWAI_Processing_Task(void *pvParameters) {
  bool awaiting_contactor_change = false;
  char contactor = 0;
  char status = 0;
  unsigned long summary_time = 0;
  Serial.println("TWAI Processing Task");
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1));
    while (xQueueReceive(rx_queue, (void *)&rxFrame, 10) == pdPASS) {
      vTaskDelay(pdMS_TO_TICKS(1));
      status = can_bus.canread(rxFrame, tx_queue, &inverter_data);
      switch (status) {
        case 0:
          // Serial.println("CAB300 with error");
          // Serial.println("Contactor Off");
          contactor = OFF;
          break;
        case 1:
          // Serial.println("Inv with error");
          // Serial.println("Contactor Off");
          contactor = OFF;
          awaiting_contactor_change = true;
          break;
        case 2:
          switch (contactor) {
            case 0:
              contactor = PRECHARGE;
              break;
            case 1:
              contactor = PRECHARGE | MAIN;
              break;
            case 3:
              contactor = MAIN;
              break;
            // Add an additional case for handling unexpected values of contactor
            default:
              // Set contactor to error state or handle differently
              break;
          }

          // esp_task_wdt_reset();
          awaiting_contactor_change = true;
          break;
        case 3:
          // Serial.println("CAB300 with no error");
          // Serial.println("Contactor no change");
          // esp_task_wdt_reset();
          break;
        default:
          // Serial.println("Unknown status");
          break;
      }
      // vTaskDelay(pdMS_TO_TICKS(1)); // Adjust delay as needed
      if ((contactor_time + 500) < millis()) {

        // Set contactors - min 1 second between changes
        if ((PRECHARGE & MAIN) && awaiting_contactor_change) {
          contactor_time = millis();
          digitalWrite(MAIN_CONTACTOR_PIN, contactor >> 1);
          vTaskDelay(pdMS_TO_TICKS(PRECHARGE_DWELL_TIME_MS));
          digitalWrite(PRECHARGE_PIN, 0);
        } else {
          digitalWrite(PRECHARGE_PIN, contactor & 1);
          digitalWrite(MAIN_CONTACTOR_PIN, contactor >> 1);
        }
        // Serial.printf("Precharge: %d | Main: %d\n\n", contactor & 1, contactor > 1);
        awaiting_contactor_change = false;
      }
    }
    if ((summary_time + 2000) < millis()) {
      can_bus.summary(&inverter_data);
      summary_time = millis();
    }
  }
}

void TWAI_Task(void *pvParameters) {

  // enable panic so ESP32 restarts on can failure
  Serial.println("TWAI_Task Start");
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }
  int last_can_id = 0;
  driver_installed = true;
  while (1) {
    if (!driver_installed) {
      // Driver not installed
      Serial.println("TWAI driver bad");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("Alert: TWAI controller has become error passive.");
      if (twai_start() == ESP_OK) {
        Serial.println("Driver restarted");
      } else {
        Serial.println("CAN failed to restart driver. Rebooting device.");
        ESP.restart();
      }
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
    }
    if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
      ESP_LOGI(EXAMPLE_TAG, "Bus Off state");
      // Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
      twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
      for (int i = 3; i > 0; i--) {
        ESP_LOGW(EXAMPLE_TAG, "Initiate bus recovery in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      twai_initiate_recovery();  // Needs 128 occurrences of bus free signal
      ESP_LOGI(EXAMPLE_TAG, "Initiate bus recovery");
      continue;
    }
    if (alerts_triggered & TWAI_ALERT_BUS_RECOVERED) {
      // Bus recovery was successful, exit control task to uninstall driver
      ESP_LOGI(EXAMPLE_TAG, "Bus Recovered");
      // break;
    }

    // Check if message is received and add to threadsafe queue
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      while (twai_receive(&rxFrame, 5) == ESP_OK) {
        // drop repeated can frame is from current measurement device
        if ((last_can_id == rxFrame.identifier) && (rxFrame.identifier == CAB300a || rxFrame.identifier == CAB300b || rxFrame.identifier == CAB300c || rxFrame.identifier == SAMSUNG_SDI)) {
          vTaskDelay(pdMS_TO_TICKS(1));  // inter-txframe delay
          continue;
        }
        last_can_id = rxFrame.identifier;
        if (xQueueSend(rx_queue, (void *)&rxFrame, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("rx_queue overflow");
        }
      }
    }
    // Check if tx messages are queued and transmit on bus
    while (xQueueReceive(tx_queue, (void *)&rxFrame, 5) == pdPASS) {
#ifdef CAN_TX_INTERSPACE_MS
      vTaskDelay(pdMS_TO_TICKS(CAN_TX_INTERSPACE_MS));  // inter-txframe delay
#endif
      vTaskDelay(pdMS_TO_TICKS(1));  // inter-txframe delay
      if (!can_tx_stop) {
        if (twai_transmit(&rxFrame, 20) != ESP_OK) {
          Serial.println("Failed to dequeue tx message for transmission");
        };
      }
    }
  }
  vTaskDelete(NULL);
}

void SPI_Task(void *pvParameters) {
  while (1) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis1000 >= interval1000) {
      previousMillis1000 = millis();
      inverter_data = maxim.read_pack();
      if (has_errors(&maxim_data)) {
        maxim.display_error();
      }
    }
    if (currentMillis - previousMillis250 >= interval250) {
      previousMillis250 = millis();
      vTaskDelay(pdMS_TO_TICKS(1));
      maxim.do_balance();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
