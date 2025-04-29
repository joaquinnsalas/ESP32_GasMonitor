#ifndef WIFI_H
#define WIFI_H

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <wifi_provisioning/manager.h>

#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
#include <wifi_provisioning/scheme_softap.h>
#endif

#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
#include <wifi_provisioning/scheme_ble.h>
#endif

#include "qrcode.h"

void wifi_provisioning_start(void);
void wifi_init_sta(void);
void get_device_service_name(char *service_name, size_t max);

#endif /* WIFI_H */
