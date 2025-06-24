/*
* Eletrônica Embarcada com IoT, IA e Robótica - exercicio 02 - Leitor de temperatura interna do IMU MPU6050 com envio via BLE
* Aluno: Victor Hugo de Toledo Nunes
* Prof.: Gustavo Ferreira Palma 
*/


#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "MPU_6050_GATT.h"
#include "haw/MPU6050.h"
#include "btstack.h"
#include "pico/btstack_cyw43.h"

#define HEARTBEAT_PERIOD_MS 100
#define APP_AD_FLAGS 0x06

static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    0x0C, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', '2', 'W', ' ', 'T', 'E', 'M', 'P',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};

static const uint8_t adv_data_len = sizeof(adv_data);

bool le_notification_enabled = false;
hci_con_handle_t con_handle;

static float temperatura = 0;
static uint16_t tx_temp = 0;
mpu6050_t mpu6050;

void init_imu_mpu_6050();
void poll_temperature();
static void heartbeat_handler(struct btstack_timer_source *ts);
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

void init_imu_mpu_6050() {
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    i2c_init(i2c_default, 400000);

    mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);
    if(!mpu6050_begin(&mpu6050)) {
        while(true){
            printf("Erro ao inicializar o sensor!\n");
            sleep_ms(500);
        }
    }

    mpu6050_set_temperature_measuring(&mpu6050, true);
}

void poll_temperature() {
    mpu6050_event(&mpu6050);
    temperatura = mpu6050_get_temperature(&mpu6050);
    tx_temp = (uint16_t)(temperatura * 100); // Ex: 25.38°C -> 2538
    printf("Temperatura: %.2f°C\n", temperatura);
}

static void heartbeat_handler(struct btstack_timer_source *ts) {
    static bool led = true;
    led = !led;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);

    poll_temperature();

    if (le_notification_enabled && con_handle) {
        att_server_request_can_send_now_event(con_handle);
    }

    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            bd_addr_t end_local;
            gap_local_bd_addr(end_local);
            printf("Bluetooth pronto no endereço: %s\n", bd_addr_to_str(end_local));
        break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            le_notification_enabled = false;
        break;

        case ATT_EVENT_CAN_SEND_NOW:
            if (le_notification_enabled) {
                att_server_notify(con_handle, ATT_CHARACTERISTIC_TEMP_NOTIFY_VALUE_HANDLE, (uint8_t *)&tx_temp, sizeof(tx_temp));
            }
        break;
    }
}

static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
    if (att_handle == ATT_CHARACTERISTIC_TEMP_NOTIFY_VALUE_HANDLE) {
        return att_read_callback_handle_blob((const uint8_t *)&tx_temp, sizeof(tx_temp), offset, buffer, buffer_size);
    }
    return 0;
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    if (att_handle == ATT_CHARACTERISTIC_TEMP_NOTIFY_CLIENT_CONFIGURATION_HANDLE) {
        le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        con_handle = connection_handle;
        att_server_request_can_send_now_event(con_handle);
    }
    return 0;
}

int main() {
    stdio_init_all();
    printf("Iniciando aplicação BLE TEMP\n");

    if (cyw43_arch_init()) {
        printf("Erro ao inicializar Wi-Fi/Bluetooth\n");
        return -1;
    }

    l2cap_init();
    sm_init();
    init_imu_mpu_6050();

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(packet_handler);

    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    att_server_init(profile_data, att_read_callback, att_write_callback);

    bd_addr_t endereco = {0};
    gap_advertisements_set_params(800, 800, 0, 0, endereco, 0x07, 0);
    gap_advertisements_set_data(adv_data_len, adv_data);
    gap_advertisements_enable(true);
    hci_power_control(HCI_POWER_ON);

    btstack_run_loop_execute();
}
