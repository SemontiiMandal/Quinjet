#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <esb.h>
#include <string.h>

#include "radio_esb.h"
#include "drivers/analog_joystick.h"

static void esb_callback(struct esb_evt const *event) {
    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            // Hardware ACK received.
            break;
        case ESB_EVENT_TX_FAILED:
            printk("TX Failed: Packet dropped over the air.\n");
            break;
        case ESB_EVENT_RX_RECEIVED:
            {
                struct esb_payload rx_payload;
                if (esb_read_rx_payload(&rx_payload) == 0) {
                    data_packet incoming_data; // Updated to use your new shared struct
                    memcpy(&incoming_data, rx_payload.data, sizeof(data_packet));
                    
                    printk("RX Success! ID: %d | Throttle: %f\n", 
                           incoming_data.packet_id, (double)incoming_data.throttle);
                }
            }
            break;
    }
}


int radio_esb_init(bool is_transmitter) {
    struct esb_config config = ESB_DEFAULT_CONFIG;
    
    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.retransmit_delay = 600;
    config.bitrate = ESB_BITRATE_2MBPS;
    config.event_handler = esb_callback;

    if (is_transmitter) {
        config.mode = ESB_MODE_PTX;
    } else {
        config.mode = ESB_MODE_PRX;
    }

    int err = esb_init(&config);
    if (err) {
        printk("ESB Init Error: %d\n", err);
        return err;
    }

    if (!is_transmitter) {
        esb_start_rx();
        printk("Receiver configured and listening...\n");
    } else {
        printk("Transmitter configured and ready...\n");
    }
    
    return 0;
}

void radio_esb_send_packet(uint8_t *data, uint8_t length) {
    struct esb_payload tx_payload = {
        .pipe = 0,
        .noack = false,
    };
    memcpy(tx_payload.data, data, length);
    tx_payload.length = length;
    esb_write_payload(&tx_payload);
}

// The dedicated transmitter thread
void esb_tx_thread(void *arg1, void *arg2, void *arg3){
    data_packet local_tx_packet;
    uint16_t packet_counter = 0;

    while(1){
        // sleep until ADC says data is ready
        k_sem_take(&wake_esb, K_FOREVER);
        
        // lock, copy, unlock
        k_mutex_lock(&joystick_data, K_FOREVER);
        memcpy(&local_tx_packet, &latest_joystick_state, sizeof(data_packet));
        k_mutex_unlock(&joystick_data);

        // tag it and send it
        local_tx_packet.packet_id = packet_counter++;
        radio_esb_send_packet((uint8_t*)&local_tx_packet, sizeof(data_packet));
    }
}

K_THREAD_DEFINE(esb_thread_id, 2048, esb_tx_thread, NULL, NULL, NULL, 5, 0, 0);