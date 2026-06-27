#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <esb.h>
#include <string.h>

#include "radio_esb.h"

// Private callback function (main.c doesn't need to know this exists)
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
                    test_payload_t incoming_data;
                    memcpy(&incoming_data, rx_payload.data, sizeof(test_payload_t));
                    
                    printk("RX Success! ID: %d | Value: %f\n", 
                           incoming_data.packet_id, (double)incoming_data.test_value);
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