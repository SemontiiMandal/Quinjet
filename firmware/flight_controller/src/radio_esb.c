#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <esb.h>
#include <string.h>
#include "radio_esb.h"
#include "drone_packet.h"

data_packet latest_rc_command;
struct k_spinlock rc_spinlock;

/*
Because the esb_callback will run in an ISR context (Interrupt triggered a soon as data packet arrives) we cannot use a k_mutex to protect the shared data. So, we use a spinlock, which disables local interrupts for a very short time (typically fraction of microsecond short) to safely copy the memory between an ISR and a thread!
*/

// ESB Hardware interrupt Callback
static void esb_callback(struct esb_evt const *event) {
    if (event->evt_id == ESB_EVENT_RX_RECEIVED){ 
        struct esb_payload rx_payload;

        if(esb_read_rx_payload(&rx_payload) == 0) {
            k_spinlock_key_t key = k_spin_lock(&rc_spinlock);
            memcpy(&latest_rc_command, rx_payload.data, sizeof(data_packet));
            k_spin_unlock(&rc_spinlock, key);
            
            // fixed print formatting
            // printk("RX! ID: %d | Throttle: %f\n", latest_rc_command.packet_id, (double)latest_rc_command.throttle);
        }
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

/*
void radio_esb_send_packet(uint8_t *data, uint8_t length) {
    struct esb_payload tx_payload = {
        .pipe = 0,
        .noack = false,
    };
    
    memcpy(tx_payload.data, data, length);
    tx_payload.length = length;
    
    esb_write_payload(&tx_payload);
}
    */