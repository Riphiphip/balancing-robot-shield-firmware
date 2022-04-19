#define MODULE main

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <nrfx_twis.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(application, CONFIG_LOG_DEFAULT_LEVEL);

static struct device *qdeca_dev;
static struct device *qdecb_dev;


static struct shield_tx_data
{
    struct sensor_value motor_a_pos;
    struct sensor_value motor_b_pos;
} twis_tx_buf;

#define TWI_BUF_SIZE 512

static unsigned char twis_rx_buf[TWI_BUF_SIZE];
const nrfx_twis_t twis_inst = NRFX_TWIS_INSTANCE(1);

static void twis_write_done_handler(void* buffer, uint32_t size){
    LOG_DBG("Data recieved from controller");
}

static void twis_event_handler(nrfx_twis_evt_t const *const p_event)
{
    LOG_DBG("NRFX TWIS event");
    switch(p_event->type){
        case NRFX_TWIS_EVT_READ_REQ: {
            LOG_DBG("NRFX_TWIS_EVT_READ_REQ");
            if (p_event->data.buf_req){
                sensor_sample_fetch(qdeca_dev);
                sensor_channel_get(qdeca_dev, SENSOR_CHAN_ROTATION, &twis_tx_buf.motor_a_pos);

                sensor_sample_fetch(qdecb_dev);
                sensor_channel_get(qdecb_dev, SENSOR_CHAN_ROTATION, &twis_tx_buf.motor_b_pos);
                LOG_DBG("Sending motor A position: %d.%d  motor B position: %d.%d", twis_tx_buf.motor_a_pos.val1, twis_tx_buf.motor_a_pos.val2, twis_tx_buf.motor_b_pos.val1,twis_tx_buf.motor_b_pos.val2);
                nrfx_twis_tx_prepare(&twis_inst, (void*) &twis_tx_buf, sizeof(twis_tx_buf));
            }

            break;
        }
        case NRFX_TWIS_EVT_READ_DONE: {
            LOG_DBG("NRFX_TWIS_EVT_READ_DONE");
            break;
        }
        case NRFX_TWIS_EVT_WRITE_REQ: {
            LOG_DBG("NRFX_TWIS_EVT_WRITE_REQ");
            if (p_event->data.buf_req){
                nrfx_twis_tx_prepare(&twis_inst, (void*) twis_rx_buf, sizeof(twis_rx_buf));
            }
            break;
        }
        case NRFX_TWIS_EVT_WRITE_DONE: {
            LOG_DBG("NRFX_TWIS_EVT_WRITE_DONE");
            twis_write_done_handler(twis_rx_buf, p_event->data.rx_amount);
            break;
        }
        case NRFX_TWIS_EVT_READ_ERROR:
        case NRFX_TWIS_EVT_WRITE_ERROR:
        case NRFX_TWIS_EVT_GENERAL_ERROR: {
            LOG_ERR("TWIS error: %d", p_event->type);
            break;
        }
    }
}

static int twis_init()
{
    nrfx_twis_config_t config;

#define TWIS_NODE DT_NODELABEL(i2c1)
    config.sda = DT_PROP(TWIS_NODE, sda_pin);
    config.sda_pull = NRF_GPIO_PIN_PULLUP;
    config.scl = DT_PROP(TWIS_NODE, scl_pin);
    config.scl_pull = NRF_GPIO_PIN_PULLUP;
    config.addr[0] = DT_PROP(TWIS_NODE, address_0);
    // address-1 field is optional. Default to 0 to disable.
#if (DT_NODE_HAS_PROP(TWIS_NODE, address_1))
    config.addr[1] = DT_PROP(TWIS_NODE, address_1);
#else
    config.addr[1] = 0;
#endif
    config.interrupt_priority = DT_IRQ(DT_NODELABEL(i2c1), priority);

    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2c1)), DT_IRQ(DT_NODELABEL(i2c1), priority), nrfx_isr, nrfx_twis_1_irq_handler, 0);
    if (nrfx_twis_init(&twis_inst, &config, twis_event_handler) != NRFX_SUCCESS)
    {
        LOG_ERR("TWIS init failed");
        return -1;
    }
    nrfx_twis_enable(&twis_inst);
    return 0;
}

void main()
{
    LOG_DBG("Starting shield firmware");
    qdeca_dev = device_get_binding(DT_LABEL(DT_NODELABEL(qdeca)));
    qdecb_dev = device_get_binding(DT_LABEL(DT_NODELABEL(qdecb)));
    if (qdeca_dev == NULL || qdecb_dev == NULL)
    {
        LOG_ERR("Failed to get bindings for qdec devices");
        return;
    }
    twis_init();
}