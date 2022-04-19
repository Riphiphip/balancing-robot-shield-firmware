#include <devicetree.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(sensor_qdec_gpio, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT nordic_qdec_gpio
#define QDEC_GPIO_INIT_PRIORITY 41

#define FULL_ANGLE 360

struct qdec_gpio_cb_container
{
    struct gpio_callback cb;
    struct device *dev;
};

struct qdec_gpio_data
{
    uint8_t prev_state_a;
    uint8_t prev_state_b;
    int32_t counter;
    int32_t fetched_counter;
    sensor_trigger_handler_t data_ready_handler;
};

struct qdec_gpio_conf
{
    struct gpio_dt_spec gpio_a;
    struct qdec_gpio_cb_container gpio_a_cb_c;
    struct gpio_dt_spec gpio_b;
    struct qdec_gpio_cb_container gpio_b_cb_c;
    int32_t ticks_per_rotation;
};

static int qdec_gpio_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    if (!(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_ROTATION))
    {
        LOG_ERR("Invalid channel %d. Only SENSOR_CHAN_ALL and SENSOR_CHAN_ROTATION are supported.", chan);
        return -ENOTSUP;
    }

    struct qdec_gpio_data *data = dev->data;
    unsigned int key = irq_lock();
    data->fetched_counter = data->counter;
    irq_unlock(key);

    return 0;
}

static int qdec_gpio_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    if (chan != SENSOR_CHAN_ROTATION)
    {
        LOG_ERR("Invalid channel %d. Only SENSOR_CHAN_ROTATION is supported.", chan);
        return -ENOTSUP;
    }

    const struct qdec_gpio_conf *conf = dev->config;
    const struct qdec_gpio_data *data = dev->data;

    int32_t steps = conf->ticks_per_rotation;

    int32_t counter = data->fetched_counter;

    val->val1 = (counter * FULL_ANGLE) / steps;

    val->val2 = (counter * FULL_ANGLE) % steps;
    val->val2 *= 1000000;
    val->val2 /= steps;

    return 0;
}

static int qdec_gpio_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
    struct qdec_gpio_data *data = dev->data;

    if (trig->type != SENSOR_TRIG_DATA_READY)
    {
        LOG_ERR("Invalid trigger type %d. Only SENSOR_TRIG_DATA_READY is supported.", trig->type);
        return -ENOTSUP;
    }

    if ((trig->chan != SENSOR_CHAN_ALL) &&
        (trig->chan != SENSOR_CHAN_ROTATION))
    {
        LOG_ERR("Invalid trigger channel %d. Only SENSOR_CHAN_ROTATION is supported.", trig->chan);
        return -ENOTSUP;
    }

    unsigned int key = irq_lock();
    data->data_ready_handler = handler;
    irq_unlock(key);

    return 0;
}

static void qdec_line_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    static int8_t lookup_table[] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};

    struct device *dev = CONTAINER_OF(cb, struct qdec_gpio_cb_container, cb)->dev;
    struct qdec_gpio_data *data = dev->data;
    struct qdec_gpio_conf *conf = dev->config;

    uint8_t old_a = data->prev_state_a;
    uint8_t old_b = data->prev_state_b;

    uint8_t new_a = gpio_pin_get(port, conf->gpio_a.pin);
    uint8_t new_b = gpio_pin_get(port, conf->gpio_b.pin);

    data->prev_state_a = new_a;
    data->prev_state_b = new_b;

    uint8_t movement_index = (old_a << 3) + (old_b << 2) + (new_a << 1) + new_b;
    if (lookup_table[movement_index] > 1)
    {
        LOG_WRN("Quadrature decoder made invalid transition. old_a: %d, old_b: %d, new_a: %d, new_b: %d", old_a, old_b, new_a, new_b);
        return;
    }

    data->counter += lookup_table[movement_index];

    if (data->data_ready_handler)
    {
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ROTATION,
        };
        data->data_ready_handler(dev, &trig);
    }
}

static int init_gpio(const struct device *dev)
{
    struct qdec_gpio_conf *conf = dev->config;
    int err;
    err = gpio_pin_configure_dt(&conf->gpio_a, GPIO_INPUT | GPIO_INT_DEBOUNCE);
    err |= gpio_pin_interrupt_configure_dt(&conf->gpio_a, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&conf->gpio_a_cb_c.cb, qdec_line_callback, BIT(conf->gpio_a.pin));
    err |= gpio_add_callback(conf->gpio_a.port, &conf->gpio_a_cb_c.cb);
    if (err)
    {
        LOG_ERR("Failed to configure gpio_a");
        return err;
    }

    err = gpio_pin_configure_dt(&conf->gpio_b, GPIO_INPUT | GPIO_INT_DEBOUNCE);
    err |= gpio_pin_interrupt_configure_dt(&conf->gpio_b, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&conf->gpio_b_cb_c.cb, qdec_line_callback, BIT(conf->gpio_b.pin));
    err |= gpio_add_callback(conf->gpio_b.port, &conf->gpio_b_cb_c.cb);
    if (err)
    {
        LOG_ERR("Failed to configure gpio_b");
        return err;
    }
    return 0;
}

static const struct sensor_driver_api qdec_gpio_driver_api = {
    .sample_fetch = qdec_gpio_sample_fetch,
    .channel_get = qdec_gpio_channel_get,
    .trigger_set = qdec_gpio_trigger_set,
};

static int init_qdec_gpio(const struct device *dev)
{

    struct qdec_gpio_conf *conf = dev->config;
    struct qdec_gpio_data *data = dev->data;
    int err;

    err = init_gpio(dev);
    if (err)
    {
        LOG_ERR("Failed to initialize gpio: %d", err);
        return err;
    }

    conf->gpio_a_cb_c.dev = dev;
    conf->gpio_b_cb_c.dev = dev;

    data->prev_state_a = gpio_pin_get_dt(&conf->gpio_a);
    data->prev_state_b = gpio_pin_get_dt(&conf->gpio_b);

    return 0;
}

#define INIT_QDEC_GPIO(inst)                                          \
    static struct qdec_gpio_data qdec_gpio_data_##inst = {0};         \
                                                                      \
    static struct qdec_gpio_conf qdec_gpio_config_##inst = {          \
        .gpio_a = GPIO_DT_SPEC_INST_GET(inst, line_a_gpios),          \
        .gpio_b = GPIO_DT_SPEC_INST_GET(inst, line_b_gpios),          \
        .ticks_per_rotation = DT_INST_PROP(inst, ticks_per_rotation), \
    };                                                                \
                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                       \
                          init_qdec_gpio,                             \
                          NULL,                                       \
                          &qdec_gpio_data_##inst,                     \
                          &qdec_gpio_config_##inst,                   \
                          POST_KERNEL,                                \
                          QDEC_GPIO_INIT_PRIORITY,                    \
                          &qdec_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_QDEC_GPIO);
