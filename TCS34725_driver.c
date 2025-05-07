#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define DRIVER_NAME "tcs34725_driver"

#define TCS34725_ADDR             0x29
#define TCS34725_COMMAND_BIT      0x80

#define TCS34725_ENABLE           0x00
#define TCS34725_ENABLE_PON       0x01
#define TCS34725_ENABLE_AEN       0x02

#define TCS34725_CDATAL           0x14
#define TCS34725_RDATAL           0x16
#define TCS34725_GDATAL           0x18
#define TCS34725_BDATAL           0x1A

static struct i2c_client *tcs34725_client;

static int tcs34725_read_word(struct i2c_client *client, u8 reg)
{
    int low, high;

    low = i2c_smbus_read_byte_data(client, TCS34725_COMMAND_BIT | reg);
    if (low < 0) return low;

    high = i2c_smbus_read_byte_data(client, TCS34725_COMMAND_BIT | (reg + 1));
    if (high < 0) return high;

    return (high << 8) | low;
}

static int tcs34725_enable(struct i2c_client *client)
{
    int ret;

    // bat nguon
    ret = i2c_smbus_write_byte_data(client, TCS34725_COMMAND_BIT | TCS34725_ENABLE, TCS34725_ENABLE_PON);
    if (ret < 0) return ret;

    msleep(10);

    // bat ADC
    ret = i2c_smbus_write_byte_data(client, TCS34725_COMMAND_BIT | TCS34725_ENABLE,
                                    TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    if (ret < 0) return ret;

    msleep(700);  // doi ADC on dinh

    return 0;
}

static int tcs34725_read_color_data(struct i2c_client *client)
{
    int c, r, g, b;

    c = tcs34725_read_word(client, TCS34725_CDATAL);
    r = tcs34725_read_word(client, TCS34725_RDATAL);
    g = tcs34725_read_word(client, TCS34725_GDATAL);
    b = tcs34725_read_word(client, TCS34725_BDATAL);

    if (c < 0 || r < 0 || g < 0 || b < 0) {
        printk(KERN_ERR "Failed to read color data\n");
        return -EIO;
    }

    printk(KERN_INFO "TCS34725 Color Data - C: %d, R: %d, G: %d, B: %d\n", c, r, g, b);
    return 0;
}

static int tcs34725_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    tcs34725_client = client;

    ret = tcs34725_enable(client);
    if (ret < 0) {
        printk(KERN_ERR "Failed to enable TCS34725\n");
        return ret;
    }

    ret = tcs34725_read_color_data(client);
    if (ret < 0) {
        return ret;
    }

    printk(KERN_INFO "TCS34725 driver initialized\n");
    return 0;
}

static int tcs34725_remove(struct i2c_client *client)
{
    printk(KERN_INFO "TCS34725 driver removed\n");
    return 0;
}

static const struct i2c_device_id tcs34725_id[] = {
    { "tcs34725", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, tcs34725_id);

static struct i2c_driver tcs34725_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
    .probe = tcs34725_probe,
    .remove = tcs34725_remove,
    .id_table = tcs34725_id,
};

static int __init tcs34725_init(void)
{
    printk(KERN_INFO "Initializing TCS34725 driver\n");
    return i2c_add_driver(&tcs34725_driver);
}

static void __exit tcs34725_exit(void)
{
    printk(KERN_INFO "Exiting TCS34725 driver\n");
    i2c_del_driver(&tcs34725_driver);
}

module_init(tcs34725_init);
module_exit(tcs34725_exit);

MODULE_AUTHOR("Duong Tam");
MODULE_DESCRIPTION("TCS34725 I2C Driver for Linux Kernel");
MODULE_LICENSE("GPL");
