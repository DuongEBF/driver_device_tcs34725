NAME: TRAN VAN DUONG         NAME: PHAM MINH TAM
ID: 21146079                 ID: 21146513
Userspace Character Driver for TCS34725 Color Sensor

OVERVIEW
This driver provides userspace access to the TCS34725 RGB color sensor via I2C using a character device interface. It allows reading color data (Red, Green, Blue, and Clear channels) from user applications through standard file operations (open, read, ioctl).

Typical use cases:
- Rapid prototyping with I2C-based color sensor modules.
- Application-level access to color data without writing kernel-space code.
- Educational or embedded systems projects where direct kernel modification is not feasible.

DEVICE CREATION, DRIVER BINDING
To enable this driver:

1. Register an I2C device with the name "tcs34725" in the I2C device tree or board file.
2. Load the kernel module, which will create a character device at /dev/tcs34725.

Expected sysfs hierarchy:
/sys/bus/i2c/devices/i2c-X/...
→ /dev/tcs34725     (character device)
/sys/class/tcs34725/tcs34725X → (optional symlink if exposed)
Udev or mdev will automatically create /dev/tcs34725 upon successful driver binding.

Unbinding:
Can be unbound by removing the I2C device or unloading the kernel module. The character device will be removed automatically.

CHARACTER DEVICE API
1. open(), close()
   Standard open() and close() operations on /dev/tcs34725 are supported.

2. read()
   Currently not implemented. Use ioctl() to get color data.

3. ioctl()
   Supported commands (defined in tcs34725.h):
   #define TCS34725_IOC_MAGIC 't'
   #define TCS34725_GET_CLEAR   _IOR(TCS34725_IOC_MAGIC, 1, int)
   #define TCS34725_GET_RED     _IOR(TCS34725_IOC_MAGIC, 2, int)
   #define TCS34725_GET_GREEN   _IOR(TCS34725_IOC_MAGIC, 3, int)
   #define TCS34725_GET_BLUE    _IOR(TCS34725_IOC_MAGIC, 4, int)

To use:
int fd = open("/dev/tcs34725", O_RDWR);
int red = 0;
ioctl(fd, TCS34725_GET_RED, &red);

Initialization Behavior:
- Device powers on the sensor and enables the RGBC ADC on open().
- On release(), the sensor is disabled to conserve power.

LIMITATIONS
- Only single-value ioctl() access is supported per call.
- No polling or interrupt-based notification is available.
- Not asynchronous.
- Assumes a fixed I2C address (0x29).
- Color values are raw 16-bit data. Calibration must be done in userspace if needed.

EXAMPLE APPLICATION
int fd = open("/dev/tcs34725", O_RDWR);
int r, g, b, c;
ioctl(fd, TCS34725_GET_RED, &r);
ioctl(fd, TCS34725_GET_GREEN, &g);
ioctl(fd, TCS34725_GET_BLUE, &b);
ioctl(fd, TCS34725_GET_CLEAR, &c);
printf("RGB: (%d, %d, %d), Clear: %d
", r, g, b, c);
close(fd);

HOW TO INTEGRATE THIS DRIVER WITH DEVICE TREE

1. Declare the Device in the Device Tree
You need to define the TCS34725 color sensor node in the device tree for your platform. Below is an example of how to declare the device on an I2C bus.

Example of a device tree declaration for TCS34725:

&i2c1 {
    status = "okay";
    tcs34725@29 {
        compatible = "ams,tcs34725";  // Compatible string used for driver matching
        reg = <0x29>;                 // I2C address of the TCS34725 sensor
        interrupt-parent = <&gpio>;
        interrupt = <17 0>;           // Interrupt configuration (optional)
        status = "okay";              // Device status
    };
};

Explanation:
- &i2c1: Reference to the I2C bus, adjust it based on your platform.
- tcs34725@29: Node name for the sensor, indicating it’s located at I2C address 0x29.
- compatible: This field links the device node to the appropriate driver in the kernel.
- reg = <0x29>: I2C address of the sensor.
- interrupt: Optional, if the sensor supports interrupts.
- status = "okay": Enables the device.

2. Modify the Driver to Use Device Tree Information
In the kernel driver code, you can access information from the device tree using the of_property_read_u32 function. For example:

static int tcs34725_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    int ret;

    struct device_node *np = dev->of_node;
    if (np) {
        u32 address;
        if (of_property_read_u32(np, "reg", &address)) {
            dev_err(dev, "Failed to read I2C address from device tree
");
            return -EINVAL;
        }
    }

    // Initialize the TCS34725 sensor here
    ret = tcs34725_init(client);
    if (ret) {
        dev_err(dev, "Failed to initialize TCS34725 sensor
");
        return ret;
    }

    return 0;
}

3. Binding the Driver
You need to register the driver with I2C so that it binds with the correct device based on the compatible string defined in the device tree.

static const struct of_device_id tcs34725_of_match[] = {
    { .compatible = "ams,tcs34725", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tcs34725_of_match);

static struct i2c_driver tcs34725_driver = {
    .driver = {
        .name = "tcs34725",
        .of_match_table = tcs34725_of_match,  // Device tree matching
    },
    .probe = tcs34725_probe,
    .remove = tcs34725_remove,
    .id_table = tcs34725_id,
};
module_i2c_driver(tcs34725_driver);

4. Build and Load the Driver
Once you have integrated the device tree and kernel driver, rebuild the kernel or module and load it onto the system:

1. Build the device tree:
   dtc -I dts -O dtb -o my_device_tree.dtb my_device_tree.dts

2. Load the updated device tree into your system:
   - You may need to update the /boot directory or use the dd command to load the device tree.

3. Check the device:
   After loading the driver, you can use dmesg or check /dev/ to verify if the device is detected properly.

MAKEFILE

To compile the kernel module, you can use a simple Makefile as follows:

obj-m += tcs34725_ioctrl_driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

Explanation of the Makefile:
- obj-m += tcs34725_ioctrl_driver.o: This tells the build system that the tcs34725_ioctrl_driver.o object file is to be compiled as part of a kernel module.
- all: This is the default target, which compiles the module by calling the kernel's build system.
- clean: This target cleans up the build environment, removing object files and other generated files.

To compile the driver:
1. Run `make` in the directory where the Makefile is located.
2. Load the module using `insmod tcs34725_ioctrl_driver.ko`.
3. Verify that the module is loaded with `lsmod`.

To remove the module:
1. Use `rmmod tcs34725_ioctrl_driver` to remove the module.


CONFIGURING ATIME AND CONTROL REGISTERS
---------------------------------------

To properly configure the integration time and gain settings of the TCS34725 sensor, you can write to the `ATIME` and `CONTROL` registers via I2C in the driver code.

ATIME (Integration Time)
~~~~~~~~~~~~~~~~~~~~~~~~~
The `ATIME` register controls the integration time of the RGBC sensor. Lower values result in shorter integration times.

- Register Address: 0x01
- Default Value: 0xFF (2.4ms)
- Example Values:
  - 0xFF → 2.4 ms
  - 0xF6 → 24 ms
  - 0xD5 → 101 ms
  - 0xC0 → 153 ms
  - 0x00 → 700 ms

To set a custom integration time, write the desired value to register 0x01:

    i2c_smbus_write_byte_data(client, 0x01 | 0x80, 0xD5);  // Example: 101ms

CONTROL (Gain Control)
~~~~~~~~~~~~~~~~~~~~~~
The `CONTROL` register sets the analog gain for the sensor.

- Register Address: 0x0F
- Gain Options:
  - 0x00 = 1x gain
  - 0x01 = 4x gain
  - 0x02 = 16x gain
  - 0x03 = 60x gain

Example of setting gain to 16x:

    i2c_smbus_write_byte_data(client, 0x0F | 0x80, 0x02);  // Set gain to 16x

These settings should be configured after enabling the sensor but before starting data acquisition.



====================
Kernel I2C Driver for TCS34725 (Probe-Based)
====================

This section describes an alternative kernel-mode I2C driver implementation for the TCS34725 color sensor using the `probe()` method, rather than exposing a userspace character device with `ioctl()`.

OVERVIEW
--------
This Linux kernel driver binds to an I2C device with name "tcs34725". It uses the standard `probe()` and `remove()` functions to initialize and communicate with the sensor. Once loaded and successfully probed, it reads the RGB and Clear channel data via I2C and prints them to the kernel log (dmesg).

Key Features:
- Direct kernel interaction with the TCS34725 via I2C SMBus
- Reads and prints RGB and Clear data from sensor to kernel log
- Does not expose user-facing APIs (e.g., no `/dev` entry)
- Good for testing I2C communication and sensor integration in kernel space

USAGE INSTRUCTIONS
------------------

1. **Build the kernel module**

Make sure your Makefile includes something like:

```
obj-m += TCS34725_driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
```

2. **Create a device tree overlay or instantiate the I2C device**

Either dynamically create the I2C device (via `/sys`) or add the TCS34725 as a device in your device tree like so:

```dts
&i2c1 {
    tcs34725@29 {
        compatible = "tcs34725";
        reg = <0x29>;
    };
};
```

Or manually via sysfs (if your system allows):

```bash
echo tcs34725 0x29 > /sys/bus/i2c/devices/i2c-1/new_device
```

3. **Insert the module**

```bash
sudo insmod TCS34725_driver.ko
```

4. **Check dmesg for color readings**

```bash
dmesg | grep TCS34725
```

Example output:

```
TCS34725 Color Data - C: 1200, R: 340, G: 400, B: 300
TCS34725 driver initialized
```

5. **Remove the module**

```bash
sudo rmmod TCS34725_driver
```

LIMITATIONS
-----------
- No `/dev` interface or ioctl support — this is a kernel-only test driver
- Sensor data is not exposed to user space
- Color data is printed only once at probe (no polling or repeating read)

AUTHORSHIP
----------
Author: Duong Tam

LICENSE
-------
GPL (General Public License)