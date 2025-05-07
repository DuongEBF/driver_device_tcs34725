#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#define DEVICE_NAME "/dev/tcs34725"

// IOCTL commands
#define TCS34725_IOCTL_READ_C _IOR('t', 1, int)
#define TCS34725_IOCTL_READ_R _IOR('t', 2, int)
#define TCS34725_IOCTL_READ_G _IOR('t', 3, int)
#define TCS34725_IOCTL_READ_B _IOR('t', 4, int)

int main()
{
    int fd;
    int ret;
    int color_data;

    // Mở thiết bị TCS34725
    fd = open(DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        perror("Failed to open the device");
        return errno;
    }

    printf("Device opened successfully\n");
    while(1){
        // Đọc giá trị C (Clear)
        ret = ioctl(fd, TCS34725_IOCTL_READ_C, &color_data);
        if (ret == 0) {
            printf("Read Clear Color Data: %d\n", color_data);
        } else {
            perror("Failed to read Clear Color Data");
        }

        // Đọc giá trị R (Red)
        ret = ioctl(fd, TCS34725_IOCTL_READ_R, &color_data);
        if (ret == 0) {
            printf("Read Red Color Data: %d\n", color_data);
        } else {
            perror("Failed to read Red Color Data");
        }

        // Đọc giá trị G (Green)
        ret = ioctl(fd, TCS34725_IOCTL_READ_G, &color_data);
        if (ret == 0) {
            printf("Read Green Color Data: %d\n", color_data);
        } else {
            perror("Failed to read Green Color Data");
        }

        // Đọc giá trị B (Blue)
        ret = ioctl(fd, TCS34725_IOCTL_READ_B, &color_data);
        if (ret == 0) {
            printf("Read Blue Color Data: %d\n", color_data);
        } else {
            perror("Failed to read Blue Color Data");
        }
        usleep(300000);
    }
    // Đóng thiết bị
    close(fd);
    printf("Device closed successfully\n");

    return 0;
}
