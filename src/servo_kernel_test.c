/*
 * servo_kernel_test.c
 *
 * Author: LikeSmith
 * Date: MARCH 2023
 * 
 * Basic test using high resolution kernel timers to generate a servo signal.
 */

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include <sys/ioctl.h>

// IOCTL commands
#define SERVO_ENB _IO('s',0)            // Enable servo
#define SERVO_DIS _IO('s',1)            // Disable servo
#define SERVO_INV _IO('s',2)            // Invert output
#define SERVO_WF  _IOW('s',3,uint32_t*) // Write flags
#define SERVO_RF  _IOR('s',4,uint32_t*) // Read flags
#define SERVO_WV  _IOW('s',5,uint32_t*) // Write Value
#define SERVO_RV  _IOW('s',6,uint32_t*) // Read Value

// Settings
#define SERVO_DEV "/dev/servo0"
#define SERVO_MIN 1000000
#define SERVO_MAX 2000000

int main(int argc, char **argv)
{
    int fd;
    float val;
    uint32_t pulse_ns;

    printf("Servo Kernel Test...\n");
    printf("Opening servo device...\n");

    if ((fd = open(SERVO_DEV, O_RDWR)) < 0)
    {
        printf("Could not open device.\n");
        return 0;
    }

    printf("Enableing servo...\n");
    if (ioctl(fd, SERVO_INV, NULL))
    {
        printf("Could not invert servo.\n");
        goto exit;
    }
    if (ioctl(fd, SERVO_ENB, NULL))
    {
        printf("Could not enable servo.\n");
        goto exit;
    }

    while(1)
    {
        printf("Enter servo value (negative value to exit): ");
        scanf("%f", &val);

        if (val < 0)
        {
            printf("exiting...\n");
            break;
        }

        pulse_ns = (uint32_t)((SERVO_MAX - SERVO_MIN) * val) + SERVO_MIN;

        if (ioctl(fd, SERVO_WV, &pulse_ns))
        {
            printf("Could not set servo value to %f.\n", val);
        }
        else
        {
            printf("Servo value set to %f (%dns)\n", val, pulse_ns);
        }

    }

exit:
    if (ioctl(fd, SERVO_DIS, NULL))
    {
        printf("Could not disable servo.");
    }
    close(fd);
    return 0;
}
