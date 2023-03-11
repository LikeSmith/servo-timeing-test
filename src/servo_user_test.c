/*
 * servo_user_test.c
 *
 * Author: LikeSmith
 * Date: MARCH 2023
 * 
 * Basic test using threads in user space to generate a servo signal.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <limits.h>

#include <sys/mman.h>

// Settings
#define PAGESIZE sysconf(_SC_PAGESIZE)
#define BASE_ADDRESS 0x13400000
#define CON_REG (0x0c20 >> 2)
#define DAT_REG (0x0c24 >> 2)
#define SERVO_BIT 2
#define POLICY SCHED_OTHER
#define SERVO_MIN 1000000
#define SERVO_MAX 2000000
#define SERVO_THREAD_PRIORITY 0

void *servo_channel(void *args);

_Atomic int32_t pulse_ns = 0;

int main(int argc, char **argv)
{
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;
    float val;

    printf("Servo User Test...\n");
    printf("Setting up thread...\n");

    if (mlockall(MCL_CURRENT|MCL_FUTURE) < 0)
    {
        printf("Could not lock memory.");
        return 0;
    }

    if (pthread_attr_init(&attr))
    {
        printf("Could not initialize pthread attr.\n");
        return 0;
    }

    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN))
    {
        printf("Could not set pthread stack size.\n");
        return 0;
    }

    if (pthread_attr_setschedpolicy(&attr, POLICY))
    {
        printf("Could not set pthread scheduler policy.\n");
        return 0;
    }

    param.sched_priority = SERVO_THREAD_PRIORITY;
    if (pthread_attr_setschedparam(&attr, &param))
    {
        printf("Could not set pthread scheduler parameters.\n");
        return 0;
    }

    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED))
    {
        printf("Could not set pthread inherit scheduler flag.\n");
        return 0;
    }

    if (pthread_create(&thread, &attr, servo_channel, NULL))
    {
        printf("Could not start servo thread.\n");
        return 0;
    }

    while (1)
    {
        if (pulse_ns < 0)
        {
            printf("Servo start failed (code %d).\n", pulse_ns);
            pthread_join(thread, NULL);
            return 0;
        }
        else if (pulse_ns > 0)
        {
            break;
        }
    }

    while (1)
    {
        printf("Enter servo value (negative value to exit): ");
        scanf("%f", &val);

        if (val < 0)
        {
            pulse_ns = -1;
            printf("exiting...\n");
            break;
        }

        pulse_ns = (int32_t)((SERVO_MAX - SERVO_MIN) * val) + SERVO_MIN;
        printf("Servo value set to %f (%dns)\n", val, pulse_ns);
    }

    if (pthread_join(thread, NULL))
    {
        printf("Could not join thread.\n");
    }

    return 0;
}

void *servo_channel(void *args)
{
    //_Atomic int32_t *t_ns = (_Atomic int32_t *)args;
    uint32_t * volatile reg;
    int fd;
    struct timespec t_next;
    struct timespec t_switch;

    pulse_ns = 0;
    // open gpio
    if ((fd = open("/dev/mem", O_RDWR)) < 0)
    {
        printf("Could not open memory device.\n");
        pulse_ns = -1;
        return NULL;
    }

    if ((reg = mmap(0, PAGESIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE_ADDRESS)) == MAP_FAILED)
    {
        printf("Could not map memory.\n");
        close(fd);
        pulse_ns = -2;
        return NULL;
    }

    // Set GPIO as output
    *(reg + CON_REG) |= 0x1 << (SERVO_BIT*4);
    *(reg + CON_REG) &= ~((~0x1) << (SERVO_BIT*4));

    *(reg + DAT_REG) |= 1 << SERVO_BIT;
    //*(reg + DAT_REG) &= ~(1 << SERVO_BIT);

    pulse_ns = SERVO_MIN;

    clock_gettime(CLOCK_MONOTONIC, &t_next);

    while(1)
    {
        *(reg + DAT_REG) ^= 1 << SERVO_BIT;

        if (pulse_ns < 0)
        {
            break;
        }

        t_switch.tv_sec = t_next.tv_sec;
        t_switch.tv_nsec = t_next.tv_nsec + pulse_ns;
        t_next.tv_nsec += 20000000;
        t_switch.tv_sec += t_switch.tv_nsec / 1000000000;
        t_switch.tv_nsec = t_switch.tv_nsec % 1000000000;
        t_next.tv_sec += t_next.tv_nsec / 1000000000;
        t_next.tv_nsec = t_next.tv_nsec % 1000000000;

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_switch, NULL);

        *(reg + DAT_REG) ^= 1 << SERVO_BIT;

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, NULL);
    }

    *(reg + DAT_REG) &= ~(1 << SERVO_BIT);

    close(fd);
    return NULL;
}
