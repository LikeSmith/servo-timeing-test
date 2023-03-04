/*
 * servos.c
 *
 * Author: LikeSmith
 * Date: March 2023
 * 
 * Driver for generating Pulse Period Modulated (PPM) signals for controlling
 * RC servos.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <asm/atomic.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LikeSmith");
MODULE_DESCRIPTION("A driver for generating PPM signals to control RC Servos.");
MODULE_VERSION("1.0:0");

// Settings
#define MIN_PERIOD 1000000
#define MAX_PERIOD 2000000
#define SERVO_PERIOD 20000000

// Flags
#define SERVO_ENABLED 0
#define SERVO_INVERTED 1
#define SERVO_ACTIVE 2
#define SERVO_OPEN 3

// IOCTL commands
#define SERVO_ENB _IO('s',0)            // Enable servo
#define SERVO_DIS _IO('s',1)            // Disable servo
#define SERVO_INV _IO('s',2)            // Invert output
#define SERVO_WF  _IOW('s',3,uint32_t*) // Write flags
#define SERVO_RF  _IOR('s',4,uint32_t*) // Read flags
#define SERVO_WV  _IOW('s',5,uint32_t*) // Write Value
#define SERVO_RV  _IOW('s',6,uint32_t*) // Read Value

// Variables
struct servo_data
{
    struct gpio_desc *gpio;
    struct hrtimer timer;
    atomic_t period_ns;
    unsigned int flags;
    unsigned long t_switch;
    unsigned long t_next;
    unsigned int idx;
};

static struct servo_data *servos;
static dev_t servo_dev_first;
static struct class *servo_class;
static struct cdev servo_cdev;
uint8_t n_servos = -1;
struct device_node *dt_dev;

// Get device ids
static const struct of_device_id servo_ids[] =
{
    {.compatible = "servos"},
    {},
};
MODULE_DEVICE_TABLE(of, servo_ids);

// platform device functions
int servo_probe(struct platform_device  *pdev);
int servo_remove(struct platform_device  *pdev);

// timer callback funcitons
enum hrtimer_restart servo_cb(struct hrtimer *timer);

// device file callback functions
int servo_open(struct inode *inode, struct file *file);
int servo_release(struct inode *inode, struct file *file);
ssize_t servo_read(struct file *file, char __user *buf, size_t len, loff_t *off);
ssize_t servo_write(struct file *file, const char __user *buf, size_t len, loff_t *off);
long servo_ioctl(struct file *file, unsigned int, unsigned long);

// device file operations
static struct file_operations servo_fops =
{
    .owner = THIS_MODULE,
    .read = servo_read,
    .write = servo_write,
    .open = servo_open,
    .release = servo_release,
    .unlocked_ioctl = servo_ioctl,
};

// platform driver
static struct platform_driver servo_driver =
{
    .probe = servo_probe,
    .remove = servo_remove,
    .driver = {
        .name = "servo driver",
        .of_match_table = of_match_ptr(servo_ids),
        .owner = THIS_MODULE,
    },
};
module_platform_driver(servo_driver);

// function definitions:
int servo_probe(struct platform_device *pdev)
{
    unsigned int i;
    struct property *prop;
    char n_servos_str[4];
    n_servos_str[0] = '\0';

    pr_info("servos: [INFO] Starting servo driver...\n");

    if (!(dt_dev = of_find_compatible_node(NULL, NULL, "servos")))
    {
        pr_err("servos: [FATAL] Could not locate compatible dt node.\n");
        return -1;
    }

    prop = dt_dev->properties;

    while (prop)
    {
        if (!strcmp(prop->name, "n-servos"))
        {
            strncpy(n_servos_str, prop->value, 3);
            n_servos_str[3] = '\0';
            break;
        }
        prop = prop->next;
    }

    if (kstrtou8(n_servos_str, 10, &n_servos))
    {
        pr_err("servos: [FATAL] Could not determine number of servos in system\n");
        goto nservo_fail;
    }

    pr_info("servos: [INFO] system has %d servos.\n", n_servos);

    if ((servos = kmalloc(sizeof(struct servo_data)*n_servos, GFP_KERNEL)) == NULL)
    {
        pr_err("servos: [FATAL] Could not allocate memory for servos");
        goto nservo_fail;
    }

    // get device major/minor numbers
    if ((i = alloc_chrdev_region(&servo_dev_first, 0, n_servos, "servos")) < 0)
    {
        pr_err("servos: [FATAL] Could not allocate major number\n");
        return -1;
    }

    pr_info("servos: [INFO] Servos got character device %d:%d-%d\n", MAJOR(servo_dev_first), MINOR(servo_dev_first), MINOR(servo_dev_first)+n_servos-1);

    // acquire gpio pins, setup servos
    for (i = 0; i < n_servos; i++)
    {
        if (IS_ERR(servos[i].gpio = gpiod_get_index(&(pdev->dev), "servo", i, GPIOD_OUT_HIGH)))
        {
            pr_err("servos: [FATAL] Could not lock gpio for servo %d.\n", i);
            goto gpio_fail;
        }

        atomic_set(&(servos[i].period_ns), MIN_PERIOD);
        servos[i].flags = 0;
        servos[i].idx = i;
        hrtimer_init(&(servos[i].timer), CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        servos[i].timer.function = &servo_cb;
        hrtimer_start(&(servos[i].timer), ktime_set(0, MIN_PERIOD), HRTIMER_MODE_REL);

        pr_info("servos: [INFO] Servo %d setup.\n", i);
    }

    // setup servo devices
    cdev_init(&servo_cdev, &servo_fops);
    if (IS_ERR(servo_class = class_create(THIS_MODULE, "servo_class")))
    {
        pr_err("servos: [FATAL] Could not create class.\n");
        goto class_fail;
    }
    if (cdev_add(&servo_cdev, servo_dev_first, n_servos) < 0)
    {
        pr_err("servos: [FATAL] Could not add devices to cdev");
        goto class_fail;
    }
    for (i = 0; i < n_servos; i++)
    {
        dev_t dev = MKDEV(MAJOR(servo_dev_first), i);
        if (IS_ERR(device_create(servo_class, &(pdev->dev), dev, NULL, "servo%d", i)))
        {
            pr_err("servos: [FATAL] Failed to create device servo%d\n", i);
            goto device_fail;
        }

        pr_info("servos: [INFO] Created dev file for servo %d", i);
    }

    pr_info("servos: [INFO] Servos module successfully probed.\n");
    return 0;

    // Cleanup in case of failure
device_fail:
    for (i = 0; i < n_servos; i++)
    {
        dev_t dev = MKDEV(MAJOR(servo_dev_first), i);
        device_destroy(servo_class, dev);
    }
    class_destroy(servo_class);
class_fail:
    cdev_del(&servo_cdev);
gpio_fail:
    for (i = 0; i < n_servos; i++)
    {
        hrtimer_cancel(&(servos[i].timer));
        gpiod_set_value(servos[i].gpio, 0);
        gpiod_put(servos[i].gpio);
    }
    unregister_chrdev_region(servo_dev_first, n_servos);
    kfree(servos);
nservo_fail:
    of_node_put(dt_dev);
    return -1;
}

int servo_remove(struct platform_device *pdev)
{
    unsigned char i;

    for (i = 0; i < n_servos; i++)
    {
        dev_t dev = MKDEV(MAJOR(servo_dev_first), i);
        device_destroy(servo_class, dev);
        hrtimer_cancel(&(servos[i].timer));
        gpiod_set_value(servos[i].gpio, 0);
        gpiod_put(servos[i].gpio);
    }
    class_destroy(servo_class);
    cdev_del(&servo_cdev);
    unregister_chrdev_region(servo_dev_first, n_servos);
    kfree(servos);
    of_node_put(dt_dev);

    pr_info("servos: [INFO] Servos module successfully removed.\n");

    return 0;
}

enum hrtimer_restart servo_cb(struct hrtimer *timer)
{
    struct servo_data *servo = container_of(timer, struct servo_data, timer);

    if (test_bit(SERVO_ENABLED, (void *) &(servo->flags)))
    {
        if (test_bit(SERVO_ACTIVE, (void *) &(servo->flags)))
        {
            gpiod_set_value(servo->gpio, test_bit(SERVO_INVERTED, (void *) &(servo->flags)));
            clear_bit(SERVO_ACTIVE, (void *) &(servo->flags));
            hrtimer_add_expires_ns(timer, servo->t_next);
        }
        else
        {
            gpiod_set_value(servo->gpio, !test_bit(SERVO_INVERTED, (void *) &(servo->flags)));
            set_bit(SERVO_ACTIVE, (void *) &(servo->flags));

            servo->t_switch = atomic_read(&(servo->period_ns));
            servo->t_next = SERVO_PERIOD - servo->t_switch;
            hrtimer_add_expires_ns(timer, servo->t_switch);
        }
    }
    else
    {
        hrtimer_add_expires_ns(timer, SERVO_PERIOD);
    }

    return HRTIMER_RESTART;
}

int servo_open(struct inode *inodep, struct file *filp)
{
    unsigned int idx = MINOR(inodep->i_rdev);

    if (test_bit(SERVO_OPEN, (void *) &(servos[idx].flags)))
    {
        pr_warn("servos: [ERROR] A process tried to open servo %d when it was already opened.\n", idx);
        return -1;
    }

    set_bit(SERVO_OPEN, (void *) &(servos[idx].flags));
    filp->private_data = (void *) &(servos[idx]);
    return 0;
}

int servo_release(struct inode *inodep, struct file *filp)
{
    unsigned int idx = MINOR(inodep->i_rdev);
    clear_bit(SERVO_OPEN, (void *) &(servos[idx].flags));
    return 0;
}

ssize_t servo_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    struct servo_data *servo = (struct servo_data *)(filp->private_data);
    unsigned int period_ns = atomic_read(&(servo->period_ns));
    unsigned char kbuf[16];
    size_t klen;
    size_t min;

    if (*off > 0)
    {
        return 0;
    }

    klen = snprintf(kbuf, 16, "%u\n", period_ns);

    min = len < klen ? len : klen;

    if (copy_to_user(buf, kbuf, min))
    {
        pr_err("servos: [ERROR] Could not write to output buffer.\n");
        return 0;
    }

    *off += min;
    return min;
}

ssize_t servo_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    struct servo_data *servo = (struct servo_data *)(filp->private_data);
    unsigned int period_ns;
    unsigned char kbuf[16];
    size_t klen = len < 15 ? len : 15;

    if (copy_from_user(kbuf, buf, klen))
    {
        pr_err("servos: [ERROR] Could not read from input buffer.\n");
        return 0;
    }

    kbuf[klen+1] = '\0';
    if (sscanf(kbuf, "%d\n", &period_ns) != 1)
    {
        pr_err("servos: [ERROR] Could not convert input \"%s\" to int.\n", kbuf);
        return klen;
    }

    if (period_ns < MIN_PERIOD)
    {
        pr_warn("servos: [WARN] specified period of %dns is below minimum period of %dns, using minimum.\n", period_ns, MIN_PERIOD);
        period_ns = MIN_PERIOD;
    }
    else if (period_ns > MAX_PERIOD)
    {
        pr_warn("servos: [WARN] specified period of %dns is above maximum period of %dns, using maximum.\n", period_ns, MIN_PERIOD);
        period_ns = MAX_PERIOD;
    }
    
    atomic_set(&(servo->period_ns), period_ns);

    return klen;
}

long servo_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct servo_data *servo = (struct servo_data *)(filp->private_data);
    unsigned int new_value = 0;
    int success = 0;

    switch (cmd)
    {
    case SERVO_ENB:
        clear_bit(SERVO_ACTIVE, (void *) &(servo->flags));
        set_bit(SERVO_ENABLED, (void *) &(servo->flags));
        break;
    case SERVO_DIS:
        clear_bit(SERVO_ENABLED, (void *) &(servo->flags));
        break;
    case SERVO_INV:
        change_bit(SERVO_INVERTED, (void *) &(servo->flags));
        break;
    case SERVO_WF:
        if (copy_from_user(&new_value, (uint32_t *)arg, sizeof(new_value)))
        {
            pr_err("servos: [ERROR] Servo %d received new flags, but could not apply them.\n", servo->idx);
            success = -1;
            break;
        }
        if (new_value & (1 << SERVO_ENABLED))
        {
            set_bit(SERVO_ENABLED, (void *) &(servo->flags));
            pr_info("servos: setting enable.\n");
        }
        else
        {
            clear_bit(SERVO_ENABLED, (void *) &(servo->flags));
            pr_info("servos: clearing enable.\n");
        }
        if (new_value & (1 << SERVO_INVERTED))
        {
            set_bit(SERVO_INVERTED, (void *) &(servo->flags));
            pr_info("servos: setting invert.\n");
        }
        else
        {
            clear_bit(SERVO_INVERTED, (void *) &(servo->flags));
            pr_info("servos: clearing invert.\n");
        }
        pr_info("servos: [INFO] writing new flags (%d) to servo %d\n", new_value, servo->idx);
        break;
    case SERVO_RF:
        if (test_bit(SERVO_ENABLED, (void *) &(servo->flags)))
        {
            new_value |= (1 << SERVO_ENABLED);
        }
        if (test_bit(SERVO_INVERTED, (void *) &(servo->flags)))
        {
            new_value |= (1 << SERVO_INVERTED);
        }
        if (copy_to_user((uint32_t *)arg, &new_value, sizeof(new_value)))
        {
            pr_err("servos: [ERROR] Servo %d was asked for flags, but could not supply them.\n", servo->idx);
            success = -2;
        }
        pr_info("servos: [INFO] reading flags (%d) from servo %d\n", new_value, servo->idx);
        break;
    case SERVO_WV:
        if (copy_from_user(&new_value, (uint32_t *)arg, sizeof(new_value)))
        {
            pr_err("servos: [ERROR] Servo %d received new value, but could not apply them.\n", servo->idx);
            success = -3;
            break;
        }
        if (new_value < MIN_PERIOD)
        {
            pr_warn("servos: [WARN] On Servo %d, specified period of %dns is below minimum period of %dns, using minimum.\n", servo->idx, new_value, MIN_PERIOD);
            new_value = MIN_PERIOD;
        }
        else if (new_value > MAX_PERIOD)
        {
            pr_warn("servos: [WARN] On Servo %d, pecified period of %dns is above maximum period of %dns, using maximum.\n", servo->idx, new_value, MIN_PERIOD);
            new_value = MAX_PERIOD;
        }
        atomic_set(&(servo->period_ns), new_value);
        break;
    case SERVO_RV:
        new_value = atomic_read(&(servo->period_ns));

        if (copy_to_user((uint32_t *)arg, &new_value, sizeof(new_value)))
        {
            pr_err("servos: [ERROR] Servo %d was asked to for value, but could not supply it.\n", servo->idx);
            success = -4;
        }
        break;
    default:
        pr_warn("servos: [WARN] Servo %d received unknown IOCTL command %d.\n", servo->idx, cmd);
        success = -5;
        break;
    }

    return success;
}
