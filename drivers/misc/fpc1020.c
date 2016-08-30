/**
 * Copyright (c) 2016 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <asm/unaligned.h>

#define FPC_NUM_SUPPLIES		2

#define FPC_CMD_DEEP_SLEEP		0x2c

#define FPC_REG_IRQ			0x18
#define FPC_REG_HWID			0xfc

#define FPC_FINGER_PRESENT		BIT(0)

struct fpc_data {
	struct device dev;
	struct cdev cdev;

	struct spi_device *spi;

	struct regulator_bulk_data supplies[FPC_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;

	wait_queue_head_t wait;

	struct mutex mutex;

	bool opened;
	bool dead;
	bool data_available;

	u16 hwid;

	u8 *xfer_buf;
	size_t xfer_size;

	u8 scratch_buf[4] ____cacheline_aligned;
};

static struct class *fpc_class;
static const char fpc_char_device_name[] = "fpc_sensor";
static int fpc_major;
static DEFINE_IDA(fpc_minor_ida);

static irqreturn_t fpc1020_irq_handler(int irq, void *_dev)
{
	struct fpc_data *fpc = _dev;

	wake_up(&fpc->wait);
	return IRQ_HANDLED;
}

static int fpc1020_spi_writeread(struct spi_device *spi, u8 *buf, int len)
{
	struct spi_transfer transfer = {
		.rx_buf = buf,
		.tx_buf = buf,
		.len = len,
	};

	return spi_sync_transfer(spi, &transfer, 1);
}

static int fpc1020_sleep(struct fpc_data *fpc)
{
	int error;

	fpc->scratch_buf[0] = FPC_CMD_DEEP_SLEEP;
	error = fpc1020_spi_writeread(fpc->spi, fpc->scratch_buf, 1);
	if (error) {
		dev_err(&fpc->spi->dev, "%s: SPI transfer failed: %d\n",
			__func__, error);
		return error;
	}
	return 0;
}

static int fpc1020_get_finger_present(struct fpc_data *fpc,
				      bool *finger_present)
{
	int error;

	*finger_present = false;

	fpc->scratch_buf[0] = FPC_REG_IRQ;

	error = fpc1020_spi_writeread(fpc->spi, fpc->scratch_buf, 2);
	if (error) {
		dev_err(&fpc->spi->dev,
			"Failed to read finger status: %d\n", error);
		return error;
	}

	if (fpc->scratch_buf[1] & FPC_FINGER_PRESENT)
		*finger_present = true;

	return 0;
}

static int fpc1020_power_on(struct fpc_data *fpc)
{
	int error;

	if (fpc->reset_gpio)
		gpiod_set_value_cansleep(fpc->reset_gpio, 1);

	error = regulator_bulk_enable(ARRAY_SIZE(fpc->supplies), fpc->supplies);
	if (error) {
		dev_err(&fpc->spi->dev,
			"failed to enable regulators: %d\n", error);
		return error;
	}

	if (fpc->reset_gpio) {
		usleep_range(100, 200);
		gpiod_set_value_cansleep(fpc->reset_gpio, 0);
	}

	usleep_range(10000, 20000);

	return 0;
}

static unsigned int fpc1020_poll(struct file *filp, poll_table *wait)
{
	struct fpc_data *fpc = filp->private_data;
	bool finger_present = false;
	unsigned int retval = POLLOUT;

	poll_wait(filp, &fpc->wait, wait);

	mutex_lock(&fpc->mutex);
	if (fpc->dead) {
		retval = POLLHUP;
		goto out;
	}

	if (fpc1020_get_finger_present(fpc, &finger_present)) {
		retval = POLLERR;
		goto out;
	}

	if (finger_present)
		retval = POLLIN | POLLRDNORM;

out:
	mutex_unlock(&fpc->mutex);
	return retval;
}

static int fpc1020_open(struct inode *inode, struct file *file)
{
	struct fpc_data *fpc = container_of(inode->i_cdev,
					    struct fpc_data, cdev);
	int retval;

	mutex_lock(&fpc->mutex);

	if (fpc->opened) {
		retval = -EBUSY;
		goto out;
	}

	fpc->opened = true;
	get_device(&fpc->dev);

	file->private_data = fpc;

	retval = 0;

out:
	mutex_unlock(&fpc->mutex);
	return retval;
}

static int fpc1020_release(struct inode *inode, struct file *file)
{
	struct fpc_data *fpc = file->private_data;

	mutex_lock(&fpc->mutex);
	fpc->opened = false;
	fpc->data_available = false;
	kfree(fpc->xfer_buf);
	fpc->xfer_buf = NULL;
	fpc->xfer_size = 0;
	mutex_unlock(&fpc->mutex);

	put_device(&fpc->dev);
	return 0;
}

static ssize_t fpc1020_read(struct file *filp, char *buf, size_t count,
			    loff_t *offp)
{
	struct fpc_data *fpc = filp->private_data;
	int error = 0;

	mutex_lock(&fpc->mutex);

	if (!fpc->data_available) {
		dev_dbg(&fpc->spi->dev, "No data available to read\n");
		error = -EIO;
		goto out;
	}

	if (count > fpc->xfer_size) {
		dev_dbg(&fpc->spi->dev,
			"Invalid read size, count %zu, have %zu\n",
			count, fpc->xfer_size);
		error = -EINVAL;
		goto out;
	}

	if (copy_to_user(buf, fpc->xfer_buf, count))
		error = -EFAULT;

	kfree(fpc->xfer_buf);
	fpc->xfer_buf = NULL;
	fpc->data_available = false;

out:
	mutex_unlock(&fpc->mutex);
	return error ?: count;
}

static ssize_t fpc1020_write(struct file *filp, const char *buf, size_t count,
			     loff_t *offp)
{
	struct fpc_data *fpc = filp->private_data;
	int error = 0;

	mutex_lock(&fpc->mutex);

	if (fpc->data_available) {
		dev_dbg(&fpc->spi->dev,
			"Cannot write, there is data available to read.\n");
		error = -EBUSY;
		goto out;
	}

	fpc->xfer_size = count;
	fpc->xfer_buf = memdup_user(buf, count);
	if (!fpc->xfer_buf) {
		error = -ENOMEM;
		goto out;
	}

	error = fpc1020_spi_writeread(fpc->spi, fpc->xfer_buf, fpc->xfer_size);
	if (error) {
		kfree(fpc->xfer_buf);
		fpc->xfer_buf = NULL;
		goto out;
	}

	fpc->data_available = true;

out:
	mutex_unlock(&fpc->mutex);
	return error ?: count;
}

static const struct file_operations fpc_fops = {
	.owner		= THIS_MODULE,
	.poll		= fpc1020_poll,
	.open		= fpc1020_open,
	.release	= fpc1020_release,
	.read		= fpc1020_read,
	.write		= fpc1020_write,
	.llseek		= no_llseek,
};

static ssize_t hwid_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fpc_data *fpc = spi_get_drvdata(spi);

	return snprintf(buf, PAGE_SIZE, "%04X\n", fpc->hwid);
}

static DEVICE_ATTR_RO(hwid);

static const struct attribute_group fpc1020_group = {
	.attrs = (struct attribute *[]) {
		&dev_attr_hwid.attr,
		NULL
	},
};

static void fpc1020_free(struct device *dev)
{
	struct fpc_data *fpc = container_of(dev, struct fpc_data, dev);

	kfree(fpc->xfer_buf);
	kfree(fpc);
}

static int fpc1020_get_hwid(struct fpc_data *fpc)
{
	int error;

	fpc->scratch_buf[0] = FPC_REG_HWID;
	fpc->scratch_buf[1] = 0;
	fpc->scratch_buf[2] = 0;

	error = fpc1020_spi_writeread(fpc->spi, fpc->scratch_buf, 3);
	if (error) {
		dev_err(&fpc->spi->dev, "%s: SPI transfer failed: %d\n",
			__func__, error);
		return error;
	}

	fpc->hwid = get_unaligned_be16(&fpc->scratch_buf[1]);
	if (fpc->hwid == 0 || fpc->hwid == 0xffff) {
		dev_err(&fpc->spi->dev, "invalid HWID: %#04x\n", fpc->hwid);
		return -EIO;
	}

	return 0;
}

static int fpc1020_spi_probe(struct spi_device *spi)
{
	struct fpc_data *fpc;
	int minor;
	int error = -1;

	minor = ida_simple_get(&fpc_minor_ida, 0, 0, GFP_KERNEL);
	if (minor < 0) {
		error = minor;
		dev_err(&spi->dev,
			"failed to get a new minor number: %d\n", error);
		return error;
	}

	fpc = kzalloc(sizeof(*fpc), GFP_KERNEL);
	if (!fpc)
		goto err_free_minor;

	dev_set_name(&fpc->dev, "fpc_sensor%d", minor);
	fpc->dev.devt = MKDEV(fpc_major, minor);
	fpc->dev.class = fpc_class;
	fpc->dev.parent = &spi->dev;
	fpc->dev.release = fpc1020_free;
	device_initialize(&fpc->dev);

	fpc->spi = spi;
	mutex_init(&fpc->mutex);
	init_waitqueue_head(&fpc->wait);
	fpc->opened = false;

	fpc->supplies[0].supply = "vdd";
	fpc->supplies[1].supply = "vddio";

	error = regulator_bulk_get(&spi->dev,
		ARRAY_SIZE(fpc->supplies), fpc->supplies);
	if (error) {
		dev_err(&spi->dev, "couldn't get regulators: %d\n", error);
		goto err_free_fpc;
	}

	fpc->reset_gpio = gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(fpc->reset_gpio)) {
		error = PTR_ERR(fpc->reset_gpio);
		dev_err(&spi->dev, "failed to acquire reset gpio: %d\n", error);
		goto err_free_regulators;
	}

	error = fpc1020_power_on(fpc);
	if (error) {
		dev_err(&spi->dev, "failed to power on chip: %d\n", error);
		goto err_free_gpio;
	}

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	error = spi_setup(spi);
	if (error) {
		dev_err(&spi->dev, "failed to set up SPI device: %d\n", error);
		goto err_power_off;
	}

	error = fpc1020_get_hwid(fpc);
	if (error)
		goto err_power_off;

	cdev_init(&fpc->cdev, &fpc_fops);
	fpc->cdev.kobj.parent = &fpc->dev.kobj;
	error = cdev_add(&fpc->cdev, fpc->dev.devt, 1);
	if (error)
		goto err_power_off;

	error = device_add(&fpc->dev);
	if (error)
		goto err_remove_cdev;

	error = request_irq(spi->irq, fpc1020_irq_handler,
			    IRQF_TRIGGER_FALLING /* FIXME: follow DTS */,
			    "fpc1020", fpc);
	if (error) {
		dev_err(&spi->dev, "failed to request irq %d: %d",
			spi->irq, error);
		goto err_device_del;
	}

	fpc->data_available = false;

	spi_set_drvdata(spi, fpc);

	error = fpc1020_sleep(fpc);
	if (error) {
		dev_err(&spi->dev,
			"failed to put device to sleep: %d\n", error);
		goto err_device_del;
	}

	error = sysfs_create_group(&spi->dev.kobj, &fpc1020_group);
	if (error) {
		dev_err(&spi->dev,
			"failed to create sysfs attributes: %d\n", error);
		goto err_device_del;
	}

	return 0;

err_device_del:
	fpc->dead = true;
	wake_up(&fpc->wait);
	device_del(&fpc->dev);
err_remove_cdev:
	cdev_del(&fpc->cdev);
err_power_off:
	regulator_bulk_disable(ARRAY_SIZE(fpc->supplies), fpc->supplies);
err_free_gpio:
	if (fpc->reset_gpio)
		gpiod_put(fpc->reset_gpio);
err_free_regulators:
	regulator_bulk_free(ARRAY_SIZE(fpc->supplies), fpc->supplies);
err_free_fpc:
	put_device(&fpc->dev);
err_free_minor:
	ida_simple_remove(&fpc_minor_ida, minor);
	return error;
}

static int fpc1020_spi_remove(struct spi_device *spi)
{
	struct fpc_data *fpc = spi_get_drvdata(spi);
	int minor = MINOR(fpc->dev.devt);

	sysfs_remove_group(&spi->dev.kobj, &fpc1020_group);

	/* Let userspace waiters know that device is being removed */
	fpc->dead = true;
	wake_up(&fpc->wait);

	device_del(&fpc->dev);
	cdev_del(&fpc->cdev);
	regulator_bulk_disable(ARRAY_SIZE(fpc->supplies), fpc->supplies);
	regulator_bulk_free(ARRAY_SIZE(fpc->supplies), fpc->supplies);
	if (fpc->reset_gpio)
		gpiod_put(fpc->reset_gpio);
	put_device(&fpc->dev);
	ida_simple_remove(&fpc_minor_ida, minor);
	return 0;
}

static int __maybe_unused fpc1020_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct fpc_data *fpc = spi_get_drvdata(spi);

	if (fpc1020_sleep(fpc))
		dev_warn(&spi->dev,
			 "failed to put device to sleep when suspending\n");

	return 0;
}

static int __maybe_unused fpc1020_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(fpc1020_pm_ops, fpc1020_suspend, fpc1020_resume);

#ifdef CONFIG_OF
static const struct of_device_id fpc_of_match[] = {
	{ .compatible = "fpc,fpc1020" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpc_of_match);
#endif

static struct spi_driver fpc1020_spi_driver = {
	.driver = {
		.name = "fpc1020",
		.pm = &fpc1020_pm_ops,
		.of_match_table = of_match_ptr(fpc_of_match),
	},
	.probe =	fpc1020_spi_probe,
	.remove =	fpc1020_spi_remove,
};

static int __init fpc1020_init(void)
{
	int error;

	fpc_class = class_create(THIS_MODULE, "fpc_sensor");
	if (IS_ERR(fpc_class)) {
		error = PTR_ERR(fpc_class);
		pr_err("failed to create fpc_sensor class: %d\n", error);
		return error;
	}

	fpc_major = register_chrdev(0, fpc_char_device_name, &fpc_fops);
	if (fpc_major < 0) {
		error = fpc_major;
		pr_err("failed to register char device: %d\n", error);
		goto err_destroy_class;
	}

	error = spi_register_driver(&fpc1020_spi_driver);
	if (error) {
		pr_err("failed to register FPC SPI driver: %d\n", error);
		goto err_unregister_chrdev;
	}

	return 0;

err_unregister_chrdev:
	unregister_chrdev(fpc_major, fpc_char_device_name);
err_destroy_class:
	class_destroy(fpc_class);
	return error;
}
module_init(fpc1020_init);

static void __exit fpc1020_exit(void)
{
	spi_unregister_driver(&fpc1020_spi_driver);
	unregister_chrdev(fpc_major, fpc_char_device_name);
	class_destroy(fpc_class);
}
module_exit(fpc1020_exit);

MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 touch sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:fpc1020");
