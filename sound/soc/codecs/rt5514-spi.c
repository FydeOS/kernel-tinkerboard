/*
 * rt5514-spi.c  --  RT5514 SPI driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "rt5514.h"
#include "rt5514-spi.h"



static int rt5514_spi_burst_read(struct rt5514_dsp *dsp, unsigned int addr,
		u8 *rxbuf, size_t len);
static int rt5514_spi_burst_write(struct rt5514_dsp *dsp, u32 addr,
		const u8 *txbuf, size_t len);

static const struct snd_pcm_hardware rt5514_spi_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= 0x20000 / 8,
	.periods_min		= 8,
	.periods_max		= 8,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= 0x20000,
};

static struct snd_soc_dai_driver rt5514_spi_dai = {
	.name = "rt5514-dsp-cpu-dai",
	.id = 0,
	.capture = {
		.stream_name = "DSP Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static void rt5514_spi_start(struct rt5514_dsp *dsp)
{

	int ret;
	const struct firmware *fw;
	struct regmap *regmap = dsp->priv->i2c_regmap;

	/* Reset */
	regmap_write(regmap, 0x18002000, 0x000010ec);
	/* LDO_I_limit */
	regmap_write(regmap, 0x18002200, 0x00028604);
	/* I2C bypass enable */
	regmap_write(regmap, 0xfafafafa, 0x00000001);
	/* mini-core reset */
	regmap_write(regmap, 0x18002f00, 0x0005514b);
	regmap_write(regmap, 0x18002f00, 0x00055149);
	/* I2C bypass disable */
	regmap_write(regmap, 0xfafafafa, 0x00000000);
	/* PIN config */
	regmap_write(regmap, 0x18002070, 0x00000040);
	/* PLL3(QN)=RCOSC*(10+2) */
	regmap_write(regmap, 0x18002240, 0x0000000a);
	/* PLL3 source=RCOSC, fsi=rt_clk */
	regmap_write(regmap, 0x18002100, 0x0000000b);
	/* Power on RCOSC, pll3 */
	regmap_write(regmap, 0x18002004, 0x00808b81);
	/* DSP clk source = pll3, ENABLE DSP clk */
	regmap_write(regmap, 0x18002f08, 0x00000005);
	/* Enable DSP clk auto switch */
	regmap_write(regmap, 0x18001114, 0x00000001);
	/* Reduce DSP power */
	regmap_write(regmap, 0x18001118, 0x00000001);

	ret = request_firmware(&fw, RT5514_FIRMWARE1, dsp->dev);
	if (ret) {
		dev_err(dsp->dev, "request_firmware failed %d: "
				RT5514_FIRMWARE1, ret);
		return;
	}
	rt5514_spi_burst_write(dsp, 0x4ff60000, fw->data,
			round_up(fw->size, 8));
	release_firmware(fw);

	ret = request_firmware(&fw, RT5514_FIRMWARE2, dsp->dev);
	if (ret) {
		dev_err(dsp->dev, "request_firmware failed %d: "
				RT5514_FIRMWARE2, ret);
		return;
	}
	rt5514_spi_burst_write(dsp, 0x4ffc0000, fw->data,
			round_up(fw->size, 8));
	release_firmware(fw);

	/* DSP run */
	regmap_write(regmap, 0x18002f00, 0x00055148);

	dsp->priv->state = RT5514_DSP_ARMED;
}

static int rt5514_spi_prepare_to_stream(struct rt5514_dsp *dsp)
{
	u8 buf[8];

	/**
	 * The address area x1800XXXX is the register address, and it cannot
	 * support spi burst read perfectly. So we use the spi burst read
	 * individually to make sure the data correctly.
	*/
	rt5514_spi_burst_read(dsp, RT5514_BUFFER_VOICE_BASE, (u8 *)&buf,
		sizeof(buf));
	dsp->buf_base = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;

	rt5514_spi_burst_read(dsp, RT5514_BUFFER_VOICE_LIMIT, (u8 *)&buf,
		sizeof(buf));
	dsp->buf_limit = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;

	rt5514_spi_burst_read(dsp, RT5514_BUFFER_VOICE_RP, (u8 *)&buf,
		sizeof(buf));
	dsp->buf_rp = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;

	rt5514_spi_burst_read(dsp, RT5514_BUFFER_VOICE_SIZE, (u8 *)&buf,
		sizeof(buf));
	dsp->buf_size = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;

	if (dsp->buf_base && dsp->buf_limit && dsp->buf_rp && dsp->buf_size) {
		dsp->dsp_offset = 0;
		dsp->priv->state = RT5514_DSP_STREAM;
		dev_info(dsp->dev, "buf_base=0x%x, buf_limit=0x%x, buf_rp=0x%x"
				", buf_size=%zu. dma_bytes=%zu, "
				"buffer_size=%luF, periods=%u, period_size=%luF",
				dsp->buf_base, dsp->buf_limit, dsp->buf_rp,
				dsp->buf_size,
				dsp->substream->runtime->dma_bytes,
				dsp->substream->runtime->buffer_size,
				dsp->substream->runtime->periods,
				dsp->substream->runtime->period_size);

		if (dsp->buf_base != dsp->buf_rp)
			dev_err(dsp->dev, "buf_base != buf_rp");
		if (dsp->buf_size != dsp->buf_limit - dsp->buf_base)
			dev_err(dsp->dev, "buf_size != buf_limit - buf_base");
		return 0;
	}

	queue_delayed_work(system_freezable_wq, &dsp->work, 5);
	return -EAGAIN;
}

static void rt5514_spi_stream_zero(struct rt5514_dsp *dsp, u32 size)
{
	struct snd_pcm_runtime *runtime = dsp->substream->runtime;
	size_t target = dsp->dma_offset + size;

	memset(runtime->dma_area + dsp->dma_offset, 64,
			min(target, runtime->dma_bytes) - dsp->dma_offset);
	if (target > runtime->dma_bytes)
		memset(runtime->dma_area, 64, target - runtime->dma_bytes);

	if (target >= runtime->dma_bytes)
		target -= runtime->dma_bytes;
	dsp->dma_offset = target;
}

static void rt5514_spi_stream_data(struct rt5514_dsp *dsp, u32 addr, u32 size)
{
	struct snd_pcm_runtime *runtime = dsp->substream->runtime;
	size_t target = dsp->dma_offset + size;
	size_t block_size = min(target, runtime->dma_bytes) - dsp->dma_offset;

	rt5514_spi_burst_read(dsp, addr, runtime->dma_area + dsp->dma_offset,
			block_size);
	if (target > runtime->dma_bytes)
		rt5514_spi_burst_read(dsp, addr + block_size, runtime->dma_area,
				target - runtime->dma_bytes);

	if (target >= runtime->dma_bytes)
		target -= runtime->dma_bytes;
	dsp->dma_offset = target;
}

static void rt5514_spi_stream(struct rt5514_dsp *dsp)
{
	struct snd_pcm_runtime *runtime;
	unsigned long delay_ms = RT5514_DSP_STREAM_DELAY_MS;
	size_t period_bytes, truncated_bytes = 0;
	size_t copy_bytes = 0;

	if (!dsp->substream) {
		dev_err(dsp->dev, "No pcm substream\n");
		return;
	}

	runtime = dsp->substream->runtime;
	period_bytes = snd_pcm_lib_period_bytes(dsp->substream);

	if (frames_to_bytes(runtime, snd_pcm_capture_hw_avail(runtime)) <
			period_bytes)
		goto done;

	if (dsp->dsp_offset < dsp->buf_size) {
		copy_bytes = min(period_bytes, dsp->buf_size - dsp->dsp_offset);

		if (dsp->buf_rp + copy_bytes <= dsp->buf_limit) {
			rt5514_spi_stream_data(dsp, dsp->buf_rp, copy_bytes);
			if (dsp->buf_rp + copy_bytes == dsp->buf_limit)
				dsp->buf_rp = dsp->buf_base;
			else
				dsp->buf_rp += copy_bytes;
		} else {
			truncated_bytes = dsp->buf_limit - dsp->buf_rp;
			rt5514_spi_stream_data(dsp, dsp->buf_rp,
					truncated_bytes);
			rt5514_spi_stream_data(dsp, dsp->buf_base,
					copy_bytes - truncated_bytes);

			dsp->buf_rp = dsp->buf_base +
					copy_bytes - truncated_bytes;
		}
		dsp->dsp_offset += copy_bytes;
		if (dsp->dsp_offset == dsp->buf_size) {
			dev_info(dsp->dev, "dsp_offset -> buf_size(%zu), dma_offset=%zu",
					dsp->buf_size, dsp->dma_offset);
		}
	}

	if (period_bytes - copy_bytes) {
		rt5514_spi_stream_zero(dsp, period_bytes - copy_bytes);
		delay_ms = runtime->period_size * 1000 / runtime->rate;
	}


	snd_pcm_period_elapsed(dsp->substream);
done:
	queue_delayed_work(system_freezable_wq, &dsp->work,
			msecs_to_jiffies(delay_ms));
}

/*
 * A delayed_work that configures the codec audio path for hotwording, loads
 * DSP firmware, streams audio samples after a hotword is detected, and stops
 * the DSP when requested.
 */
static void rt5514_spi_work(struct work_struct *work)
{
	struct rt5514_dsp *dsp = container_of(work, struct rt5514_dsp,
			work.work);

	mutex_lock(&dsp->priv->dsp_lock);
	switch (dsp->priv->state) {
	case RT5514_DSP_START:
		rt5514_spi_start(dsp);
		break;
	case RT5514_DSP_TRIGGERED:
		dev_info(dsp->dev, "Hotword fired, prepare to stream\n");
		if (rt5514_spi_prepare_to_stream(dsp))
			break;
		/* Fall through to start streaming */
	case RT5514_DSP_STREAM:
		rt5514_spi_stream(dsp);
		break;
	default:
		break;
	}
	mutex_unlock(&dsp->priv->dsp_lock);
}

/* PCM for streaming audio from the DSP buffer */
static int rt5514_spi_pcm_open(struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rt5514_spi_pcm_hardware);

	return 0;
}

static int rt5514_spi_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5514_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);
	int ret;

	ret = snd_pcm_lib_alloc_vmalloc_buffer(substream,
			params_buffer_bytes(hw_params));
	dsp->substream = substream;
	dsp->priv = snd_soc_codec_get_drvdata(rtd->codec);

	if (ret < 0)
		return ret;

	mutex_lock(&dsp->priv->dsp_lock);
	switch (dsp->priv->state) {
	case RT5514_IDLE:
		dsp->priv->state = RT5514_DSP_START;
		break;
	case RT5514_AIF1_ON:
		dsp->priv->state = RT5514_AIF1_ON_DSP_PENDING;
		break;
	default:
		break;
	}
	mutex_unlock(&dsp->priv->dsp_lock);
	return ret;
}

static int rt5514_spi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5514_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);

	cancel_delayed_work_sync(&dsp->work);

	mutex_lock(&dsp->priv->dsp_lock);
	dsp->substream = NULL;
	mutex_unlock(&dsp->priv->dsp_lock);

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int rt5514_spi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5514_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);

	mutex_lock(&dsp->priv->dsp_lock);
	dsp->dma_offset = 0;
	dsp->dsp_offset = 0;
	mutex_unlock(&dsp->priv->dsp_lock);

	return 0;
}

static int rt5514_spi_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5514_dsp *dsp = snd_soc_platform_get_drvdata(rtd->platform);

	if (cmd == SNDRV_PCM_TRIGGER_START)
		queue_delayed_work(system_freezable_wq, &dsp->work, 0);

	return 0;
}

static snd_pcm_uframes_t rt5514_spi_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_platform_get_drvdata(rtd->platform);

	return bytes_to_frames(runtime, rt5514_dsp->dma_offset);
}

static const struct snd_pcm_ops rt5514_spi_pcm_ops = {
	.open		= rt5514_spi_pcm_open,
	.hw_params	= rt5514_spi_hw_params,
	.hw_free	= rt5514_spi_hw_free,
	.trigger	= rt5514_spi_trigger,
	.prepare	= rt5514_spi_prepare,
	.pointer	= rt5514_spi_pcm_pointer,
	.mmap		= snd_pcm_lib_mmap_vmalloc,
	.page		= snd_pcm_lib_get_vmalloc_page,
};

static irqreturn_t rt5514_spi_irq(int irq, void *data)
{
	struct rt5514_dsp *dsp = data;

	if (!dsp->priv)
		return IRQ_NONE;

	mutex_lock(&dsp->priv->dsp_lock);
	if (dsp->priv->state == RT5514_DSP_ARMED) {
		dsp->priv->state = RT5514_DSP_TRIGGERED;
		queue_delayed_work(system_freezable_wq, &dsp->work, 0);
	}
	mutex_unlock(&dsp->priv->dsp_lock);
	return IRQ_HANDLED;
}

static int rt5514_spi_pcm_probe(struct snd_soc_platform *platform)
{
	int irq, ret;
	struct rt5514_dsp *dsp;

	dsp = devm_kzalloc(platform->dev, sizeof(*dsp), GFP_KERNEL);
	if (!dsp)
		return -ENOMEM;

	dsp->dev = platform->dev;

	INIT_DELAYED_WORK(&dsp->work, rt5514_spi_work);
	snd_soc_platform_set_drvdata(platform, dsp);

	irq = to_spi_device(dsp->dev)->irq;
	if (!irq) {
		dev_err(dsp->dev, "No IRQ assigned.\n");
		return -ENODEV;
	}
	ret = devm_request_threaded_irq(dsp->dev, irq, NULL, rt5514_spi_irq,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT, "rt5514", dsp);
	if (ret)
		dev_err(dsp->dev, "Failed to request IRQ: %d\n", ret);

	return ret;
}

static struct snd_soc_platform_driver rt5514_spi_platform = {
	.probe = rt5514_spi_pcm_probe,
	.ops = &rt5514_spi_pcm_ops,
};

static const struct snd_soc_component_driver rt5514_spi_dai_component = {
	.name		= "rt5514-spi-dai",
};

/**
 * rt5514_spi_burst_read - Read data from SPI by rt5514 address.
 * @addr: Start address.
 * @rxbuf: Data Buffer for reading.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
static int rt5514_spi_burst_read(struct rt5514_dsp *dsp, unsigned int addr,
				 u8 *rxbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_READ;
	int status;
	u8 write_buf[8];
	unsigned int i, end, offset = 0;

	struct spi_message message;
	struct spi_transfer x[3];

	while (offset < len) {
		if (offset + RT5514_SPI_BUF_LEN <= len)
			end = RT5514_SPI_BUF_LEN;
		else
			end = len % RT5514_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		spi_message_init(&message);
		memset(x, 0, sizeof(x));

		x[0].len = 5;
		x[0].tx_buf = write_buf;
		spi_message_add_tail(&x[0], &message);

		x[1].len = 4;
		x[1].tx_buf = write_buf;
		spi_message_add_tail(&x[1], &message);

		x[2].len = end;
		x[2].rx_buf = rxbuf + offset;
		spi_message_add_tail(&x[2], &message);

		status = spi_sync(to_spi_device(dsp->dev), &message);

		if (status)
			return false;

		offset += RT5514_SPI_BUF_LEN;
	}

	for (i = 0; i < len; i += 8) {
		write_buf[0] = rxbuf[i + 0];
		write_buf[1] = rxbuf[i + 1];
		write_buf[2] = rxbuf[i + 2];
		write_buf[3] = rxbuf[i + 3];
		write_buf[4] = rxbuf[i + 4];
		write_buf[5] = rxbuf[i + 5];
		write_buf[6] = rxbuf[i + 6];
		write_buf[7] = rxbuf[i + 7];

		rxbuf[i + 0] = write_buf[7];
		rxbuf[i + 1] = write_buf[6];
		rxbuf[i + 2] = write_buf[5];
		rxbuf[i + 3] = write_buf[4];
		rxbuf[i + 4] = write_buf[3];
		rxbuf[i + 5] = write_buf[2];
		rxbuf[i + 6] = write_buf[1];
		rxbuf[i + 7] = write_buf[0];
	}

	return true;
}

/**
 * rt5514_spi_burst_write - Write data to SPI by rt5514 address.
 * @addr: Start address.
 * @txbuf: Data Buffer for writng.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
static int rt5514_spi_burst_write(struct rt5514_dsp *dsp, u32 addr,
				  const u8 *txbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_WRITE;
	u8 *write_buf;
	unsigned int i, end, offset = 0;

	write_buf = kmalloc(RT5514_SPI_BUF_LEN + 6, GFP_KERNEL);

	if (write_buf == NULL)
		return -ENOMEM;

	while (offset < len) {
		if (offset + RT5514_SPI_BUF_LEN <= len)
			end = RT5514_SPI_BUF_LEN;
		else
			end = len % RT5514_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		for (i = 0; i < end; i += 8) {
			write_buf[i + 12] = txbuf[offset + i + 0];
			write_buf[i + 11] = txbuf[offset + i + 1];
			write_buf[i + 10] = txbuf[offset + i + 2];
			write_buf[i +  9] = txbuf[offset + i + 3];
			write_buf[i +  8] = txbuf[offset + i + 4];
			write_buf[i +  7] = txbuf[offset + i + 5];
			write_buf[i +  6] = txbuf[offset + i + 6];
			write_buf[i +  5] = txbuf[offset + i + 7];
		}

		write_buf[end + 5] = spi_cmd;

		spi_write(to_spi_device(dsp->dev), write_buf, end + 6);

		offset += RT5514_SPI_BUF_LEN;
	}

	kfree(write_buf);

	return 0;
}

static int rt5514_spi_probe(struct spi_device *spi)
{
	int ret;

	ret = devm_snd_soc_register_platform(&spi->dev, &rt5514_spi_platform);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register platform.\n");
		return ret;
	}

	ret = devm_snd_soc_register_component(&spi->dev,
					      &rt5514_spi_dai_component,
					      &rt5514_spi_dai, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register component.\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id rt5514_of_match[] = {
	{ .compatible = "realtek,rt5514", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5514_of_match);

static struct spi_driver rt5514_spi_driver = {
	.driver = {
		.name = "rt5514",
		.of_match_table = of_match_ptr(rt5514_of_match),
	},
	.probe = rt5514_spi_probe,
};
module_spi_driver(rt5514_spi_driver);

MODULE_DESCRIPTION("RT5514 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
