/*
 * Rockchip machine ASoC driver for boards using MAX98357A/RT5514/DA7219
 *
 * Copyright (c) 2016, ROCKCHIP CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/hdmi-codec.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "rockchip_i2s.h"
#include "../codecs/da7219.h"
#include "../codecs/da7219-aad.h"
#include "../codecs/rt5514.h"

#define DRV_NAME "rk3399-gru-sound"

#define SOUND_FS	256

static struct snd_soc_jack rockchip_sound_jack;

static const struct snd_soc_dapm_widget rockchip_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Speakers", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
};

static const struct snd_soc_dapm_route rockchip_dapm_routes[] = {
	/* Input Lines */
	{"MIC", NULL, "Headset Mic"},
	{"DMIC1L", NULL, "Int Mic"},
	{"DMIC1R", NULL, "Int Mic"},

	/* Output Lines */
	{"Headphones", NULL, "HPL"},
	{"Headphones", NULL, "HPR"},
	{"Speakers", NULL, "Speaker"},
};

static const struct snd_kcontrol_new rockchip_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Speakers"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static int rockchip_sound_max98357a_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int mclk;
	int ret;

	/* max98357a supports these sample rates */
	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		mclk = params_rate(params) * SOUND_FS;
		break;
	default:
		dev_err(rtd->card->dev, "%s() doesn't support this sample rate: %d\n",
				__func__, params_rate(params));
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, 0, mclk, 0);
	if (ret) {
		dev_err(rtd->card->dev, "%s() error setting sysclk to %u: %d\n",
				__func__, mclk, ret);
		return ret;
	}

	return 0;
}

static int rockchip_sound_rt5514_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int mclk;
	int ret;

	mclk = params_rate(params) * SOUND_FS;

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->card->dev, "Can't set cpu clock out %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5514_SCLK_S_MCLK,
				     mclk, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(rtd->card->dev, "%s() error setting sysclk to %u: %d\n",
				__func__, params_rate(params) * 512, ret);
		return ret;
	}

	return 0;
}

static int rockchip_sound_da7219_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk, ret;

	/* in bypass mode, the mclk has to be one of the frequencies below */
	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set cpu clock out %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock in %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, DA7219_SYSCLK_MCLK, 0, 0);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set pll sysclk mclk %d\n", ret);
		return ret;
	}

	return 0;
}

static int rockchip_sound_da7219_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec_dais[0]->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	/* We need default MCLK and PLL settings for the accessory detection */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 12288000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Init can't set codec clock in %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, DA7219_SYSCLK_MCLK, 0, 0);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Init can't set pll sysclk mclk %d\n", ret);
		return ret;
	}

	/* Enable Headset and 4 Buttons Jack detection */
	ret = snd_soc_card_jack_new(rtd->card, "Headset Jack",
				    SND_JACK_HEADSET | SND_JACK_LINEOUT |
				    SND_JACK_BTN_0 | SND_JACK_BTN_1 |
				    SND_JACK_BTN_2 | SND_JACK_BTN_3,
				    &rockchip_sound_jack, NULL, 0);

	if (ret) {
		dev_err(rtd->card->dev, "New Headset Jack failed! (%d)\n", ret);
		return ret;
	}

	snd_jack_set_key(rockchip_sound_jack.jack, SND_JACK_BTN_0, KEY_MEDIA);
	snd_jack_set_key(
		rockchip_sound_jack.jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
	snd_jack_set_key(
		rockchip_sound_jack.jack, SND_JACK_BTN_2, KEY_VOLUMEDOWN);
	snd_jack_set_key(
		rockchip_sound_jack.jack, SND_JACK_BTN_3, KEY_VOICECOMMAND);

	da7219_aad_jack_det(codec, &rockchip_sound_jack);

	return 0;
}


static int rockchip_sound_cdndp_hw_params(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk, ret;

	/* in bypass mode, the mclk has to be one of the frequencies below */
	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set cpu clock out %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_jack cdn_dp_card_jack;

static int rockchip_sound_cdndp_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_codec *codec = runtime->codec;
	int ret;

	/* enable jack detection */
	ret = snd_soc_card_jack_new(card, "DP Jack", SND_JACK_LINEOUT,
				    &cdn_dp_card_jack, NULL, 0);
	if (ret) {
		dev_err(card->dev, "Can't create DP Jack %d\n", ret);
		return ret;
	}

	return hdmi_codec_set_jack_detect(codec, &cdn_dp_card_jack);
}

static struct snd_soc_ops rockchip_sound_max98357a_ops = {
	.hw_params = rockchip_sound_max98357a_hw_params,
};

static struct snd_soc_ops rockchip_sound_rt5514_ops = {
	.hw_params = rockchip_sound_rt5514_hw_params,
};

static struct snd_soc_ops rockchip_sound_da7219_ops = {
	.hw_params = rockchip_sound_da7219_hw_params,
};

static struct snd_soc_ops rockchip_sound_cdndp_ops = {
	.hw_params = rockchip_sound_cdndp_hw_params,
};

enum {
	DAILINK_MAX98357A,
	DAILINK_RT5514,
	DAILINK_DA7219,
	DAILINK_CDNDP,
	DAILINK_RT5514_DSP,
	DAILINK_SIZE,
};

static struct snd_soc_dai_link rockchip_dailinks[] = {
	[DAILINK_MAX98357A] = {
		.name = "MAX98357A",
		.stream_name = "MAX98357A PCM",
		.codec_dai_name = "HiFi",
		.ops = &rockchip_sound_max98357a_ops,
		/* set max98357a as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
	[DAILINK_RT5514] = {
		.name = "RT5514",
		.stream_name = "RT5514 PCM",
		.codec_dai_name = "rt5514-aif1",
		.ops = &rockchip_sound_rt5514_ops,
		/* set rt5514 as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
	[DAILINK_DA7219] = {
		.name = "DA7219",
		.stream_name = "DA7219 PCM",
		.codec_dai_name = "da7219-hifi",
		.init = rockchip_sound_da7219_init,
		.ops = &rockchip_sound_da7219_ops,
		/* set da7219 as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
	[DAILINK_CDNDP] = {
		.name = "DP",
		.stream_name = "DP PCM",
		.codec_dai_name = "i2s-hifi",
		.init = rockchip_sound_cdndp_init,
		.ops = &rockchip_sound_cdndp_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
	/* RT5514 DSP for voice wakeup via spi bus */
	[DAILINK_RT5514_DSP] = {
		.name = "RT5514 DSP",
		.stream_name = "Wake on Voice",
		/* cpu_dai_name is not set since rt5514-spi has only one DAI */
		.codec_dai_name = "rt5514-dspbuffer",
		.ignore_suspend = 1,
	},
};

static struct snd_soc_card rockchip_sound_card = {
	.name = "rk3399-gru-sound",
	.owner = THIS_MODULE,
	.dai_link = rockchip_dailinks,
	.num_links =  ARRAY_SIZE(rockchip_dailinks),
	.dapm_widgets = rockchip_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rockchip_dapm_widgets),
	.dapm_routes = rockchip_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rockchip_dapm_routes),
	.controls = rockchip_controls,
	.num_controls = ARRAY_SIZE(rockchip_controls),
};

static int rockchip_sound_dailink_init(struct platform_device *pdev, int index)
{
	struct device_node *cpu, *codec;

	cpu = of_parse_phandle(pdev->dev.of_node, "rockchip,cpu", index);
	if (!cpu) {
		dev_err(&pdev->dev, "'rockchip,cpu'[%d] invalid\n", index);
		return -EINVAL;
	}

	codec = of_parse_phandle(pdev->dev.of_node, "rockchip,codec", index);
	if (!codec) {
		dev_err(&pdev->dev, "'rockchip,codec'[%d] invalid\n", index);
		return -EINVAL;
	}

	rockchip_dailinks[index].platform_of_node = cpu;
	rockchip_dailinks[index].cpu_of_node = cpu;
	rockchip_dailinks[index].codec_of_node = codec;

	return 0;
}

static int rockchip_sound_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &rockchip_sound_card;
	int i, ret;

	for (i = 0; i < DAILINK_SIZE; i++) {
		ret = rockchip_sound_dailink_init(pdev, i);
		if (ret)
			return ret;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);

	return ret;
}

static const struct of_device_id rockchip_sound_of_match[] = {
	{ .compatible = "rockchip,rk3399-gru-sound", },
	{},
};

static struct platform_driver rockchip_sound_driver = {
	.probe = rockchip_sound_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rockchip_sound_of_match,
#ifdef CONFIG_PM
		.pm = &snd_soc_pm_ops,
#endif
	},
};

module_platform_driver(rockchip_sound_driver);

MODULE_AUTHOR("Xing Zheng <zhengxing@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_sound_of_match);
