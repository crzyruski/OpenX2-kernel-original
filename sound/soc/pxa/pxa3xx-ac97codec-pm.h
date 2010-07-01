#ifndef __AC97CODEC_PM__
#define __AC97CODEC_PM__

typedef enum {
	CODEC_POWER_ON = 0,
	CODEC_LOWPOWER,
	CODEC_POWER_OFF,
	CODEC_READY_LOWPOWER,
} codec_state_t;

typedef enum {
	CODEC_SUB_POWER_ON = 0,
	CODEC_SUB_LOWPOWER,
	CODEC_SUB_POWER_OFF,
} codec_sub_state_t;

extern struct snd_ac97_bus_ops soc_ac97_ops;
extern int codec_client;
extern int set_codec_sub_state(int client, int state);
extern int register_codec(int *client);
extern int unregister_codec(int client);

#endif

