#ifndef __YUHUA_BOARD_DEV_INFO_H__
#define __YUHUA_BOARD_DEV_INFO_H__

struct yuhua_board_info {
	int camera_detect;
	int camera2_detect;
	int wifi_detect;
	int sdcard_detect;
	int sdcard_capacity;
	int fm_detect;
	int gsensor_detect;
	int rtc_detect;
	int touch_detect;
	char* camera_id; /* indicate which camera attached to board */
};

extern struct yuhua_board_info g_yuhua_board_info;

#define set_camera_detect(detect) (g_yuhua_board_info.camera_detect=detect)
#define set_camera2_detect(detect) (g_yuhua_board_info.camera2_detect=detect)
#define set_wifi_detect(detect) (g_yuhua_board_info.wifi_detect=detect)
#define set_sdcard_detect(detect) (g_yuhua_board_info.sdcard_detect=detect)
#define set_sdcard_capacity(capacity) (g_yuhua_board_info.sdcard_capacity=capacity)
#define set_fm_detect(detect) (g_yuhua_board_info.fm_detect=detect)
#define set_gsensor_detect(detect) (g_yuhua_board_info.gsensor_detect=detect)
#define set_rtc_detect(detect) (g_yuhua_board_info.rtc_detect=detect)
#define set_camera_id(id) (g_yuhua_board_info.camera_id=id)
#define set_touch_detect(detect) (g_yuhua_board_info.touch_detect=detect)

#define is_camera_detect() (g_yuhua_board_info.camera_detect)
#define is_camera2_detect() (g_yuhua_board_info.camera2_detect)
#define is_wifi_detect() (g_yuhua_board_info.wifi_detect)
#define is_sdcard_detect() (g_yuhua_board_info.sdcard_detect)
#define is_fm_detect() (g_yuhua_board_info.fm_detect)
#define is_gsensor_detect() (g_yuhua_board_info.gsensor_detect)
#define is_rtc_detect() (g_yuhua_board_info.rtc_detect)
#define is_touch_detect() (g_yuhua_board_info.touch_detect)

#define is_camera_id(id) (g_yuhua_board_info.camera_detect && (!strcmp(g_yuhua_board_info.camera_id, id)))

#endif

