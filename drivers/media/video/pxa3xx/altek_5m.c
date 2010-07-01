/* 
	Camera driver for Altek 5M
	Copyright@yuhuatel 2009
	Exp by frank.du 
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/rtc.h>
	
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/littleton.h>
#include <mach/micco.h>
#ifdef CONFIG_YUHUA_MISC_DEV	
#include <mach/yuhua_board_dev_info.h>
#endif
#include <mach/pxafb.h>

#include "altek_5m.h"
#ifdef CONFIG_CI_CAMERA_GPIO_I2C
#include "i2c-gpio.h"
#endif

static struct i2c_client *g_client;
static struct delayed_work altek5m_work;
static altek5m_work_type g_workType;
static altek5m_status g_altek5m_status;
static unsigned long g_altek5m_init_jiffles;
static altek5m_size g_altek5m_preSize;
static u32 g_altek5m_i2c_err_cnt;
static int g_altek5m_video_mode;

#define altek5m_MAX_I2C_ERR (4)

static void altek5mhw_power_down(void);

static int DomingoIOSetGet(u8* inCmd, int inCmdLen, u8* rsp, int rspLen, int cmdDelay)
{
	int ret = 0, step = 0, retry = 3;
	altek5m_dbg("cmd 0x%x, len %d, rspLen %d\n", inCmd[0], inCmdLen, rspLen);
	
	if (g_altek5m_i2c_err_cnt>altek5m_MAX_I2C_ERR) {
		return -EIO;
	}

	retry = 3;
	if (inCmdLen>0) {
		while ((retry--) > 0) {
#ifdef CONFIG_CI_CAMERA_GPIO_I2C
			ret = i2c_gpio_send(g_client->addr, inCmd, inCmdLen);
#else
			ret = i2c_master_send(g_client, inCmd, inCmdLen);
#endif
			if (ret>=0)
				break;
			msleep(10);
			altek5m_err("step %d retry %d cmd 0x%x\n",step, retry, inCmd[0]);
		}
	}
	if (ret < 0)	
		goto out;

	msleep(1);
	step++;
	retry = 2;
	if (rspLen>0) {
		while ((retry--) > 0) {		
#ifdef CONFIG_CI_CAMERA_GPIO_I2C
			ret = i2c_gpio_recv(g_client->addr, rsp, rspLen);
#else
			ret = i2c_master_recv(g_client, rsp, rspLen);
#endif
			if (ret>=0)
				break;
			msleep(10);
			altek5m_err("step %d retry %d cmd 0x%x\n",step, retry, inCmd[0]);
		}
	}

out:
	if (ret < 0) {
		g_altek5m_i2c_err_cnt++;		
		altek5m_err("cmd 0x%x, step %d, ret %d, %d\n", inCmd[0], step, ret, g_altek5m_i2c_err_cnt);
		if (g_altek5m_i2c_err_cnt>=altek5m_MAX_I2C_ERR) {
			altek5m_err("power off altek 5m as i2c err\n");
			altek5mhw_power_down(); /* release i2c bus */
		} else
			msleep(50);
	} else {
		g_altek5m_i2c_err_cnt = 0;
	}
	
	return ret;
}

static int DomingoSetAndWait(u8* cmdbuf, int length, int cmd_delay_ms)
{
    u8 status[4];
    int ret, retry;

    ret = 0;
    retry = 5;
    while (retry--) {
        ret = DomingoIOSetGet(cmdbuf, length, NULL, 0, 1);
        if (ret>=0)
            break;

        altek5m_info("cmd 0x%x, len %d, ret %d, retry %dd\n", cmdbuf[0], length, ret, retry);
        msleep(50);
    }

    //if (cmd_delay_ms>50)
         msleep(cmd_delay_ms);

    if (ret<0) {
        altek5m_err("cmd 0x%x, len %d, ret %d, retry %dd\n", cmdbuf[0], length, ret, retry);
        return ret;
    }

    // check the command status
    retry = 5;
    status[1] = status[2] = 0xff;
    status[0] = 0x09; // Get status of last executed command
    while (retry--) {
        DomingoIOSetGet(status, 1, status+1, 2, cmd_delay_ms);
        if (status[1] == cmdbuf[0])
            break;

        altek5m_err("[0x%x]: busy (%x, %d)\n", cmdbuf[0], status[1], status[2]);
        msleep(20);
    }

    if (status[2] != 0) {
        altek5m_err("[0x%x]: status error (%d)\n", cmdbuf[0], status[2]);
        return -EIO;
    }

    return 0;
}

#define UINT8 	u8
//========================
// freq : 12->13M   0->6.5M
static int Domingo_SetFreq(UINT8 a_FreqIndex)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x72; //OPCODE_SETVIFFREQ
    cmdbuf[1] = a_FreqIndex;  // 12->13M  00->6.5M
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

static int Domingo_SetVIFTiming(int boJpegMode)
{
    UINT8 cmdbuf[10];

    // set Vsync low active and Hsync high active
    cmdbuf[0] = 0x5A;// Set VIF mode
    cmdbuf[1] = 1;   // YCC422 = cbYcrY
    cmdbuf[2] = 1;   // YSync valid mode       ___
    cmdbuf[3] = 0;   // VSync active low    __| B |_________
    cmdbuf[4] = 1;   // HSync valid mode    __     _________
    cmdbuf[5] = 1;   // HSync active high     |_B_|
    if (boJpegMode)
        cmdbuf[6] = 16;
    else
        cmdbuf[6] = 0x89; // 0->defaut(128)  1~0xff-> 4~1020 blanking pixel  4->16 pixel
    cmdbuf[7] = 4;   // QV/JPEG dummy line 4 is means add 4 dummy line
    cmdbuf[8] = 0;   // reserved
    DomingoSetAndWait(cmdbuf, 9, 5);

    return 0;
}

static int Domingo_SetTime(UINT8 year, UINT8 mon, UINT8 day, UINT8 hour, UINT8 min, UINT8 sec)
{
    UINT8 cmdbuf[8];

    //set time to Domingo module
    cmdbuf[0] = 0x4E; //OPCODE_SETTIME
    cmdbuf[1] = sec;
    cmdbuf[2] = min;
    cmdbuf[3] = hour;
    cmdbuf[4] = day;
    cmdbuf[5] = mon;
    cmdbuf[6] = year;
    return DomingoSetAndWait(cmdbuf, 7, 5);
}

//===================================
static int Domingo_SetMakerModelName(UINT8 a_index, char *a_strMaker)
{
    UINT8 cmdbuf[18] = {0};
 
	cmdbuf[0] = 0x70; // OPCODE_SETEXIFSTRING
	cmdbuf[1] = a_index;    // 0 = Maker name, 1 = Model name
	memcpy(cmdbuf+2, a_strMaker, strlen(a_strMaker));
	return DomingoSetAndWait(cmdbuf, 18, 5); 
}

//========================
//extern void DomingoWaitVSync(void);
static int Domingo_ShowFocusFrame(UINT8 a_frame)
{
    int boWait = 0;
    UINT8 cmdbuf[5];
    int ret;

    if (a_frame==2)
        boWait = 1;

    // change to new style of Focus frame
    //if ((a_frame==1 || a_frame==2))
        //a_frame += 2;

    cmdbuf[0] = 0x74;
    cmdbuf[1] = a_frame;
    ret = DomingoSetAndWait(cmdbuf, 2, 10);
    
    if (boWait) { // wait 1 VD just for synchronize display
        //DomingoWaitVSync();
        //DomingoWaitVSync();
        //msleep(150); /* about 2 frame */
    }

    return ret;
}

//========================  // 1->QVGA mode  2->VGA mode
static int Domingo_SetQuickViewSize(UINT8 a_Size)
{
    UINT8 cmdbuf[5];
    int ret;

    // enable Quick view
    cmdbuf[0] = 0x46;       // OP-Code
    cmdbuf[1] = a_Size;
    ret = DomingoSetAndWait(cmdbuf, 2, 5);
    return ret;
}

//======================== anti-flicker
static int Domingo_SetBanding(UINT8 a_banding)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x14; // OP-Code
    cmdbuf[1] = a_banding;
    return DomingoSetAndWait(cmdbuf, 2, 50);
}

//========================
static int Domingo_SetAFMode(UINT8 a_range, UINT8 a_mode)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x24; // OP-Code
    cmdbuf[1] = a_range;
    cmdbuf[2] = a_mode;
    return DomingoSetAndWait(cmdbuf, 3, 5);
}

//======================== video anti-shaking
static int Domingo_SetEIS(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x28; // OP-Code
    cmdbuf[1] = a_flag;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetAEAWBWindow(UINT8 a_awb, UINT8 a_ae)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x3E; // OP-Code
    cmdbuf[1] = a_awb;
    cmdbuf[2] = a_ae;
    return DomingoSetAndWait(cmdbuf, 3, 5);
}

//========================
static int Domingo_SetISO(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x16; // OP-Code
    cmdbuf[1] = a_flag;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//======================== light mode
static int Domingo_SetAdaptiveLighting(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x2c; // OP-Code
    cmdbuf[1] = a_flag;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetBlurDetect(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x52; // OP-Code
    cmdbuf[1] = a_flag;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetPreviewSize(UINT8 a_Size)
{
    UINT8 cmdbuf[5];
	
    cmdbuf[0] = 0x34; // OP-Code
    cmdbuf[1] = a_Size;
    return DomingoSetAndWait(cmdbuf, 2, 50);
}

//========================
static int Domingo_SetCaptureSize(UINT8 a_Size)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x32; // OP-Code
    cmdbuf[1] = a_Size;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetVideoSize(UINT8 a_Size)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x4C;   //OPCODE_SETVICSIZE
    cmdbuf[1] = a_Size;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetTimeStamp(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x50; // OP-Code
    cmdbuf[1] = a_flag;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetWB(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x1C; // OP-Code
    cmdbuf[1] = a_flag;
    return DomingoSetAndWait(cmdbuf, 2, 50);
}

static int Domingo_EnableResetSettingOfSceneMode()
{
    UINT8 cmdbuf[5]; 

    cmdbuf[0] = 0x8c; // OP-Code
    cmdbuf[1] = 1;
    cmdbuf[2] = 1;
    cmdbuf[3] = 1;
    cmdbuf[4] = 1;
    return DomingoSetAndWait(cmdbuf, 5, 5);
}

//========================
static int Domingo_SetScene(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x2E; // OP-Code
    cmdbuf[1] = a_flag;
    DomingoSetAndWait(cmdbuf, 2, 20);
    Domingo_EnableResetSettingOfSceneMode();
    return 0;
}

//========================
static int Domingo_SetEffect(UINT8 a_flag)
{
    UINT8 cmdbuf[5];

    cmdbuf[0] = 0x1E; // OP-Code
    cmdbuf[1] = a_flag;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetDigitalZoom(UINT8 a_zoom)
{
  UINT8 cmdbuf[2];
  UINT8 outbuf;
  int ret;

  cmdbuf[0] = 0x27;
  outbuf = 0xFF;
  DomingoIOSetGet(cmdbuf, 1, &outbuf, 1, 0);

  if (outbuf != a_zoom)
  {
    cmdbuf[0] = 0x26;
    cmdbuf[1] = a_zoom;
    ret = DomingoSetAndWait(cmdbuf, 2, 250);
  }
  else
    ret = 77;

  return ret;
}

//======================== exposue compensation
static int Domingo_SetEvComp(UINT8 a_EvComp)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x12; // OP-Code
    cmdbuf[1] = a_EvComp;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetFlashMode(UINT8 a_value)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x38; // OP-Code
    cmdbuf[1] = a_value;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetFlashType(UINT8 a_value)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x62; // OP-Code
    cmdbuf[1] = a_value;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}
//========================
static int Domingo_SetRedeyeRemoval(UINT8 a_value)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x64; // OP-Code
    cmdbuf[1] = a_value;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}
//========================
static int Domingo_SetPreFlash(UINT8 a_value)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x66; // OP-Code
    cmdbuf[1] = a_value;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}
//========================
static int Domingo_SetFlashLevel(UINT8 a_value)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x6a; // OP-Code
    cmdbuf[1] = a_value;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_GetBlurStatus(UINT8 *a_pu8Status)
{
    int retCode;
    UINT8 cmdbuf[2];

    //get blur status
    cmdbuf[0] = 0x53;
    retCode = DomingoIOSetGet(cmdbuf, 1, a_pu8Status, 1, 1);
    //altek5m_info("%d", *a_pu8Status);

    return retCode;
}

//========================
static int Domingo_SetFrameRate(UINT8 a_rate)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x0E; //OPCODE
    cmdbuf[1] = a_rate;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//========================
static int Domingo_SetAFLEDMode(UINT8 a_mode)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x68; // OP-Code
    cmdbuf[1] = a_mode; // 0 for no AF LED or disable AF LED; 1 for enable AF LED
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

//================================
static int Domingo_SetPreviewQuality(UINT8 contrast, UINT8 sharpness, UINT8 saturation, UINT8 imagesmooth)
{
    UINT8 cmdbuf[2];

    //let the LCD looks better -
    cmdbuf[0] = 0x18; //OPCODE_SETCONTRAST
    cmdbuf[1] = contrast;
    DomingoSetAndWait(cmdbuf, 2, 50);

    cmdbuf[0] = 0x1A; //OPCODE_SETSHARPNESS
    cmdbuf[1] = sharpness;
    DomingoSetAndWait(cmdbuf, 2, 10);

    cmdbuf[0] = 0x20; //OPCODE_SETSATURATION
    cmdbuf[1] = saturation;
    DomingoSetAndWait(cmdbuf, 2, 10);

    cmdbuf[0] = 0x22; //OPCODE_SETIMAGESMOOTH
    cmdbuf[1] = imagesmooth;
    DomingoSetAndWait(cmdbuf, 2, 10);

    return 0;
}

static int Domingo_SetOutputFormat(UINT8 preview_mode, UINT8 capture_mode)
{
    UINT8 cmdbuf[3];

    cmdbuf[0] = 0x42; // Set Output Data Format command
    cmdbuf[1] = preview_mode;
    cmdbuf[2] = capture_mode;
    return DomingoSetAndWait(cmdbuf, 3, 20);
}

//========================
static int Domingo_GetSmileStatus(UINT8 *a_pu8Status)
{
    int retCode;
    UINT8 cmdbuf[2];
    UINT8 outbuf[57];

    *a_pu8Status = 0; //default to flase
    memset(outbuf, 0, 57);
    //get face-detection status
    cmdbuf[0] = 0x4B;
    retCode = DomingoIOSetGet(cmdbuf, 1, outbuf, 57, 1);

    if(retCode == 0) //IO succeed
	{
	  int FaceCount;
	  int checkbyte;
	  int i;

	  FaceCount = outbuf[0];
	  checkbyte = 1 + FaceCount * 8;
	  for(i = 1; i < checkbyte; i+=8)
	  {
		if((outbuf[i] & 0x80) != 0 ) //found smile face
		{
		  *a_pu8Status = 1;
		  break;
		}
	  }
	}
    //TRACE(("Domingo_GetSmilestatus: %d", *a_pu8Status));

    return retCode;
}

static int Domingo_SetAutoExposureWb(UINT8 a_enable)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x2A; // OP-Code
    cmdbuf[1] = a_enable;
    return DomingoSetAndWait(cmdbuf, 2, 5);
}

static int Domingo_SetCaptureSharpness(UINT8 sharpness)
{
    UINT8 cmdbuf[2];

    cmdbuf[0] = 0x02; //Set Sharpness of Capture    
    cmdbuf[1] = sharpness;
    altek5m_info("%d", sharpness);
    DomingoSetAndWait(cmdbuf, 2, 50);
    return 0;
}

static int Domingo_EnableLiveView(int onoff)
{
    UINT8 cmdbuf[9];
    int ret;

    msleep(100);

    cmdbuf[0] = 0x7c; // OP-Code
    cmdbuf[1] = 40;
    cmdbuf[2] = 0;
    cmdbuf[3] = 0;
    cmdbuf[4] = 0;
    //cmdbuf[5] = 0;
    cmdbuf[6] = 0;
    cmdbuf[7] = 0;
    cmdbuf[8] = 0;

    if (onoff)    {
        cmdbuf[5] = 1;
    } else {
        cmdbuf[5] = 0;
    }

    ret = DomingoSetAndWait(cmdbuf, 9, 5);
    msleep(300);	
    return ret;
}

#define MAX_FW_SIZE                 (512*1024)
#define SEND_BUFFER_SIZE            (510)
#define OPCODE_SENDBULKDATA         (0x7E)
#define HOSTIF_SENDBULK_UPDATEFW     0
#define HEADER_LENGTH               32
static int Domingo_Update_Firmware(u8 *pFileData, int dataLength)
{
    int dataLeft,writeByte, ret;
    int nTimeOutCount=0;
    u8 dataBuf[SEND_BUFFER_SIZE];
    int writeRet,readRet;
    u8 outBuf=0;

    if (dataLength > MAX_FW_SIZE) {
        altek5m_err("dataLength err %d %d\n", dataLength, MAX_FW_SIZE);
        return -1;
    }

    altek5m_info("Start download camera main code %d", dataLength);
    memset(dataBuf, 0, sizeof(dataBuf));
    dataLeft = dataLength;
    dataBuf[0] = OPCODE_SENDBULKDATA;
    dataBuf[1] = HOSTIF_SENDBULK_UPDATEFW;

    //sending firmware to module
    while(dataLeft > 0)
    {
        writeByte = dataLeft>SEND_BUFFER_SIZE-2? SEND_BUFFER_SIZE-2:dataLeft;
        memcpy(&(dataBuf[2]), pFileData+(dataLength-dataLeft),writeByte);

#ifdef CONFIG_CI_CAMERA_GPIO_I2C
	ret = i2c_gpio_send(g_client->addr, dataBuf, writeByte+2);
#else        
	 ret = i2c_master_send(g_client, dataBuf, writeByte+2);
#endif
        if (ret < 0) {			
		altek5m_err("dataLeft %d, writeByte %d, ret %d\n", dataLeft, writeByte, ret);
		return -2;
        }

        msleep(10);

#ifdef CONFIG_CI_CAMERA_GPIO_I2C
	ret = i2c_gpio_recv(g_client->addr, &outBuf, 1);
#else
	ret = i2c_master_recv(g_client, &outBuf, 1);
#endif
	if (ret<0 || (0x01!=outBuf)) {
		altek5m_err("dataLeft %d, writeByte %d, ret %d, outBuf %d\n", dataLeft, writeByte, ret, outBuf);
		return -3;
	}

        dataLeft -= writeByte;
        msleep(10);
    }

    msleep(100);

    altek5m_info("Burnning camera main code");
#ifdef CONFIG_CI_CAMERA_GPIO_I2C
    ret = i2c_gpio_send(g_client->addr, dataBuf, 2);
#else        
    ret = i2c_master_send(g_client, dataBuf, 2);
#endif
    if (ret < 0) {			
	altek5m_err("OPCODE_SENDBULKDATA fail %d\n", ret);
	return -4;
    }

    outBuf=0;
    do{
        msleep(100);
#ifdef CONFIG_CI_CAMERA_GPIO_I2C
        ret = i2c_gpio_recv(g_client->addr, &outBuf, 1);
#else
        ret = i2c_master_recv(g_client, &outBuf, 1);
#endif
	if (ret<0) {
		altek5m_err("get OPCODE_SENDBULKDATA fail %d\n", ret);
		return -5;
	}
    }while(outBuf!=0x01 && nTimeOutCount++<200);

    if(nTimeOutCount<201) {
        altek5m_info("Caamera main code is updated!!");
	 ret = 0;
    } else {
        altek5m_err("Burn camera main code fail!!");
         ret = -6;
    }

    return ret;
}

static int getHwVersion(void)
{
	u8 cmd[1];
	char altek_version[35];
	int ret;
	altek_version[34] = 0x0;

	cmd[0] = 0x8;
	ret = DomingoIOSetGet(cmd, 1, altek_version, 34, 20);
	if (ret>=0)
		altek5m_info("%s\n", altek_version);
	return ret;
}

static int Domingo_Init(void)
{
	int ret = 0;
	UINT8 cmdbuf[4];

	g_altek5m_init_jiffles = jiffies;

#if 1
	Domingo_SetPreviewSize(0x7); /* vga */
	Domingo_SetQuickViewSize(0x2);
	g_altek5m_preSize = altek5m_size_640_480;
#else
	Domingo_SetPreviewSize(0x4); /* qvga */
	Domingo_SetQuickViewSize(0x1); 
	g_altek5m_preSize = altek5m_size_320_240;
#endif
	g_altek5m_video_mode = 1;

	Domingo_SetAFMode(3,2);
	// enable the focus bracket
	//Domingo_ShowFocusFrame(1);

#if defined(CONFIG_BOARD_X2)
{
	u8 flip;
	cmdbuf[0] = 0x3D; // OPCODE_GETIMAGEFLIP
	DomingoIOSetGet(cmdbuf, 1, &flip, 1, 20);
	altek5m_info("flip %d\n", flip);

	if (flip!=3) {
		cmdbuf[0] = 0x3C; //OPCODE_SETIMAGEFLIP
		cmdbuf[1] = 3;    //both
		ret = DomingoSetAndWait(cmdbuf, 2, 100);
	}
}
#endif

	Domingo_SetFlashType(1);  // 1 if no flash set 0, 1 led, 2 xron
	Domingo_SetPreFlash(1); //led flash use it
	Domingo_SetRedeyeRemoval(0); //led flash use it
	Domingo_SetAFLEDMode(1);
	Domingo_SetFlashMode(2); // auto

	//Domingo_SetMakerModelName(0, "Sony");	
	//Domingo_SetBlurDetect(1);
	
{
	struct rtc_device *rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc)  {
		struct rtc_time tm;
		rtc_read_time(rtc, &tm);
		Domingo_SetTime(tm.tm_year, tm.tm_mon+1, tm.tm_mday, 
			tm.tm_hour, tm.tm_min, tm.tm_sec);
		rtc_class_close(rtc);
	}
}

	msleep(10);
	Domingo_SetMakerModelName(0, "Sony");

	//Domingo_SetCaptureSharpness(8);

	//cmdbuf[0] = 0x56; cmdbuf[1] = 0x1; /* histogram flag on */
	//DomingoSetAndWait(cmdbuf, 2, 10);	
	return ret;
}

static int Domingo_AfStart;
static int Domingo_ReleaseAfHardware(void)
{
	UINT8 cmdbuf[2];
	if (Domingo_AfStart) {
		//release auto-focus hardware,
		cmdbuf[0] = 0x40; //OPCODE_AFCONTROL;
		cmdbuf[1] = 1;    //HOSTIF_AF_CTRL_RELEASE;
		DomingoSetAndWait(cmdbuf, 2, 50);
		Domingo_AfStart = 0;
	}
	
	return 0;
}

//========================
static int Domingo_EnterStillPreview(void)
{
    u32  t1, t2;
    UINT8 cmdbuf[5];
    UINT8 outbuf[2];
    int ret, retry = 0;
	
    //return to preview mode
    cmdbuf[0] = 0x0C; //OPCODE_SETOPMODE;
    cmdbuf[1] = 1;    // HOSTIF_OPMODE_PREVIEW;
    ret = DomingoIOSetGet(cmdbuf, 2, NULL, 0, 1);
    msleep(10);

    //check if set mode is stable
    do {		
        cmdbuf[0] = 0x55;
        DomingoIOSetGet(cmdbuf, 1, outbuf, 1, 10); //get mode status
        //altek5m_info("%d\n", outbuf[0]);
        if (outbuf[0] != 1)
             msleep(10);
    } while((outbuf[0]!=1) && (retry++<50));   //repeat check mode status until it is finished
    altek5m_info("status %d, retry %d\n", outbuf[0], retry);   

    g_altek5m_init_jiffles = jiffies;

    return ret;
}

//========================
#define VIDEOTRYTIME    20
static int Domingo_EnterVideoPreview(void)
{
    UINT8 cmdbuf[5];
    UINT8 capret[2];
    int ret;

    altek5m_info("\n");
    msleep(250);

    cmdbuf[0] = 0x36; // OPCODE_CAPTURECONTROL;
    cmdbuf[1] = 1;    //HOSTIF_CAPTURE_VIDEO;
    ret = DomingoIOSetGet(cmdbuf, 2, NULL, 0, 20);

    if(ret == 0)
    {
      int cnt = 0;  //work around
      //get capture status
      cmdbuf[0] = 0x37; //OPCODE_GETCAPTURESTATUS;
      capret[0] = 1;
      while(capret[0] != 0 && cnt < VIDEOTRYTIME)
      {
          DomingoIOSetGet(cmdbuf, 1, capret, 2, 20);
          cnt++;
          //TRACE(("status= %d", capret[0]));
      }
      if(cnt == VIDEOTRYTIME) ret = 77;
    }
	
    return ret;
}

//========================
static int Domingo_PowerDown(void)
{
    int ret = 0, retry= 0;
    UINT8 cmdbuf[5];
    UINT8 outbuf[5];

    Domingo_ReleaseAfHardware();

    // Enter power-down mode
    cmdbuf[0] = 0x0C; //OPCODE_SETOPMODE;
    cmdbuf[1] = 2;
    DomingoIOSetGet(cmdbuf, 2, NULL, 0, 1);
    msleep(10);

    //check if set mode is stable
    do {
        cmdbuf[0] = 0x55;
        outbuf[0] = 0;
        DomingoIOSetGet(cmdbuf, 1, outbuf, 1, 10); //get mode status
        //altek5m_info("%d\n", outbuf[0]);
        if (outbuf[0] != 1)
             msleep(10);
    } while( (outbuf[0] != 1) && (retry++<5));   //repeat check mode status until it is finished;
    altek5m_info("status %d, retry %d\n", outbuf[0], retry);

    msleep(50*2); /* wait power down */
	
    return ret;
}

//========================
static int Domingo_DoS1(void)
{
    UINT8 cmdbuf[5];
    UINT8 af_status;
    int retry = 0, ret = -EAGAIN;	

    //start AF process (and AE/AWB)
    cmdbuf[0] = 0x40; // Select AF control
    cmdbuf[1] = 0;    // Start AF
    DomingoSetAndWait(cmdbuf, 2, 20);
    Domingo_AfStart = 1;

    //get AF status
    cmdbuf[0] = 0x3b; // Get AF status
    af_status = 0x15; // in process flags
    while(af_status == 0x15 && retry<250)  { //repeat checking AF status until it is finished 
        msleep(5);
        DomingoIOSetGet(cmdbuf, 1, &af_status, 1, 50);
        //altek5m_dbg("af_status 0x%x\n", af_status);
        retry++;
    }

    altek5m_info("af_status 0x%x, retry %d\n", af_status, retry);
    if (retry<250) {
	 if (0==af_status)
		ret = 0;
	 else
		ret = -EIO;	 
    } else /* wait */ {
        Domingo_ReleaseAfHardware();
    }

    Domingo_SetAutoExposureWb(1);	
    Domingo_ShowFocusFrame(2);
    msleep(200); /* wait auto ae stable */
    return ret;
}

//==============================
static void Domingo_DoS2(void)
{
    UINT8 cmdbuf[5];
    altek5m_info("\n");

    //send take picture command
    cmdbuf[0] = 0x36; // Capture Control command
    cmdbuf[1] = 0;  // Still image capture

    //send Capture Control command
    DomingoIOSetGet(cmdbuf, 2, NULL, 0, 1);
}

//========================
static int Domingo_WaitS2(void)
{
    UINT8 cmdbuf[5];
    UINT8 af_status = -1;
    int retry = 0;
	
    //get capture status
    af_status = 1;
    udelay(100);
    while(af_status != 0 && (retry++<20))  //repeat checking capture status until it is finished
    {        
        cmdbuf[0] = 0x37; // Get Capture Control Status
        if (DomingoIOSetGet(cmdbuf, 1, &af_status, 1, 20)<0)
        {
            altek5m_err("status read error\n");
            af_status = 1;
        }
        if (af_status != 0)
             msleep(10);
        //altek5m_info("s2_status: %d\n", af_status);        
    }
    altek5m_info("s2_status 0x%x, retry %d\n", af_status, retry);
    return (af_status!=0)?-EIO:0;
}

static void altek5m_gpio_init(void)
{
	pxa3xx_mfp_set_afds(CIF_nRESET, CIF_nRESET_ALT, MFP_DS04X);
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(CIF_nRESET, MFP_LPM_DRIVE_LOW); /* low in sleep */

	pxa3xx_mfp_set_afds(CIF_nPOWERDN, CIF_nPOWERDN_ALT, MFP_DS04X); 
	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(CIF_nPOWERDN, MFP_LPM_DRIVE_LOW); /* low in sleep */

#if defined(CONFIG_BOARD_LANDMARK) /* fix me */
	pxa3xx_mfp_set_afds(CIF_nCOREEN, MFP_AF0, MFP_DS04X);
	gpio_direction_output(CIF_nCOREEN, GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(CIF_nCOREEN, MFP_LPM_DRIVE_LOW);
#else
	pxa3xx_mfp_set_afds(CIF_nCOREEN, CIF_nCOREEN_ALT, MFP_DS04X); 
	gpio_direction_output(CIF_nCOREEN, GPIO_LEVEL_HIGH);

	pxa3xx_mfp_set_afds(CIF_PWREN2V8, CIF_PWREN2V8_AF, MFP_DS04X);
	gpio_direction_output(CIF_PWREN2V8, GPIO_LEVEL_LOW);

	pxa3xx_mfp_set_afds(CIF_nPOWER1V8, CIF_nPOWER1V8_ALT, MFP_DS04X);
	gpio_direction_output(CIF_nPOWER1V8, GPIO_LEVEL_LOW);

	pxa3xx_mfp_set_afds(CIF_VEXIOVDD, CIF_VEXIOVDD_AF, MFP_DS04X);
	gpio_direction_output(CIF_VEXIOVDD, GPIO_LEVEL_LOW);
#endif
}

static void altek5mhw_power_up(void)
{	
	UINT8 cmdbuf[1];
	UINT8 outbuf[1];
	int retry = 0, ret;
	cmdbuf[0] = 0x0D; //OPCODE_GETOPMODE;
	g_altek5m_i2c_err_cnt = 0;
	altek5m_info("start\n"); 	

#ifdef CONFIG_CI_CAMERA_GPIO_I2C
	i2c_gpio_init();
#endif

#if defined(CONFIG_BOARD_LANDMARK)
	pxa3xx_enable_qci_pins(1);

	gpio_direction_output(CIF_nCOREEN, GPIO_LEVEL_HIGH);
	udelay(100);
	/* DVDD:1.8V, IOVDD:2.8V,  AVDD:2.8V */
	micco_enable_LDO13(1);
	micco_enable_LDO9(1);
	micco_enable_LDO7(1);
#else
	gpio_direction_output(CIF_nCOREEN, GPIO_LEVEL_HIGH); /* 1.2V */
	micco_enable_LDO13(1);
	udelay(100);
	
	gpio_direction_output(CIF_nPOWER1V8, GPIO_LEVEL_HIGH); /* 1.8V */
	
	gpio_direction_output(CIF_PWREN2V8, GPIO_LEVEL_HIGH); /* 2.8V */

	gpio_direction_output(CIF_VEXIOVDD, GPIO_LEVEL_HIGH);  /* Motor */
#endif

	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_HIGH);	
	ci_set_clock(1, 1, CIF_MCLK_KHZ);
	udelay(100); /* reset */
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_HIGH);		
	msleep(TbootupMs+100); /* Tbootup */

	do { //check if set mode is preview mode
		ret = DomingoIOSetGet(cmdbuf, 1, outbuf, 1, 1);
		if (outbuf[0] != 1)
			msleep(500);
	} while((outbuf[0]!=1) && (retry++<2));   //repeat check mode status until it is finished	
	altek5m_info("status %d, retry %d\n", outbuf[0], retry); 
}

static void altek5mhw_power_down(void)
{ /* keep power @ sleep */
	g_workType = altek5m_work_none;
	
	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_LOW);
	
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_LOW);
	
	ci_set_clock(0, 0, CIF_MCLK_KHZ);

#if defined(CONFIG_BOARD_LANDMARK) /* fix me */
	/* IOVDD:2.8V, DVDD:1.8V, AVDD:2.8V */
	micco_enable_LDO13(0);	
	micco_enable_LDO7(0);
	micco_enable_LDO9(0); 
	gpio_direction_output(CIF_nCOREEN, GPIO_LEVEL_LOW);

	pxa3xx_enable_qci_pins(0); // current leak
#else
	gpio_direction_output(CIF_nPOWER1V8, GPIO_LEVEL_LOW);
	
	gpio_direction_output(CIF_PWREN2V8, GPIO_LEVEL_LOW);

	gpio_direction_output(CIF_VEXIOVDD, GPIO_LEVEL_LOW);

	#if 0
	gpio_direction_output(CIF_nCOREEN, GPIO_LEVEL_LOW);
	micco_enable_LDO13(0);
	#else
	pxa3xx_mfp_set_lpm(CIF_nCOREEN, MFP_LPM_DRIVE_HIGH); /* high in sleep */
	#endif
#endif

#ifdef CONFIG_CI_CAMERA_GPIO_I2C
	i2c_gpio_deinit();
#endif	
	altek5m_info("succ \n");
}

void altek5mhw_power_mode(u8 power_mode)
{
	if (power_mode == CAMERA_POWER_OFF ) {
		g_altek5m_status = altek5m_status_powerdown;
		//g_altek5m_i2c_err_cnt = 0;
		Domingo_PowerDown();
		altek5mhw_power_down();
	} else {
		g_altek5m_status = altek5m_status_powerup;
		altek5mhw_power_up();
	}
}

static int altek5mhw_set_mode(u32 mode,u32 value)
{	
	int ret = -EINVAL;
	altek5m_info("0x%x 0x%x\n", mode, value);
	
	switch(mode){
	case V4L2_CID_DO_WHITE_BALANCE:
		if (value<5)
			ret = Domingo_SetWB(value);
		break;

	case V4L2_CID_ZOOM:
		if (value<9)
			ret = Domingo_SetDigitalZoom(value);	
		break;

	case V4L2_CID_FILTER:
		if (value==2) //negative
			value = 5;
		if (value==3) //sepia
			value = 7;

		if (value<8)
			ret = Domingo_SetEffect(value);
		break;

	case V4L2_CID_SCENE:
		if (value<8)
			ret = Domingo_SetScene(value);
		break;

	case V4L2_CID_FLASHLIGHT:
		if (value<3)
			ret = Domingo_SetFlashMode(value);
		break;

	case V4L2_CID_EXPOSURE_COMP:
		if (value<13)
			ret = Domingo_SetEvComp(value);
		break;

	case V4L2_CID_ISO:
		if (value<0xff)
			ret = Domingo_SetISO(value);
		break;

	case V4L2_CID_ANTIFLICKER:
		if (value<4)
			ret = Domingo_SetBanding(value);
		break;

	case V4L2_CID_TIMESTAMP:
		if (value<5)
			ret = Domingo_SetTimeStamp(value);
		break;

	case V4L2_CID_FOCUSMODE:
		if (value<5)
			ret = Domingo_SetAFMode(3, value);
		break;

	case V4L2_CID_AUTOFOCUS:
		if (time_before(jiffies, g_altek5m_init_jiffles+msecs_to_jiffies(2000))) {			
			altek5m_info("V4L2_CID_AUTOFOCUS too early\n");
			msleep(g_altek5m_init_jiffles+msecs_to_jiffies(2000)-jiffies);
		}
		
		ret = Domingo_DoS1();
		break;

	case V4L2_CID_GETQUCIKVIEW: {
		int w, h;
		u8 size = 12;//5m
		w = SI_SIZE_GTE_W(value);
		h = SI_SIZE_GTE_H(value);

		if (640==w && 480==h)
			size = 4; //vga
		else if (2560==w && 1920==h)
			size = 12; //5m
		else if (2048==w && 1536==h)
			size = 11; // 3m
		else if (1600==w && 1200==h)
			size = 9; //2m
		else if (1280==w && 960==h)
			size = 7; //1.3m
		else if (800==w && 600==h)
			size = 5; //svga
		altek5m_info("V4L2_CID_GETQUCIKVIEW %d * %d, %d\n", w, h, size);
		Domingo_SetCaptureSize(size);
		
		Domingo_DoS2();
		ret = Domingo_WaitS2();
		break;
	}

	case V4L2_CID_SHOWFOCUS:
		if (value) {
			g_altek5m_video_mode = 0;
			Domingo_ShowFocusFrame(1);
		} else {
			Domingo_ShowFocusFrame(0);
			Domingo_SetFrameRate(15); /* 15fps */		
			g_altek5m_video_mode = 1;
		}
		ret = 0;
		break;

	default:
		ret =  -EINVAL;
	}

	if (ret<0)
		altek5m_err("mode 0x%x, value 0x%x, ret %d\n", mode, value, ret);
	return ret;
}

static int altek5mhw_get_mode(u32 mode,u32* value)
{
	int ret = -EINVAL;	
	
	switch(mode){
	case V4L2_CID_GETBLURSTATUS: {
		u8 blurStatus = 0;
		Domingo_GetBlurStatus(&blurStatus);
		*value = blurStatus;
		ret = 0;
		break;
	}
	default:
		ret =  -EINVAL;
	}

	//altek5m_info("0x%x 0x%x\n", mode, *value);
	if (ret<0)
		altek5m_err("mode 0x%x, value 0x%x, ret %d\n", mode, value, ret);
	return ret;
}

static int altek5mhw_set_format(altek5m_size size, int mode)
{
	int ret = -EIO;
	
	if (CAMERA_MODE_VIDEO==mode) {
		if (altek5m_status_capture==g_altek5m_status) {
			msleep(TbootupMs); /* sleep to wait stable*/
			Domingo_EnterStillPreview();
		}
		if (g_altek5m_preSize!=size) {
			if (altek5m_size_640_480==size) { 
				ret = Domingo_SetPreviewSize(0x7); /* vga */
				ret = Domingo_SetQuickViewSize(0x2); 
			} else if (altek5m_size_320_240==size)  { 
				ret = Domingo_SetPreviewSize(0x4); /* qvga */
				ret = Domingo_SetQuickViewSize(0x1); 
			} else if (altek5m_size_352_288==size) { 
				ret = Domingo_SetPreviewSize(0x5);  
			} else if (altek5m_size_176_144==size) { 
				ret = Domingo_SetPreviewSize(0x2); 
			}
			g_altek5m_preSize = size;
		} else
			ret = 0;
			
		if (altek5m_status_capture==g_altek5m_status) {
			Domingo_ReleaseAfHardware();			 
			Domingo_SetAutoExposureWb(1);
		} else {
			Domingo_EnterStillPreview();
		}
		if ((altek5m_size_640_480==size || altek5m_size_320_240==size) 
				&& !g_altek5m_video_mode) {
			Domingo_ShowFocusFrame(1);
		}
		g_altek5m_status = altek5m_status_preview;
	} else {
		ret = 0;
		g_altek5m_status = altek5m_status_capture;
	}

	return ret;
}

static void altek5mhw_init(void)
{
	Domingo_Init();
	//g_workType = altek5m_work_doAF;
	//schedule_work(&altek5m_work); 
}

#define LCDBL_LEVEL_FOR_ALTEK (66)
extern int lcdbl_set_level(int level);
extern int lcdbl_get_level(void);
static int g_lcdbl_old_level = 50;
static int altek5m_lcdbl_workaround(int enable)
{
	int new_lcdbl_level;

	pxafb_adjust_pclk_workaround(enable);
	
	if (enable) {
		g_lcdbl_old_level = lcdbl_get_level();
		new_lcdbl_level = g_lcdbl_old_level<LCDBL_LEVEL_FOR_ALTEK?
			LCDBL_LEVEL_FOR_ALTEK:g_lcdbl_old_level;
	} else
		new_lcdbl_level = g_lcdbl_old_level;
	lcdbl_set_level(new_lcdbl_level);
	
	return 0;
}

int altek5m_init(p_camera_context_t camera_context)
{	
	/* Configure master parallel with 8 data pins */
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8);

	/* data sample on falling and h,vsync active high */
	ci_set_polarity(0, 0, 0);

	/* fifo control */
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_0_TO_7);

	/* Sensor Power on */
	altek5mhw_power_mode(CAMERA_POWER_FULL);

	/* Set initial code */
	altek5mhw_init();

#ifdef CONFIG_BOARD_X2
	altek5m_lcdbl_workaround(1);
#endif

	return 0;
}

int altek5m_deinit(p_camera_context_t camera_context)
{
#ifdef CONFIG_BOARD_X2
	altek5m_lcdbl_workaround(0);
#endif
	altek5mhw_power_mode(CAMERA_POWER_OFF);
	return 0;
}

int altek5m_sleep(p_camera_context_t camera_context)
{
	return altek5m_deinit(camera_context);
}

int altek5m_wake(p_camera_context_t camera_context)
{
	return altek5m_init(camera_context);
}

int altek5m_set_capture_format(p_camera_context_t camera_context )
{
	CI_MP_TIMING timing;
	altek5m_size size;
	
	altek5m_dbg("in %d out %d w %d h %d\n", camera_context->capture_input_format, 
		camera_context->capture_output_format, camera_context->capture_input_width,
		camera_context->capture_input_height);
	if (CAMERA_MODE_VIDEO==camera_context->capture_mode) {
		if (V4L2_PIX_FMT_YUV422P!=camera_context->capture_input_format) {
			altek5m_err("%d.\n", camera_context->capture_input_format);
			return -EINVAL;	
		}		
	} else {
		if (V4L2_PIX_FMT_JPEG!=camera_context->capture_input_format) {
			altek5m_err("%d.\n", camera_context->capture_input_format);
			return -EINVAL;	
		}
	}

	if (SENSOR_CHECK_WH(camera_context, 640, 480))
		size = altek5m_size_640_480;
	else if (SENSOR_CHECK_WH(camera_context, 2560, 1920))
		size = altek5m_size_2560_1920;
	else if (SENSOR_CHECK_WH(camera_context, 2048, 1536))
		size = altek5m_size_2048_1536;
	else if (SENSOR_CHECK_WH(camera_context, 1600, 1200))
		size = altek5m_size_1600_1200;
	else if (SENSOR_CHECK_WH(camera_context, 1280, 960))
		size = altek5m_size_1280_960;
	else if (SENSOR_CHECK_WH(camera_context, 800, 600))
		size = altek5m_size_800_600;
	else if (SENSOR_CHECK_WH(camera_context, 320, 240))
		size = altek5m_size_320_240;
	else if (SENSOR_CHECK_WH(camera_context, 352, 288))
		size = altek5m_size_352_288;
	else  if (SENSOR_CHECK_WH(camera_context, 176, 144))
		size = altek5m_size_176_144;
	else	{
		altek5m_err("size %d %d not support\n", 
			camera_context->capture_input_width, camera_context->capture_input_height);
		return -EINVAL;
	}

	altek5mhw_set_format(size, camera_context->capture_mode);

	/* set capture width/height and timing */
	timing.BFW = 0x0;
	timing.BLW = 0x0;
	ci_configure_mp(camera_context->capture_input_width-1,
			camera_context->capture_input_height-1, &timing);
#if defined(CONFIG_CPU_PXA310)
	ci_set_ycbcr_420_down_sample (camera_context->ycbcr_ds);
#endif

	return 0;
}

int altek5m_start_capture(p_camera_context_t camera_context,
		unsigned int frames)
{
	return 0;
}

int altek5m_stop_capture(  p_camera_context_t camera_context )
{
	return 0;
}

int altek5m_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	altek5mhw_power_mode(mode);
	return 0;
}

int altek5m_set_mode( p_camera_context_t camera_context,
		u32 mode, u32 value)
{
	return altek5mhw_set_mode(mode,value);
}

int altek5m_get_mode( p_camera_context_t camera_context, u32 mode, u32* value)
{
	return altek5mhw_get_mode(mode,value);
}


int altek5m_read_reg( p_camera_context_t camera_context,u32 reg_addr, u32* reg_val)
{
	return -EIO;
}

int altek5m_write_reg( p_camera_context_t camera_context,u32 reg_addr, u32 reg_val)
{
	return -EIO;
}

static int altek5m_format_list[] = {
	V4L2_PIX_FMT_YUV422P,
	V4L2_PIX_FMT_JPEG,
	-1	
};
static p_camera_function_t altek5m_functions;
static int altek5m_register(void)
{
	altek5m_functions = kzalloc(sizeof(camera_function_t), GFP_KERNEL);
	if (!altek5m_functions) {
		altek5m_err("fail\n");
		return -ENOMEM;
	}

	altek5m_functions->format_list = altek5m_format_list;	
	altek5m_functions->init = altek5m_init;     	
	altek5m_functions->deinit = altek5m_deinit;     
	altek5m_functions->set_capture_format = altek5m_set_capture_format; 	
	altek5m_functions->start_capture =	altek5m_start_capture;	 		
	altek5m_functions->stop_capture =	altek5m_stop_capture; 			
	altek5m_functions->sleep = altek5m_sleep;	
	altek5m_functions->wakeup = altek5m_wake;	
	altek5m_functions->read_reg = altek5m_read_reg;			
	altek5m_functions->write_reg = altek5m_write_reg;
	altek5m_functions->set_power_mode = altek5m_set_power_mode;			
	altek5m_functions->set_mode =	altek5m_set_mode;
	altek5m_functions->get_mode =	altek5m_get_mode;	
	altek5m_functions->name = "altek5m";
	altek5m_functions->id = 0;

	if (sensor_register(altek5m_functions, altek5m_ID) < 0) {
		altek5m_err("sensor_register failed !\n");
		kfree(altek5m_functions);
		altek5m_functions = NULL;
		return -EFAULT;
	}
	
	return 0;
}

static int altek5m_unregister(void)
{
	sensor_unregister(altek5m_ID);
	
	if (altek5m_functions) {
		kfree(altek5m_functions);
		altek5m_functions = NULL;
	}
	return 0;
}

static ssize_t af_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	int result = Domingo_DoS1();
	return sprintf(buf, "%d\n", result);
}
static DEVICE_ATTR(af, 0644,af_show,NULL);
static ssize_t power_up_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct clk *clk;
	clk = clk_get(NULL, "CAMCLK");
	if (!clk) {
		altek5m_err("failed to get camera clock\n");
		return -EIO;
	}
	
	clk_enable(clk);
	altek5mhw_power_up();

	clk_put(clk);
	return count;
}
static DEVICE_ATTR(power_up, 0644,NULL,power_up_store);
static ssize_t power_down_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct clk *clk;
	clk = clk_get(NULL, "CAMCLK");
	if (!clk) {
		altek5m_err("failed to get camera clock\n");
		return -EIO;
	}

	altek5mhw_power_down();
	clk_disable(clk);
	
	clk_put(clk);
	return count;
}
static DEVICE_ATTR(power_down, 0644,NULL,power_down_store);

static ssize_t quality_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int contrast, sharpness, saturation, imagesmooth;
	
	sscanf(buf, "%d %d %d %d", &contrast, &sharpness, &saturation, &imagesmooth);
	Domingo_SetPreviewQuality(contrast, sharpness, saturation, imagesmooth);
	
	return count;
}
static DEVICE_ATTR(quality, 0644,NULL,quality_store);

static char* g_firmware_data;
static int g_firmware_len;
static ssize_t altek5m_firmware_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	
	printk("mcu_bin_store %d %d\n", count, g_firmware_len);	
	if (count>=4095) {
		if (g_firmware_data) {
			memcpy(g_firmware_data+g_firmware_len, buf, count);
			g_firmware_len += count;
		} else {
			g_firmware_data = kmalloc(MAX_FW_SIZE, GFP_KERNEL);
			g_firmware_len = 0;

			memcpy(g_firmware_data+g_firmware_len, buf, count);
			g_firmware_len += count;
		}
		ret = count;
	} else {
		struct clk *clk;
		
		memcpy(g_firmware_data+g_firmware_len, buf, count);
		g_firmware_len += count;
		
		clk = clk_get(NULL, "CAMCLK");
		clk_enable(clk);		
		altek5mhw_power_up();

		Domingo_EnableLiveView(0);
		ret = Domingo_Update_Firmware(g_firmware_data, g_firmware_len);
		Domingo_EnableLiveView(1);
		
		kfree(g_firmware_data);
		g_firmware_data = NULL;
		g_firmware_len = 0;
		
		altek5mhw_power_down();		
		clk_disable(clk);
		clk_put(clk);
	}
	
	return ret;
}
static DEVICE_ATTR(firmware, 0644,NULL,altek5m_firmware_store);

static ssize_t capture_sharpness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int sharpness;
	
	sscanf(buf, "%d", &sharpness);
	Domingo_SetCaptureSharpness(sharpness);	
	return count;
}
static DEVICE_ATTR(capture_sharpness, 0644,NULL,capture_sharpness_store);

static struct attribute *af_attributes[] = {	
	&dev_attr_af.attr,
	&dev_attr_power_up.attr,
	&dev_attr_power_down.attr,
	&dev_attr_quality.attr,
	&dev_attr_firmware.attr,
	&dev_attr_capture_sharpness.attr,
	NULL,
};
static struct attribute_group altek5m_attr_group ={
	.attrs = af_attributes,
};

static int altek5m_detect(void)
{	
	int ret;
	struct clk *clk;
	altek5m_dbg("\n");
	
#ifdef CONFIG_YUHUA_MISC_DEV
	if (is_camera_detect())
		return -EIO;
#endif

	clk = clk_get(NULL, "CAMCLK");
	if (!clk) {
		altek5m_err("failed to get camera clock\n");
		return -EIO;
	}
	clk_enable(clk);

	/* detect altek5m */
	altek5mhw_power_up();	
	
	ret = getHwVersion();
	altek5m_dbg("ret %d\n", ret);
	if (ret>=0) {
		/* Todo: i2c power-off cmd */
		Domingo_PowerDown();	
		msleep(50*2);
#ifdef CONFIG_YUHUA_MISC_DEV
		set_camera_id("altek_5m");
		set_camera_detect(1);
#endif
	}
	altek5mhw_power_down();

	clk_disable(clk);
	clk_put(clk);

	if (ret>=0) {
		ret = altek5m_register();		
	}

	sysfs_create_group(&g_client->dev.kobj, &altek5m_attr_group);
	
	printk("Altek 5m camera register %s\n", ret>=0?"succ":"fail");
	return ret;
}

static void altek5m_work_func(struct work_struct* work)
{
	if (altek5m_work_detect==g_workType)
		altek5m_detect();
	else if (altek5m_work_doAF==g_workType) {
		Domingo_DoS1();
	} else if (altek5m_work_releaseAF==g_workType) {
		Domingo_ReleaseAfHardware();
	} else if (altek5m_work_doAE==g_workType) {
		if (altek5m_status_preview==g_altek5m_status) {
			altek5m_info("altek5m_work_doAE\n");
			Domingo_SetAutoExposureWb(1);
		}
	}
	g_workType = altek5m_work_none;
}

static int __devinit altek5m_probe(struct i2c_client *client)
{
	g_client = client;
#ifdef CONFIG_YUHUA_MISC_DEV
	if (is_camera_detect())
		return -EIO;
#endif

	altek5m_gpio_init();
	
	INIT_DELAYED_WORK(&altek5m_work, altek5m_work_func);
	g_workType = altek5m_work_detect;
	schedule_delayed_work(&altek5m_work, msecs_to_jiffies(500)); 
	return 0;
}

static int altek5m_remove(struct i2c_client *client)
{	
	altek5m_unregister();
	g_client = NULL;
	return 0;
}

static const struct i2c_device_id altek5m_id[] = {
	{ "altek_5m", 0 },
	{ }
};

static struct i2c_driver altek5m_driver = {
	.driver = {
		.name	= "altek_5m",
	},
	.id_table	= altek5m_id,
	.probe		= altek5m_probe,
	.remove		= altek5m_remove,
};

static int __init i2c_altek5m_init(void)
{
	return i2c_add_driver(&altek5m_driver);
}
static void __exit i2c_altek5m_exit(void)
{
	i2c_del_driver(&altek5m_driver);
}
MODULE_DESCRIPTION("Altek 5m camera driver");
module_init(i2c_altek5m_init);
module_exit(i2c_altek5m_exit);

