#ifndef __YUHUA_SIGN_H__
#define __YUHUA_SIGN_H__
/*
 *  Copyright (C) Yuhua Tel 2008
 */

/* Add for YuhuaTel develope board */
#if defined(CONFIG_BOARD_MN1)
#define YUHUA_BOARD_NAME "MN1"
#elif defined(CONFIG_BOARD_TMQV1)
#define YUHUA_BOARD_NAME "TMQ_V1"
#elif defined(CONFIG_BOARD_X2_V4)
#define YUHUA_BOARD_NAME "X2_V4"
#elif defined(CONFIG_BOARD_X2)
#define YUHUA_BOARD_NAME "X2"
#elif defined(CONFIG_BOARD_PHOENIX)
#define YUHUA_BOARD_NAME "PHOENIX"
#elif defined(CONFIG_BOARD_XPHONE)
#define YUHUA_BOARD_NAME "XPHONE"
#elif defined(CONFIG_BOARD_BRAVA)
#define YUHUA_BOARD_NAME "BRAVA"
#elif defined(CONFIG_BOARD_X2G)
#define YUHUA_BOARD_NAME "X2G"
#elif defined(CONFIG_BOARD_LANDMARK)
#define YUHUA_BOARD_NAME "LANDMARK"
#else
#pragma pls fix it
#define YUHUA_BOARD_NAME "UNKNOW"
#endif

#define YUHUA_KERNEL_SIGN_PRE "YUHUA_KERSIGN_"
#if defined(CONFIG_BOARD_PHOENIX) /* tricky to blob check */
#define YUHUA_KERNEL_SIGN YUHUA_KERNEL_SIGN_PRE"XPHONE"
#else
#define YUHUA_KERNEL_SIGN YUHUA_KERNEL_SIGN_PRE""YUHUA_BOARD_NAME
#endif

#endif

