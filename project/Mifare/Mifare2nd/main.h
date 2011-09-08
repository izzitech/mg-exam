#ifndef __MAIN_H__
#define __MAIN_H__

#define		DEBUG_UART

#define		ReadCdSrl		0x61		//读取卡片序列号
#define		ReadCdDat		0x62		//读块数据
#define		WritCdDat		0x63		//写块数据
#define		InitCdPrs		0x64		//初始化钱包
#define		ReadCdPrs		0x65		//读取钱包值
#define		IncrCdPrs		0x66		//钱包充值
#define		DecrCdPrs		0x67		//钱包扣款
#define		HaltCdIns		0x68		//卡片休眠
#define		TranCdPrs		0x69		//备份钱包
#define		SetBound		0x6a		//波特率设置

#endif
