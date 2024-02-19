/**
 * @brief		FRA measurement header for LC898129
 *
 * @author		(C) 2019 ON Semiconductor.
 *
 * @file		OisFRA.h
 * @date		svn:$Date:: 2021-01-20 16:15:44 +0900#$
 * @version		svn:$Revision: 2 $
 * @attention
 **/
#ifndef OISFRA_H_
#define OISFRA_H_

//****************************************************
//	extern selector for API
//****************************************************
#ifdef	__OISFRA__
	#define	__OIS_FRA_HEADER__
#else
	#define	__OIS_FRA_HEADER__		extern
#endif

typedef struct STFRA_PARAM {
	struct {
		UnFltVal		SfFrqCom ;
		UnFltVal		SfAmpCom ;
		unsigned char	UcAvgCycl ;
	} StHostCom ;

	float				SfGain[ 10 ] ;
	float				SfPhase[ 10 ] ;

	struct {
		float			SfGainAvg ;
		float			SfPhaseAvg ;
	} StMesRslt ;
} StFRAParam_t ;

__OIS_FRA_HEADER__	StFRAParam_t	StFRAParam ;



#endif	// OISFRA_H_
