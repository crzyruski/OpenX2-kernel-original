
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>

#define SIN_INTERVAL_NUM 91
#define SIN_0   0   
#define SIN_1   9   
#define SIN_2   18  
#define SIN_3   27  
#define SIN_4   36  
#define SIN_5   45  
#define SIN_6   53  
#define SIN_7   62  
#define SIN_8   71  
#define SIN_9   80  
#define SIN_10  89  
#define SIN_11  98  
#define SIN_12  106 
#define SIN_13  115 
#define SIN_14  124 
#define SIN_15  132 
#define SIN_16  141 
#define SIN_17  150 
#define SIN_18  158 
#define SIN_19  167 
#define SIN_20  175 
#define SIN_21  183 
#define SIN_22  192 
#define SIN_23  200 
#define SIN_24  208 
#define SIN_25  216 
#define SIN_26  224 
#define SIN_27  232 
#define SIN_28  240 
#define SIN_29  248 
#define SIN_30  256 
#define SIN_31  264 
#define SIN_32  271 
#define SIN_33  279 
#define SIN_34  286 
#define SIN_35  294 
#define SIN_36  301 
#define SIN_37  308 
#define SIN_38  315 
#define SIN_39  322 
#define SIN_40  329 
#define SIN_41  336 
#define SIN_42  342 
#define SIN_43  349 
#define SIN_44  356 
#define SIN_45  362 
#define SIN_46  368 
#define SIN_47  374 
#define SIN_48  380 
#define SIN_49  386 
#define SIN_50  392 
#define SIN_51  398 
#define SIN_52  403 
#define SIN_53  409 
#define SIN_54  414 
#define SIN_55  419 
#define SIN_56  424 
#define SIN_57  429 
#define SIN_58  434 
#define SIN_59  439 
#define SIN_60  443 
#define SIN_61  448 
#define SIN_62  452 
#define SIN_63  456 
#define SIN_64  460 
#define SIN_65  464 
#define SIN_66  468 
#define SIN_67  471 
#define SIN_68  475 
#define SIN_69  478 
#define SIN_70  481 
#define SIN_71  484 
#define SIN_72  487 
#define SIN_73  490 
#define SIN_74  492 
#define SIN_75  494 
#define SIN_76  497 
#define SIN_77  499 
#define SIN_78  501 
#define SIN_79  503 
#define SIN_80  504 
#define SIN_81  506 
#define SIN_82  507 
#define SIN_83  508 
#define SIN_84  509 
#define SIN_85  510 
#define SIN_86  511 
#define SIN_87  511 
#define SIN_88  512 
#define SIN_89  512 
#define SIN_90  512 


unsigned short motionEngineASin_value[SIN_INTERVAL_NUM]=
{
SIN_0 , SIN_1 , SIN_2 , SIN_3 , SIN_4 , SIN_5 , SIN_6 , SIN_7 , SIN_8 , SIN_9 , 
SIN_10, SIN_11, SIN_12, SIN_13, SIN_14, SIN_15, SIN_16, SIN_17, SIN_18, SIN_19, 
SIN_20, SIN_21, SIN_22, SIN_23, SIN_24, SIN_25, SIN_26, SIN_27, SIN_28, SIN_29, 
SIN_30, SIN_31, SIN_32, SIN_33, SIN_34, SIN_35, SIN_36, SIN_37, SIN_38, SIN_39,
SIN_40, SIN_41, SIN_42, SIN_43, SIN_44, SIN_45, SIN_46, SIN_47, SIN_48, SIN_49,
SIN_50, SIN_51, SIN_52, SIN_53, SIN_54, SIN_55, SIN_56, SIN_57, SIN_58, SIN_59,
SIN_60, SIN_61, SIN_62, SIN_63, SIN_64, SIN_65, SIN_66, SIN_67, SIN_68, SIN_69,
SIN_70, SIN_71, SIN_72, SIN_73, SIN_74, SIN_75, SIN_76, SIN_77, SIN_78, SIN_79,
SIN_80, SIN_81, SIN_82, SIN_83, SIN_84, SIN_85, SIN_86, SIN_87, SIN_88, SIN_89,
SIN_90};


/*
* FUNCTION                                                            
*	MotionEngine_GetAngle
*
* DESCRIPTION                                                           
*   	This function is to get angle
*
* CALLS  
*
* PARAMETERS
*	
*	x,0~512
* RETURNS
*	None
*/                   
void  axis_to_orientation(short x,short *xOrientation)
{        
   short i;
     
   i=(SIN_INTERVAL_NUM-1);
   while(abs(x)<motionEngineASin_value[i] && i>=0)
   {
     	 i--;
   }

   if(x>0)
   	
          *xOrientation=i;
   else
          *xOrientation=i*(-1);   	    
} 
