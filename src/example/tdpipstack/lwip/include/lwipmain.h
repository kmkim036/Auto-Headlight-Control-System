/**
  ******************************************************************************
  * @file    lwipmain.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   This file contains all the functions prototypes for the main.c
  *          file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LWIPMAIN_H
#define __LWIPMAIN_H

#ifdef __cplusplus
 extern "C" {
#endif
#if(PROCESSOR == PROCESSOR_STM32F103C8T6)
#else
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f1x7_eth_bsp.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Time_Update(void);
void Delay(uint32_t nCount);


#ifdef __cplusplus
}
#endif
#endif //PROCESSOR
#endif /* __LWIPMAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

