/**
  ******************************************************************************
  * @file    compiler.h
  * @author  MCD Application Team
  * @version V1.0
  * @date    2014/12/11
  * @brief   This file contains macros for compiler intependent code
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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

#if defined(__GNUC__)
#define __weak __attribute__ ((weak))
#define __unused __attribute__ ((unused))
#define __maybe_unused __attribute__ ((unused))
#endif

#ifndef __unused
#define __unused
#endif

#ifndef __maybe_unused
#define __maybe_unused
#endif

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
