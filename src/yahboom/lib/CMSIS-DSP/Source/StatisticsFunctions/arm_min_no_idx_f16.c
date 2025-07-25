/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_min_no_idx_f16.c
 * Description:  Minimum value of a floating-point vector without returning the index
 *
 * $Date:        16 November 2021
 * $Revision:    V1.10.0
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2021 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "dsp/statistics_functions_f16.h"

#if defined(ARM_FLOAT16_SUPPORTED)

#if defined(ARM_MATH_NEON) || defined(ARM_MATH_MVEF)
#include <limits.h>
#endif

/**
  @ingroup groupStats
 */


/**
  @addtogroup Min
  @{
 */

/**
  @brief         Minimum value of a floating-point vector.
  @param[in]     pSrc       points to the input vector
  @param[in]     blockSize  number of samples in input vector
  @param[out]    pResult    minimum value returned here
 */

#if defined(ARM_MATH_MVE_FLOAT16) && !defined(ARM_MATH_AUTOVECTORIZE)

ARM_DSP_ATTRIBUTE void arm_min_no_idx_f16(
    const float16_t *pSrc,
    uint32_t   blockSize,
    float16_t *pResult)
{
   f16x8_t     vecSrc;
   f16x8_t     curExtremValVec = vdupq_n_f16(F16_MAX);
   float16_t   minValue = F16_MAX;
   float16_t   newVal;
   uint32_t    blkCnt;

   /* Loop unrolling: Compute 4 outputs at a time */
   blkCnt = blockSize >> 3U;

   while (blkCnt > 0U)
   {

        vecSrc = vldrhq_f16(pSrc);
        /*
         * update per-lane min.
         */
        curExtremValVec = vminnmq(vecSrc, curExtremValVec);
        /*
         * Decrement the blockSize loop counter
         * Advance vector source and destination pointers
         */
        pSrc += 8;
        blkCnt --;
    }
    /*
     * Get min value across the vector
     */
    minValue = vminnmvq(minValue, curExtremValVec);

    blkCnt = blockSize & 7;

    while (blkCnt > 0U)
    {
        newVal = *pSrc++;

        /* compare for the minimum value */
        if ((_Float16)minValue > (_Float16)newVal)
        {
            /* Update the minimum value and it's index */
            minValue = newVal;
        }

        blkCnt --;
    }

    *pResult = minValue;
}

#else

ARM_DSP_ATTRIBUTE void arm_min_no_idx_f16(
    const float16_t *pSrc,
    uint32_t   blockSize,
    float16_t *pResult)
{
   float16_t   minValue = F16_MAX;
   float16_t   newVal;

   while (blockSize > 0U)
   {
       newVal = *pSrc++;
   
       /* compare for the minimum value */
       if ((_Float16)minValue > (_Float16)newVal)
       {
           /* Update the minimum value and it's index */
           minValue = newVal;
       }
   
       blockSize --;
   }
    
   *pResult = minValue;
}

#endif /* defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE) */

/**
  @} end of Min group
 */

#endif /* #if defined(ARM_FLOAT16_SUPPORTED) */ 

