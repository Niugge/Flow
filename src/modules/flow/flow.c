/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "no_warnings.h"
#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "dcmi.h"
#include "debug.h"

#define __INLINE inline
#define __ASM asm
#include "core_cm4_simd.h"

//帧的大小：FRAME_SIZE = 64
#define FRAME_SIZE	global_data.param[PARAM_IMAGE_WIDTH]
//PARAM_MAX_FLOW_PIXEL=13
//搜索范围：SEARCH_SIZE = 4
#define SEARCH_SIZE	global_data.param[PARAM_MAX_FLOW_PIXEL] // maximum offset to search: 4 + 1/2 pixels
//什么意思
#define TILE_SIZE	8               						// x & y tile size
//遍历的一个轴的块的数量，总共5 * 5 = 25 个
#define NUM_BLOCKS	5 // x & y number of tiles to check

#define sign(x) (( x > 0 ) - ( x < 0 ))

uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y);

/****************************************
 *
 *		两帧图像进行比较：	寻找匹配点
 *
 *		usada8 %[result], r4, r5, %[result]的意思
 *		我猜测是：
 *				r4与r5的差的绝对值加上result再赋值给result，就是绝对值的差累加；
 */

// compliments of Adam Williams
#define ABSDIFF(frame1, frame2) \
({ \
 int result = 0; \
 //asm是C++中的一个关键字，用于在C++源码中内嵌汇编语言
 asm volatile( \
  "mov %[result], #0\n"           /* accumulator */ \

/*******************************************************
*
*			第二帧图像第0列(相对第二帧图像坐标)的8个数值、第4列(相对第二帧图像坐标)的8个数值分别
* 		  与第一帧图像第0列(相对第一帧图像坐标)的8个数值、第4列(相对第一帧图像坐标)的8个数值进行差的绝对值再累加
*
*		src：frame1
*		dst：frame2
*/
 \
  "ldr r4, [%[src], #0]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #0]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #4]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #4]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 1)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 1)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 1 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 1 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 2)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 2)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 2 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 2 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 3)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 3)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 3 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 3 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 4)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 4 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 4 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 5)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 5)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 5 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 5 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 6)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 6)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 6 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 6 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  "ldr r4, [%[src], #(64 * 7)]\n"        /* read data from address + offset*/ \
  "ldr r5, [%[dst], #(64 * 7)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
  "ldr r4, [%[src], #(64 * 7 + 4)]\n"        /* read data from address + offset */ \
  "ldr r5, [%[dst], #(64 * 7 + 4)]\n" \
  "usada8 %[result], r4, r5, %[result]\n"      /* difference */ \
 \
  : [result] "+r" (result) \
  : [src] "r" (frame1), [dst] "r" (frame2) \
  : "r4", "r5" \
  ); \
  \
 result; \
})

/**
 * @brief Computes the Hessian at a pixel location
 *
 * The hessian (second order partial derivatives of the image) is
 * a measure of the salience of the image at the appropriate
 * box filter scale. It allows to judge wether a pixel
 * location is suitable for optical flow calculation.
 *
 * @param image the array holding pixel data
 * @param x location of the pixel in x
 * @param y location of the pixel in y
 *
 * @return gradient magnitude
 */
static inline uint32_t compute_hessian_4x6(uint8_t *image, uint16_t x, uint16_t y, uint16_t row_size)
{
	// candidate for hessian calculation:
	uint16_t off1 = y*row_size + x;   	// First row of ones
	uint16_t off2 = (y+1)*row_size + x;   // Second row of ones
	uint16_t off3 = (y+2)*row_size + x;   // Third row of minus twos
	uint16_t off4 = (y+3)*row_size + x;   // Third row of minus twos
	uint16_t off5 = (y+4)*row_size + x;   // Third row of minus twos
	uint16_t off6 = (y+5)*row_size + x;   // Third row of minus twos
	uint32_t magnitude;

	// Uncentered for max. performance:
	// center pixel is in brackets ()

	//  1   1   1   1
	//  1   1   1   1
	// -2 (-2) -2  -2
	// -2  -2  -2  -2
	//  1   1   1   1
	//  1   1   1   1

	magnitude = __UADD8(*((uint32_t*) &image[off1 - 1]), *((uint32_t*) &image[off2 - 1]));
	magnitude -= 2*__UADD8(*((uint32_t*) &image[off3 - 1]), *((uint32_t*) &image[off4 - 1]));
	magnitude += __UADD8(*((uint32_t*) &image[off5 - 1]), *((uint32_t*) &image[off6 - 1]));

	return magnitude;
}


/**
 * @brief Compute the average pixel gradient of all horizontal and vertical steps
 *
 * TODO compute_diff is not appropriate for low-light mode images
 *
 * @param image ...
 * @param offX x coordinate of upper left corner of 8x8 pattern in image
 * @param offY y coordinate of upper left corner of 8x8 pattern in image
 */

/******************************************
 *
 *	寻找角点
 *
 *	参数1：输入图像
 *	参数2：8x8像素的左上角---》x
 *	参数3：8x8像素的左上角---》y
 *	参数4：	64
 */

static inline uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
	/* calculate position in image buffer */
//从(5+2)*64+(5+2)=455开始
//怎么感觉是：64*64的矩阵，上多出两行，左多出两列，是这样计算的呢？为啥？
	uint16_t off = (offY + 2) * row_size + (offX + 2); // we calc only the 4x4 pattern
	uint32_t acc;

	/* calc row diff */
//__USAD8 :无符号值的差的绝对值求和
//__USADA8:无符号值的差的绝对值求和累加
	acc = __USAD8 (*((uint32_t*) &image[off + 0 + 0 * row_size]), *((uint32_t*) &image[off + 0 + 1 * row_size]));
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 1 * row_size]), *((uint32_t*) &image[off + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 2 * row_size]), *((uint32_t*) &image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 0 + 1 * row_size] << 16 | image[off + 0 + 2 * row_size] << 8 | image[off + 0 + 3 * row_size];
	uint32_t col2 = (image[off + 1 + 0 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 1 + 2 * row_size] << 8 | image[off + 1 + 3 * row_size];
	uint32_t col3 = (image[off + 2 + 0 * row_size] << 24) | image[off + 2 + 1 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 | image[off + 2 + 3 * row_size];
	uint32_t col4 = (image[off + 3 + 0 * row_size] << 24) | image[off + 3 + 1 * row_size] << 16 | image[off + 3 + 2 * row_size] << 8 | image[off + 3 + 3 * row_size];

	/* calc column diff */
	acc = __USADA8(col1, col2, acc);
	acc = __USADA8(col2, col3, acc);
	acc = __USADA8(col3, col4, acc);

	return acc;

}

/**
 * @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 * @param acc array to store SAD distances for shift in every direction
 */

/***********************************
 *
 *
 */

static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
//所有的图像都是一维数组
	/* calculate position in image buffer */
//第一帧图像的下标
	uint16_t off1 = off1Y * row_size + off1X; // image1
//第二帧图像的下标
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	 * calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	 * every iteration is one line of the 8x8 field.
	 *
	 *  + - + - + - + - + - + - + - + - +
	 *  |   |   |   |   |   |   |   |   |
	 *  + - + - + - + - + - + - + - + - +
	 *
	 *
	 */

	for (uint16_t i = 0; i < 8; i++)
	{
		/*
		 * first column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  | x | x | x | x |   |   |   |   |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 * the 8 s values are from following positions for each pixel (X):
		 * (0-7)代表方向
		 *  + - + - + - +
		 *  +   5   7   +
		 *  + - + 6 + - +
		 *  +   4 X 0   +
		 *  + - + 2 + - +
		 *  +   3   1   +
		 *  + - + - + - +
		 *
		 *  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		 *
		 */

		/* compute average of two pixel values */
		s0 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+0) * row_size])));
		s1 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+1) * row_size])));
		s2 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i+1) * row_size])));
		s3 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+1) * row_size])));
		s4 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+0) * row_size])));
		s5 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i-1) * row_size])));
		s6 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i-1) * row_size])));
		s7 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i-1) * row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		/*
		 * finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		 *  + - + - + - +
		 *  |   |   |   |
		 *  + - 5 6 7 - +
		 *  |   4 X 0   |
		 *  + - 3 2 1 - +
		 *  |   |   |   |
		 *  + - + - + - +
		 */

		/* fill accumulation vector */
		acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		 * same for second column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  |   |   |   |   | x | x | x | x |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 */

		s0 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+0) * row_size])));
		s1 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+1) * row_size])));
		s2 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i+1) * row_size])));
		s3 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+1) * row_size])));
		s4 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+0) * row_size])));
		s5 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i-1) * row_size])));
		s6 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i-1) * row_size])));
		s7 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i-1) * row_size])));

		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}

/**
 * @brief Compute SAD of two 8x8 pixel windows.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 */
static inline uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	acc = __USAD8 (*((uint32_t*) &image1[off1 + 0 + 0 * row_size]), *((uint32_t*) &image2[off2 + 0 + 0 * row_size]));
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 0 * row_size]), *((uint32_t*) &image2[off2 + 4 + 0 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 1 * row_size]), *((uint32_t*) &image2[off2 + 0 + 1 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 1 * row_size]), *((uint32_t*) &image2[off2 + 4 + 1 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 2 * row_size]), *((uint32_t*) &image2[off2 + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 2 * row_size]), *((uint32_t*) &image2[off2 + 4 + 2 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 3 * row_size]), *((uint32_t*) &image2[off2 + 0 + 3 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 3 * row_size]), *((uint32_t*) &image2[off2 + 4 + 3 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 4 * row_size]), *((uint32_t*) &image2[off2 + 0 + 4 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 4 * row_size]), *((uint32_t*) &image2[off2 + 4 + 4 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 5 * row_size]), *((uint32_t*) &image2[off2 + 0 + 5 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 5 * row_size]), *((uint32_t*) &image2[off2 + 4 + 5 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 6 * row_size]), *((uint32_t*) &image2[off2 + 0 + 6 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 6 * row_size]), *((uint32_t*) &image2[off2 + 4 + 6 * row_size]), acc);

	acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 7 * row_size]), *((uint32_t*) &image2[off2 + 0 + 7 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 7 * row_size]), *((uint32_t*) &image2[off2 + 4 + 7 * row_size]), acc);

	return acc;
}

/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * and calculates the average offset of all.
 *
 * @param image1 previous image buffer
 * @param image2 current image buffer (new)
 * @param x_rate gyro x rate
 * @param y_rate gyro y rate
 * @param z_rate gyro z rate
 *
 * @return quality of flow calculation
 */

/************************************************
 *
 * 		光流计算最重要的一个公式了：
 *
 * 		参数1：第一帧图像
 * 		参数2：第二帧图像
 * 		参数3：陀螺仪x的速度
 * 		参数4：陀螺仪y的速度
 * 		参数5：陀螺仪z的速度
 * 		参数6：计算出的x的像素位移
 * 		参数7：计算出的y的像素位移
 *
 * 		图像大小：64 * 64
 */

uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y) {

	/* constants */
//SEARCH_SIZE=4
//搜索窗口大小
	const int16_t winmin = -SEARCH_SIZE;  //-4
	const int16_t winmax = SEARCH_SIZE;   //4
//直方图大小：2 * (4 + 4 + 1) + 1 = 19
	const uint16_t hist_size = 2*(winmax-winmin+1)+1;

	/* variables */
        uint16_t pixLo = SEARCH_SIZE + 1;                            //5
//FRAME_SIZE=64
        uint16_t pixHi = FRAME_SIZE - (SEARCH_SIZE + 1) - TILE_SIZE; //51 = 64-(4+1)-8
//NUM_BLOCKS=5
        uint16_t pixStep = (pixHi - pixLo) / NUM_BLOCKS + 1;         //10 = (51-5)/5+1
	uint16_t i, j;
	uint32_t acc[8]; // subpixels
//维数:19
	uint16_t histx[hist_size]; // counter for x shift
	uint16_t histy[hist_size]; // counter for y shift
//x方向的位移
	int8_t  dirsx[64]; // shift directions in x
	int8_t  dirsy[64]; // shift directions in y
//位移方向
	uint8_t  subdirs[64]; // shift directions of best subpixels
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;

	/* initialize with 0 */
//hist_size = 19
	for (j = 0; j < hist_size; j++) { histx[j] = 0; histy[j] = 0; }

	/* iterate迭代 over all patterns
	 */

/***************************************************************************************
 *
 * 		开启整个迭代过程
 *
 * 		pixLo:5		pixHi:51		pixStep:10
 */
/********************第一、二个循环*******************************
 *
 * 		第一、二个for循环：寻找第一帧图像的角点
 */
	for (j = pixLo; j < pixHi; j += pixStep)         //行
	{
		for (i = pixLo; i < pixHi; i += pixStep)	 //列
		{
			/* test pixel if it is suitable for flow tracking */
/*************测试像素是否适合光流跟踪			***********
			 *
			 *	寻找角点           5,15,25,35,45
			 *
			 *	参数1：输入图像
			 *	参数2：8x8像素的左上角---》x
			 *	参数3：8x8像素的左上角---》y
			 *	参数4：	64
			 *
			 *	返回值：差值累加和
			 */
			uint32_t diff = compute_diff(image1, i, j, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);

/***********************************
 *
 * 			判断diff 是否小于阈值30---------diff只有大于30才能成为角点
 *
 * 			continue表示满足if条件则提前结束循环体的后面的语句，回到循环条件判断是否再循环
			若没有角点，跳出此层循环，移动到下一个块j+pixStep；若有角点，继续运行下面代码
 */

			if (diff < global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD]) //阈值：30
			{
				continue;
			}
/*************************************/
			uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
			int8_t sumx = 0;  //范围：(-4,+4)
			int8_t sumy = 0;  //范围：(-4,+4)
			int8_t ii, jj;
/*************************************
 *
 * 	指向角点的指针：  (i,j表示角点在第i列第j行      ，，，  每行64个像素点)
 *
 * 				*base1 = 第一帧图像基地址  + j * 64 + i;
 */																		//参数：64
			uint8_t *base1 = image1 + j * (uint16_t) global_data.param[PARAM_IMAGE_WIDTH] + i;

/*********************第三、四个循环*****************************
 *
 *  第三、四个循环： 寻找第二帧图像中与第一帧图像匹配的角点(第一帧图像坐标固定不变，第二帧坐标会变，因为要进行遍历)
 *
 *  			遍历以角点为中心(0,0),范围(-4，+4)的像素点，也就是搜索9*9的大小的图像，寻找最相似的匹配点
 *					搜索范围比较小，因为这是小运动的算法，即运动幅度不大
 *						                   寻找的方法是ABSDIFF()
 */

//第3个循环遍历以第i列、第j行(也就是角点)为中心的上下各4行的像素
//winmin = -4		winmax = +4             搜索范围大小：(-4,+4)
			for (jj = winmin; jj <= winmax; jj++)
			{																	//参数：64

				uint8_t *base2 = image2 + (j+jj) * (uint16_t) global_data.param[PARAM_IMAGE_WIDTH] + i;
//第4个循环遍历以第i列、第j行(也就是角点)为中心的左右各4列的像素
				for (ii = winmin; ii <= winmax; ii++)
				{
//					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
					uint32_t temp_dist = ABSDIFF(base1, base2 + ii);
					if (temp_dist < dist)
					{
						sumx = ii;  //匹配块左上角的坐标 ： x
						sumy = jj;	//匹配块左上角的坐标 ： y
//得到dist，用这个判别 块 是否匹配
						dist = temp_dist;
					}
				}
			}

/**********************************************************************
 *
 * /* 综上，寻找匹配块就是寻找第一帧图像的角点所对应第二帧图像中的位置，遍历以第一帧图像的角点为中心而在第二帧图像(-4,+4)*(-4,+4)的每一个像素，
 * 并以该像素为左上角画一个8*8像素的块，分别计算该块的第0列和第4列和第一帧角点所对应块的第0列和第4列差值的
 * 绝对值再累加的值，其中最小值所对应的块就是匹配块
 */

/****************************************************************
 *
 * 	SAD:Sum of Absolute Differences，就是差的绝对值的和
 */
/***********************************************
 *
 * 	判别dist是否小于5000，若小于5000，则说明块可用，进入函数继续处理数据
 */
			/* acceptance SAD distance threshhold */
			if (dist < global_data.param[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD])  //参数：5000
			{
//sumx,sumy:匹配到的块在第二帧图像匹配块左上角的坐标
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

/*******************************
 *
 *
 *
 * 		   	参数1：第一帧图像
 * 		   	参数2：第二帧图像
 * 		   	参数3：第一帧图像的角点的左上角的坐标：x
 * 		   	参数4：第一帧图像的角点的左上角的坐标：y
 * 		   	参数5：第二帧图像匹配块的左上角的坐标：x
 * 		   	参数6：第二帧图像匹配块的左上角的坐标：y
 * 		   	参数7：数组：acc   维数：8
 * 		   	参数8：图像大小的行宽：64
 *
 */																									//参数：64
/*************************************************************
 *
 * 			计算位移方向:compute_subpixel()
 *
 * 			返回：acc[0-7]:
 * 						0-7代表7个方向；8代表没有方向。
 * 						acc[0-7]中8个值谁的最小，代表方向是朝哪个方向
 */
				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);

				uint32_t mindist = dist; // best SAD until now
				uint8_t mindir = 8; // direction 8 for no direction

//查找8个方向值当中哪个最小，就是现在的方向：k
				for(uint8_t k = 0; k < 8; k++)
				{
					if (acc[k] < mindist)
					{
						// SAD becomes better in direction k
						mindist = acc[k];
						mindir = k;
					}
				}

//方向向量:dirsx[64]   ;;   meancount
//sumx、sumy：匹配到的块左上角的 坐标x,y

				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
//位移方向:mindir
				subdirs[meancount] = mindir;
				meancount++;

/**************************************************************
 *
 *			计算位移方向直方图    :   histx[0-18]、histy[0-18]
 *
 *			根据上式返回的方向(0-7)
 */
/*
 *
 * 每次都有histx[hist_index_x]++;   histy[hist_index_y]++;，
 *	 变动的只是数组的下标，以便统计各方向的数量
 *
 */
				/* feed histogram filter*/

//hist_index_x范围：(1,17)---》(1,3,5,7,9,11,13,15,17)
				uint8_t hist_index_x = 2*sumx + (winmax-winmin+1);//winmax-winmin+1 = 9，

//i:第一帧图像的角点的左上角的坐标：x
//0、1、7代表右移
//3、4、5代表左移
				if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) hist_index_x += 1;
				if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) hist_index_x += -1;

//hist_index_y范围：(1,17)---》(1,3,5,7,9,11,13,15,17)
				uint8_t hist_index_y = 2*sumy + (winmax-winmin+1);

//5、6、7代表上移
//1、2、3代表下移
				if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) hist_index_y += -1;
				if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) hist_index_y += 1;

				histx[hist_index_x]++;
				histy[hist_index_y]++;

/*    综上：
 * 				   hist_index_x最小=2*(-4)+(4-(-4)+1)=1
				 * hist_index_x最大=2*4+(4-(-4)+1)=17
				 * 下面再+1或-1，范围就是0-18，共19个数
				 * 与uint16_t histx[hist_size]定义一致，即uint16_t histx[19]
				 */
		/* x方向有19个元素的数组,如下：
		 *  数字表示第二帧图像匹配块对应的左上角的像素的位置
		 * 			(-4,-3,-2,-1,0,1,2,3,4) 与  (1,3,5,7,9,11,13,15,17)相对应
		 *  数字两边的空格表示方向，左右移动
		 *
		 *  也就19个数中，偶数代表方向，奇数代表匹配块左上角像素坐标
		 *
		 *  +-+-+-+-+-+-+-+-+-+-+-+--+-+--+-+--+-+--+--+
		 *  | |1| |3| |5| |7| |9| |11| |13| |15| |17|  |
		 *  +-+-+-+-+-+-+-+-+-+-+-+--+-+--+-+--+-+--+--+
		 *  | | | | | | | | | | | |  | |  | |  | |  |  |
		 *  +-+-+-+-+-+-+-+-+-+-+-+--+-+--+-+--+-+--+--+
		 *  | |3| | | | | | | | | |  | |  | |  | |  |  |
		 *  +-+-+-+-+-+-+-+-+-+-+-+--+-+--+-+--+-+--+--+
		 *  '					'					'
		 *  '					'					'
		 *  '					'					'
		 *  ||15| | | | | | | | | |  | |  | |  | |  |  |
		 *  +-+-+-+-+-+-+-+-+-+-+-+--+-+--+-+--+-+--+--+
		 *  | | | | | | | | | | | |  | |  | |  | |  |  |
		 *  +-+-+-+-+-+-+-+-+-+-+-+--+-+--+-+--+-+--+--+
		 *  ||17| | | | | | | | | |  | |  | |  | |  |  |
		 *  +-+-+-+-+-+-+-+-+-+-+-+--+-+--+-+--+-+--+--+
		 *
		 *  y方向也有19个元素的数组，上下移动
		 */

			}

/********************************************
 *
 * 		至此，第三、四个for循环执行完成，表明已经找到一个角点和对应的匹配快；
 *
 * 		继续执行第一、二个for循环，来寻找其余的角点和匹配快
 */

		}
	}

/********************************************************************
 *
 * 			总结：至此，四个for()循环已经结束，第一帧图像的角点找到，第二帧图像中与第一帧图像角点匹配的块找到，
 * 				总共meancount个角点和对应的匹配块
 * 					块相对角点的移动方向进行累加计算:histx[0-18]、histy[0-18]
 */

	/* create flow image if needed (image1 is not needed anymore)
	 * -> can be used for debugging purpose
	 */
//	if (global_data.param[PARAM_USB_SEND_VIDEO] )//&& global_data.param[PARAM_VIDEO_USB_MODE] == FLOW_VIDEO)
//	{
//
//		for (j = pixLo; j < pixHi; j += pixStep)
//		{
//			for (i = pixLo; i < pixHi; i += pixStep)
//			{
//
//				uint32_t diff = compute_diff(image1, i, j, (uint16_t) global_data.param[PARAM_IMAGE_WIDTH]);
//				if (diff > global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD])
//				{
//					image1[j * ((uint16_t) global_data.param[PARAM_IMAGE_WIDTH]) + i] = 255;
//				}
//
//			}
//		}
//	}

/*************************************************************
 *
 * 		匹配块数量大于10时，进行计算
 *
 * 			meancount:匹配块数量
 */
	/* evaluate flow calculation */
	if (meancount > 10)
	{

//第二帧图像匹配块相对第一帧图像角点的平均坐标位移，
		meanflowx /= meancount;
		meanflowy /= meancount;

		int16_t maxpositionx = 0;
		int16_t maxpositiony = 0;
		uint16_t maxvaluex = 0;
		uint16_t maxvaluey = 0;

//寻找histx[0-18]、histx[0-18]直方图当中的峰值
		/* position of maximal histogram peak */
		for (j = 0; j < hist_size; j++)
		{
			if (histx[j] > maxvaluex)
			{
				maxvaluex = histx[j];
				maxpositionx = j;
			}
			if (histy[j] > maxvaluey)
			{
				maxvaluey = histy[j];
				maxpositiony = j;
			}
		}

		/* check if there is a peak value in histogram */
		if (1) //(histx[maxpositionx] > meancount / 6 && histy[maxpositiony] > meancount / 6)
		{
/*****************************************************
 *
 * 			在这里选择直方图还是另外一种方法：
 *			根据if()当中的条件
 *			默认，直方图方法不选择，跳过，而进入else
 */
/************************************
 *
 * 		直方图滤波方法求取光流
 */
			if (FLOAT_AS_BOOL(global_data.param[PARAM_BOTTOM_FLOW_HIST_FILTER])) //参数等于0
			{

				/* use histogram filter peak value */
				uint16_t hist_x_min = maxpositionx;
				uint16_t hist_x_max = maxpositionx;
				uint16_t hist_y_min = maxpositiony;
				uint16_t hist_y_max = maxpositiony;

/***************
 *
 * 		x方向
 */

				/* x direction */

//若下标范围是: 1  <maxpositionx < 17
				if (maxpositionx > 1 && maxpositionx < hist_size-2) //1<   <17
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 2;
				}
				else if (maxpositionx == 0)          //0
				{
					hist_x_min = maxpositionx;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-1)//18
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx;
				}
				else if (maxpositionx == 1)           //1
				{
					hist_x_min = maxpositionx - 1;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-2)//17
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 1;
				}

/***************
 *
 * 		y方向
 */

				/* y direction */
				if (maxpositiony > 1 && maxpositiony < hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == 0)
				{
					hist_y_min = maxpositiony;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-1)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony;
				}
				else if (maxpositiony == 1)
				{
					hist_y_min = maxpositiony - 1;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 1;
				}

/***************
 *
 * 	直方图滤波处理
 */

				float hist_x_value = 0.0f;
				float hist_x_weight = 0.0f;

				float hist_y_value = 0.0f;
				float hist_y_weight = 0.0f;
//x:最多5个数据
				for (uint8_t h = hist_x_min; h < hist_x_max+1; h++)
				{
				//当前下标乘以当前值的累加
					hist_x_value += (float) (h*histx[h]);
				//当前值的累加
					hist_x_weight += (float) histx[h];
				}
//y:最多5个数据
				for (uint8_t h = hist_y_min; h<hist_y_max+1; h++)
				{
				//当前下标乘以当前值的累加
					hist_y_value += (float) (h*histy[h]);
				//当前值的累加
					hist_y_weight += (float) histy[h];
				}
//得到光流x,y
				histflowx = (hist_x_value/hist_x_weight - (winmax-winmin+1)) / 2.0f ;
				histflowy = (hist_y_value/hist_y_weight - (winmax-winmin+1)) / 2.0f;

			}

/*******************************
 *
 * 		平均值方法求取光流:		histflowx         histflowy
 */

			else
			{

				/* use average of accepted flow values */
				uint32_t meancount_x = 0;
				uint32_t meancount_y = 0;

				for (uint8_t h = 0; h < meancount; h++)  //meancount:光流匹配块
				{
					float subdirx = 0.0f;
//subdirs[]位移方向
//0、1、7代表右移
//3、4、5代表左移
					if (subdirs[h] == 0 || subdirs[h] == 1 || subdirs[h] == 7) subdirx = 0.5f;
					if (subdirs[h] == 3 || subdirs[h] == 4 || subdirs[h] == 5) subdirx = -0.5f;

//dirsx[0-meancount]:当前匹配块的左上角的坐标:x              不懂为啥加0.5？？？？

					histflowx += (float)dirsx[h] + subdirx;
					meancount_x++;

					float subdiry = 0.0f;
//subdirs[]位移方向
//5、6、7代表上移
//1、2、3代表下移
					if (subdirs[h] == 5 || subdirs[h] == 6 || subdirs[h] == 7) subdiry = -0.5f;
					if (subdirs[h] == 1 || subdirs[h] == 2 || subdirs[h] == 3) subdiry = 0.5f;

//dirsy[0-meancount]:当前匹配块的左上角的坐标:y

					histflowy += (float)dirsy[h] + subdiry;
					meancount_y++;
				}

				histflowx /= meancount_x;
				histflowy /= meancount_y;

			}

/********************************************
 *
 * 		陀螺仪旋转补偿
 */

			/* compensate rotation */
			/* calculate focal_length in pixel */      //参数 =16
//这是啥:等于666.7
			const float focal_length_px = (global_data.param[PARAM_FOCAL_LENGTH_MM]) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled

			/*
			 * gyro compensation
			 * the compensated value is clamped to
			 * the maximum measurable flow value (param BFLOW_MAX_PIX) +0.5
			 * (sub pixel flow can add half pixel to the value)
			 *
			 * -y_rate gives x flow
			 * x_rates gives y_flow
			 */

//陀螺仪补偿标志，默认:0，也就是不补偿

			if (FLOAT_AS_BOOL(global_data.param[PARAM_BOTTOM_FLOW_GYRO_COMPENSATION]))//0
			{
				if(fabsf(y_rate) > global_data.param[PARAM_GYRO_COMPENSATION_THRESHOLD])
				{
					/* calc pixel of gyro */
					float y_rate_pixel = y_rate * (get_time_between_images() / 1000000.0f) * focal_length_px;
					float comp_x = histflowx + y_rate_pixel;

                    /* clamp value to maximum search window size plus half pixel from subpixel search */
                    if (comp_x < (-SEARCH_SIZE - 0.5f))
                    	*pixel_flow_x = (-SEARCH_SIZE - 0.5f);
                    else if (comp_x > (SEARCH_SIZE + 0.5f))
                    	*pixel_flow_x = (SEARCH_SIZE + 0.5f);
                    else
                    	*pixel_flow_x = comp_x;
				}
				else
				{
					*pixel_flow_x = histflowx;
				}

				if(fabsf(x_rate) > global_data.param[PARAM_GYRO_COMPENSATION_THRESHOLD])
				{
					/* calc pixel of gyro */
					float x_rate_pixel = x_rate * (get_time_between_images() / 1000000.0f) * focal_length_px;
					float comp_y = histflowy - x_rate_pixel;

					/* clamp value to maximum search window size plus/minus half pixel from subpixel search */
					if (comp_y < (-SEARCH_SIZE - 0.5f))
						*pixel_flow_y = (-SEARCH_SIZE - 0.5f);
					else if (comp_y > (SEARCH_SIZE + 0.5f))
						*pixel_flow_y = (SEARCH_SIZE + 0.5f);
					else
						*pixel_flow_y = comp_y;
				}
				else
				{
					*pixel_flow_y = histflowy;
				}

				/* alternative compensation */
//				/* compensate y rotation */
//				*pixel_flow_x = histflowx + y_rate_pixel;
//
//				/* compensate x rotation */
//				*pixel_flow_y = histflowy - x_rate_pixel;

			} else
			{
//没有陀螺仪补偿
				/* without gyro compensation */
				*pixel_flow_x = histflowx;
				*pixel_flow_y = histflowy;
			}

		}
//不进入else
		else
		{
			*pixel_flow_x = 0.0f;
			*pixel_flow_y = 0.0f;
			return 0;
		}
	}
//若匹配块小于10，则进入else
	else
	{
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}

//光流质量 ：qual   ,    范围：(0,255)

	/* calc quality */
	uint8_t qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS*NUM_BLOCKS));

	return qual;
}
