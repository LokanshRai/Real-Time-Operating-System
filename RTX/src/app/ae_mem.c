/*
 ****************************************************************************
 *
 *                  UNIVERSITY OF WATERLOO ECE 350 RTOS LAB
 *
 *                     Copyright 2020-2021 Yiqing Huang
 *                          All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice and the following disclaimer.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************
 */

/**************************************************************************//**
 * @file        ae_mem.c
 * @brief       memory lab auto-tester
 *
 * @version     V1.2021.01
 * @authors     Yiqing Huang
 * @date        2021 JAN
 *
 *****************************************************************************/

#include "rtx.h"
#include "Serial.h"
#include "printf.h"
#include "ae.h"

#if TEST == -1

int test_mem(void) {
	unsigned int start = timer_get_current_val(2);
	printf("NOTHING TO TEST.\r\n");
	unsigned int end = timer_get_current_val(2);

	// Clock counts down
	printf("This took %u us\r\n", start - end);
	return TRUE;
}

#endif
#if TEST == 1
BOOL test_coalescing_free_regions_using_count_extfrag() {
	void * p1 = k_mem_alloc(32);
	void * p2 = k_mem_alloc(32);

	unsigned int header_size = (unsigned int)p2 - (unsigned int)p1 - 32;

	void * p3 = k_mem_alloc(32);
	void * p4 = k_mem_alloc(32);
	void * p5 = k_mem_alloc(32);
	void * p6 = k_mem_alloc(32);
	void * p7 = k_mem_alloc(32);
	void * p8 = k_mem_alloc(32);
	void * p9 = k_mem_alloc(32);

	int size_33_plus_one_header = k_mem_count_extfrag(32 + header_size + 1);
	int size_97_plus_three_header = k_mem_count_extfrag(96 + 3*header_size + 1);

	if((size_33_plus_one_header!=0) || (size_97_plus_three_header!=0)) {
		printf("test_coalescing_free_regions_using_count_extfrag: 1. Either mem_alloc or mem_count_extfrag has failed.\r\n");
		k_mem_dealloc(p1);
		k_mem_dealloc(p2);
		k_mem_dealloc(p3);
		k_mem_dealloc(p4);
		k_mem_dealloc(p5);
		k_mem_dealloc(p6);
		k_mem_dealloc(p7);
		k_mem_dealloc(p8);
		k_mem_dealloc(p9);
		return FALSE;
	}

	k_mem_dealloc(p2);
	k_mem_dealloc(p4);
	k_mem_dealloc(p6);
	k_mem_dealloc(p8);

	size_33_plus_one_header = k_mem_count_extfrag(32 + header_size + 1);
	size_97_plus_three_header = k_mem_count_extfrag(96 + 3*header_size + 1);

	if((size_33_plus_one_header!=4) || (size_97_plus_three_header!=4)) {
		printf("test_coalescing_free_regions_using_count_extfrag: 2. Either mem_dealloc or coalescing has failed.\r\n");
		k_mem_dealloc(p1);
		k_mem_dealloc(p3);
		k_mem_dealloc(p5);
		k_mem_dealloc(p7);
		k_mem_dealloc(p9);
		return FALSE;
	}

	k_mem_dealloc(p3);
	k_mem_dealloc(p7);

	size_33_plus_one_header = k_mem_count_extfrag(32 + header_size + 1);
	size_97_plus_three_header = k_mem_count_extfrag(96 + 3*header_size + 1);

	if((size_33_plus_one_header!=0) || (size_97_plus_three_header!=2)) {
		printf("test_coalescing_free_regions_using_count_extfrag: 3. Either mem_dealloc or coalescing has failed.\r\n");
		k_mem_dealloc(p1);
		k_mem_dealloc(p5);
		k_mem_dealloc(p9);
		return FALSE;
	}

	k_mem_dealloc(p1);
	k_mem_dealloc(p5);
	k_mem_dealloc(p9);

	int size_289_plus_nine_header = k_mem_count_extfrag(288 + 9*header_size + 1);

	if(size_289_plus_nine_header!=0) {
		printf("test_coalescing_free_regions_using_count_extfrag: 4. Either mem_dealloc or coalescing has failed.\r\n");
		return FALSE;
	}

	return TRUE;
}


int test_mem(void) {

	int test_coalescing_free_regions_result = test_coalescing_free_regions_using_count_extfrag();

	return test_coalescing_free_regions_result;
}
#endif

#if TEST == 2

#define N 10

#define CODE_MEM_INIT -1
#define CODE_MEM_ALLOC -2
#define CODE_MEM_DEALLOC -3
#define CODE_HEAP_LEAKAGE_1 -4
#define CODE_HEAP_LEAKAGE_2 -5
#define CODE_SUCCESS 0

int heap_leakage_test() {

	char *p_old[N], *p_new[N];

	// Step 1: Allocate memory
	for (int i = 0; i < N; i++) {
		p_old[i] = (char*) k_mem_alloc(i * 256 + 255);

		// pointer to allocated memory should not be null
		// starting address of allocated memory should be four-byte aligned
		if (p_old[i] == NULL || ((unsigned int) p_old[0] & 3))
			return CODE_MEM_ALLOC;

		if (i > 0) {
			// adjacent allocated memory should not conflict
			if (p_old[i - 1] + 256 * i >= p_old[i])
				return CODE_MEM_ALLOC;
		}
	}

	// Step 2: De-allocate memory
	for (int i = 0; i < N; i++) {
		if (k_mem_dealloc(p_old[i]) == -1) {
			return CODE_MEM_DEALLOC;
		}
	}

	// Step 3: Memory Leakage
	for (int i = 0; i < N; i++) {
		p_new[i] = (char*) k_mem_alloc((N - i) * 256 - 1);

		// pointer to allocated memory should not be null
		// starting address of allocated memory should be four-byte aligned
		if (p_new[i] == NULL || ((unsigned int) p_new[0] & 3))
			return CODE_HEAP_LEAKAGE_1;

		if (i > 0) {
			// adjacent allocated memory should not conflict
			if (p_new[i - 1] + 256 * (N - i + 1) >= p_new[i])
				return CODE_HEAP_LEAKAGE_1;
		}
	}

	// the total occupied area in the re-allocation
	// should be the same as in the initial allocation
	if ((p_old[0] != p_new[0])
			|| (p_old[N - 1] + N * 256 != p_new[N - 1] + 256)) {
		return CODE_HEAP_LEAKAGE_2;
	}

	for (int i = 0; i < N; i++) {
		k_mem_dealloc(p_new[i]);
	}

	return CODE_SUCCESS;
}

int test_mem(void) {

	int result = heap_leakage_test();
	switch (result) {
	case CODE_MEM_INIT:
	case CODE_MEM_ALLOC:
		printf("Err: Basic allocation fails.\r\n");
		break;
	case CODE_MEM_DEALLOC:
		printf("Err: Basic deallocation fails.\r\n");
		break;
	case CODE_HEAP_LEAKAGE_1:
		printf("Err: Reallocation fails.\r\n");
		break;
	case CODE_HEAP_LEAKAGE_2:
		printf("Err: Heap memory is leaked.\r\n");
		break;
	case CODE_SUCCESS:
		printf("No heap leakage.\r\n");
		break;
	default:
	}

	return result == CODE_SUCCESS;
}
#endif

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
