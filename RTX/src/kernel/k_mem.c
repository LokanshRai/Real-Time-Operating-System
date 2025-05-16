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
 * @file        k_mem.c
 * @brief       Kernel Memory Management API C Code
 *
 * @version     V1.2021.01.lab2
 * @authors     Yiqing Huang
 * @date        2021 JAN
 *
 * @note        skeleton code
 *
 *****************************************************************************/

/** 
 * @brief:  k_mem.c kernel API implementations, this is only a skeleton.
 * @author: Yiqing Huang
 */

// #define DEBUG_0

#include "k_mem.h"
#include "Serial.h"
#ifdef DEBUG_0
#include "printf.h"
#endif  /* DEBUG_0 */

/*
 *==========================================================================
 *                            GLOBAL VARIABLES
 *==========================================================================
 */
// kernel stack size, referred by startup_a9.s
const U32 g_k_stack_size = K_STACK_SIZE;
// task proc space stack size in bytes, referred by system_a9.cs
const U32 g_p_stack_size = U_STACK_SIZE;

// task kernel stacks
U32 g_k_stacks[MAX_TASKS][K_STACK_SIZE >> 2] __attribute__((aligned(8)));

//process stack for tasks in SYS mode
// U32 g_p_stacks[MAX_TASKS][U_STACK_SIZE >> 2] __attribute__((aligned(8)));

// Pointer that holds value to memory address of free memory list
U32 *head = NULL;

// Hold 8-byte aligned address for OS_END
U32 end_addr = NULL;

typedef struct MemChunk {
    U32 size; // Size of chunk data
    U32 padding; // Adds padding to memchunk to be 8-byte aligned
    U64 owner_tid;
} memChunk;

// Check if pointer and non-pointer version same size
typedef struct FreeHDR {
	memChunk chunk;
	U32 *next;
} freeHDR; 

typedef struct AllocHDR {
    memChunk chunk;
} allocHDR;

// Min size of one chunk available to be placed into memory
const U32 min_chunk_size = sizeof(memChunk) + sizeof(U64);

/*
 *===========================================================================
 *                            FUNCTIONS
 *===========================================================================
 */

U32* k_alloc_k_stack(task_t tid)
{
    return g_k_stacks[tid+1];
}

U32* k_alloc_p_stack(task_t tid)
{
    task_t old_tid = gp_current_task->tid;
    gp_current_task->tid = TID_NULL;
    U32 *sp = k_mem_alloc(g_tcbs[tid].u_stack_size);
    // // Could check this outside of function
    // if (sp == NULL) {
    //     return NULL;
    // }
    sp = (U32 *)((U32)sp + (U32)g_tcbs[tid].u_stack_size);
    gp_current_task->tid = old_tid;
    return sp;
}

int k_mem_init(void) {
#ifdef DEBUG_0
    printf("k_mem_init: image ends at 0x%x\r\n", end_addr);
    printf("k_mem_init: RAM ends at 0x%x\r\n", RAM_END);
#endif /* DEBUG_0 */

    end_addr = (U32) &Image$$ZI_DATA$$ZI$$Limit;
    end_addr += 8 - (end_addr) & 0x7; // Make end_addr 8-byte alligned

    // Check if mem_init alraedy called
    if (head != NULL) {
        return RTX_ERR;
    }

    // Check if enough space for at least one free-memory header and two unsigned
    // ints (head and one 8-byte alligned space)
    if(RAM_END - end_addr + 1 < min_chunk_size){
        return RTX_ERR;
    }

    // Initalize first free-memory region
    freeHDR *new_free_header = (freeHDR *)(end_addr);
    head = (U32 *) new_free_header;
    new_free_header->chunk.size = RAM_END - ((U32)new_free_header + sizeof(memChunk)) + 1;
    new_free_header->next = (U32 *)RAM_END;

    return RTX_OK;
}

void* k_mem_alloc(size_t size) {
#ifdef DEBUG_0
    printf("k_mem_alloc: requested memory size = %d\r\n", size);
    printf("k_mem_alloc: allocing mem for tid = %d\r\n", gp_current_task->tid);
#endif /* DEBUG_0 */
    // Check if input size is zero or head not initalized and thus k_mem_init not called
    if (size == 0 || head == NULL) {
        return NULL;
    }

    freeHDR *curr_free_header = (freeHDR *)head;
    freeHDR *prev_free_header = curr_free_header;
    (size) += 8 - (size) & 0x7; // Make size a multiple of 8
    while ((U32) curr_free_header != RAM_END) {
        // If more than enough memory is available for the requested size and a free HDR region
    	if (curr_free_header->chunk.size > size && (curr_free_header->chunk.size - size) >= min_chunk_size) {
            // Setup new free-memory chunk
            freeHDR *new_free_header = (freeHDR *)((char *) curr_free_header + sizeof(allocHDR) + size);
            new_free_header->next = curr_free_header->next;
            new_free_header->chunk.size = curr_free_header->chunk.size - size - sizeof(allocHDR);

            // If not allocating memory at header, update previous freeHDR header to point towards new free-memory chunk
            if (curr_free_header != (freeHDR *)head) {
                prev_free_header->next = (U32 *) new_free_header;
            }
            // If allocating memory at header, set new head pointer to new freeHDR header
            else {
                head = (U32 *) new_free_header;
            }

            // Setup new allocated-memory chunk
            allocHDR *new_alloc_header = (allocHDR *)(curr_free_header); 
            new_alloc_header->chunk.owner_tid = gp_current_task->tid;
            new_alloc_header->chunk.size = size;
            
            return (void *) ((U32)new_alloc_header + sizeof(allocHDR));
        }

        // If more than enough memory is available for size but not enough for a free HDR region,
        // or exactly enough memory is available
        else if (curr_free_header->chunk.size >= size && (curr_free_header->chunk.size - size) < min_chunk_size) {
            // If not at header, set previous free-memory chunk header to point towards new free-memory chunk
            if (curr_free_header != (freeHDR *)head) {      
                prev_free_header->next = curr_free_header->next;
            }
            // If at head but have more free-memory chunks left, update head to point to next free-memory chunk
            else if ((U32)(curr_free_header->next) != RAM_END) {
                head = (U32 *)(curr_free_header->next);
            }
            // If at head and no other free-memory chunks left, set head to point towards RAM_END to indicate no memory left
            else {
                head = (U32 *)RAM_END;
            }

            // Setup new allocated-memory chunk
            // Free-mem chunk header is simply replaced with allocated-mem chunk header. Size is preserved.
            allocHDR *new_alloc_header = (allocHDR *)(curr_free_header);
            new_alloc_header->chunk.owner_tid = gp_current_task->tid;
            new_alloc_header->chunk.size = curr_free_header->chunk.size; 

            return (void *) ((U32)new_alloc_header + sizeof(allocHDR));
        }

        prev_free_header = curr_free_header;
        curr_free_header = (freeHDR *)(curr_free_header->next);
    }

    return NULL;
}


int k_mem_dealloc(void *ptr) {
#ifdef DEBUG_0
    printf("k_mem_dealloc: freeing 0x%x\r\n", (U32) ptr);
    printf("k_mem_dealloc: freeing mem for tid = %d\r\n", gp_current_task->tid);
#endif /* DEBUG_0 */
    allocHDR *addr_alloc_header = (allocHDR *)((U32)ptr - sizeof(allocHDR));
    freeHDR *new_free_header = (freeHDR *) addr_alloc_header;
    
    freeHDR *prev_free_header = (freeHDR *)head;
    freeHDR *curr_free_header = prev_free_header;

    // Invalid Pointer Cases:
    // Check if ptr is NULL
    // Check if address is in OS_MEM and includes checking if the initial head is selected
    // Check if address is beyond RAM_END
    // If head not initalized and thus k_mem_init not called
    if (ptr == NULL || (U32 *)addr_alloc_header < (U32 *) end_addr 
        || (U32 *)addr_alloc_header >= (U32 *) RAM_END || head == NULL 
        || gp_current_task->tid != addr_alloc_header->chunk.owner_tid) {
        return RTX_ERR;
    }
    
    // Case where memChunk is before head [handles case with no free memory]
    if ((memChunk *)addr_alloc_header < (memChunk *) head) {
        memChunk *current_mem_chunk = (memChunk *) (end_addr);
        // Check if current memChunk address matches selected address 
        while (current_mem_chunk != (memChunk *)addr_alloc_header) { 
            // Check if address is between two mem chunks, if it is means that address is invalid
            if (current_mem_chunk > (memChunk *)addr_alloc_header) {
                return RTX_ERR;
            }
            current_mem_chunk = (memChunk *)((U32) current_mem_chunk + sizeof(memChunk) + current_mem_chunk->size);
        }

        // Construct new freeHDR chunk
        new_free_header->chunk.size = addr_alloc_header->chunk.size; 
        new_free_header->next = (U32 *)head;
        head = (U32 *) new_free_header;

        freeHDR *prev = (freeHDR *)head;
        freeHDR *addr = (freeHDR *)(((freeHDR *)head)->next);
        if ((U32)prev + sizeof(memChunk) + prev->chunk.size == (U32)addr) {
            prev->next = addr->next;
            prev->chunk.size += sizeof(memChunk) + addr->chunk.size;
        }
        return RTX_OK;
    }

    while ((memChunk *)curr_free_header < (memChunk *)addr_alloc_header) {
        prev_free_header = curr_free_header;
        curr_free_header = (freeHDR *)(curr_free_header->next);
    }

    if ((memChunk *)curr_free_header == (memChunk *)addr_alloc_header) {
        return RTX_ERR;
    }


    memChunk *current_mem_chunk = (memChunk *) prev_free_header;
    // Check if current memChunk address matches selected address 
    while (current_mem_chunk != (memChunk *)addr_alloc_header) { 
        // Check if address is between two mem chunks, if it is means that address is invalid
        if (current_mem_chunk > (memChunk *)addr_alloc_header) {
            return RTX_ERR;
        }
        current_mem_chunk = (memChunk *)((U8 *) current_mem_chunk + sizeof(memChunk) + current_mem_chunk->size);
    }

    // Construct new freeHDR chunk
    new_free_header->chunk.size = addr_alloc_header->chunk.size;  // Check if size assignment really needed
    new_free_header->next = (U32 *)curr_free_header; // Check if curr is head for edge case
    prev_free_header->next = (U32 *)new_free_header;

    if ((U32)new_free_header + sizeof(memChunk) + new_free_header->chunk.size == (U32)curr_free_header) {
        new_free_header->next = curr_free_header->next;
        new_free_header->chunk.size += sizeof(memChunk) + curr_free_header->chunk.size;
    }

    if ((U32)prev_free_header + sizeof(memChunk) + prev_free_header->chunk.size == (U32)new_free_header) {
        prev_free_header->next = new_free_header->next;
        prev_free_header->chunk.size += sizeof(memChunk) + new_free_header->chunk.size;
    }

    return RTX_OK;
}

int k_mem_count_extfrag(size_t size) {
#ifdef DEBUG_0
    printf("k_mem_extfrag: size = %d\r\n", size);
#endif /* DEBUG_0 */
    freeHDR *curr_free_header = (freeHDR *)head;
    int count = 0;
    while((U32)curr_free_header != RAM_END){
        if (curr_free_header->chunk.size + sizeof(memChunk) < size){
            count ++;
        }
        curr_free_header = (freeHDR *) (curr_free_header->next);
    }
    return count;
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
