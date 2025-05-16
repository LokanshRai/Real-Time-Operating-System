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
 * @file        k_task.c
 * @brief       task management C file
 *              l2
 * @version     V1.2021.01
 * @authors     Yiqing Huang
 * @date        2021 JAN
 *
 * @attention   assumes NO HARDWARE INTERRUPTS
 * @details     The starter code shows one way of implementing context switching.
 *              The code only has minimal sanity check.
 *              There is no stack overflow check.
 *              The implementation assumes only two simple privileged task and
 *              NO HARDWARE INTERRUPTS.
 *              The purpose is to show how context switch could be done
 *              under stated assumptions.
 *              These assumptions are not true in the required RTX Project!!!
 *              Understand the assumptions and the limitations of the code before
 *              using the code piece in your own project!!!
 *
 *****************************************************************************/

//#include "VE_A9_MP.h"
#include "Serial.h"
#include "k_task.h"
#include "k_rtx.h"

// #define DEBUG_0

#ifdef DEBUG_0
#include "printf.h"
#endif /* DEBUG_0 */

/*
 *==========================================================================
 *                            GLOBAL VARIABLES
 *==========================================================================
 */

typedef struct Priority_Entry {
    TCB *first_task;
    TCB *last_task;
    U8 next_prio;
    U8 prev_prio;
} prio;

TCB             *gp_current_task = NULL;	// the current RUNNING task
TCB             g_tcbs[MAX_TASKS];			// an array of TCBs
RTX_TASK_INFO   g_null_task_info;			// The null task info
U32             g_num_active_tasks = 0;		// number of non-dormant tasks

prio prio_array[256];
U8 prio_head;
TCB *dormant_head = NULL;
task_t tid_count = 0;
task_t actual_KCD_tid = MAX_TASKS;

enum scheduling_states{CREATE_PRIOCHANGE_UNBLOCK, YIELD_PRIOCHANGE, EXIT};
enum scheduling_states scheduling_status = CREATE_PRIOCHANGE_UNBLOCK;

/*---------------------------------------------------------------------------
The memory map of the OS image may look like the following:

                       RAM_END+---------------------------+ High Address
                              |                           |
                              |                           |
                              |    Free memory space      |
                              |   (user space stacks      |
                              |         + heap            |
                              |                           |
                              |                           |
                              |                           |
 &Image$$ZI_DATA$$ZI$$Limit-->|---------------------------|-----+-----
                              |         ......            |     ^
                              |---------------------------|     |
                              |      U_STACK_SIZE         |     |
             g_p_stacks[15]-->|---------------------------|     |
                              |                           |     |
                              |  other kernel proc stacks |     |
                              |---------------------------|     |
                              |      U_STACK_SIZE         |  OS Image
              g_p_stacks[2]-->|---------------------------|     |
                              |      U_STACK_SIZE         |     |
              g_p_stacks[1]-->|---------------------------|     |
                              |      U_STACK_SIZE         |     |
              g_p_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |                           |  OS Image
                              |---------------------------|     |
                              |      K_STACK_SIZE         |     |                
             g_k_stacks[15]-->|---------------------------|     |
                              |                           |     |
                              |     other kernel stacks   |     |                              
                              |---------------------------|     |
                              |      K_STACK_SIZE         |  OS Image
              g_k_stacks[2]-->|---------------------------|     |
                              |      K_STACK_SIZE         |     |                      
              g_k_stacks[1]-->|---------------------------|     |
                              |      K_STACK_SIZE         |     |
              g_k_stacks[0]-->|---------------------------|     |
                              |   other  global vars      |     |
                              |---------------------------|     |
                              |        TCBs               |  OS Image
                      g_tcbs->|---------------------------|     |
                              |        global vars        |     |
                              |---------------------------|     |
                              |                           |     |          
                              |                           |     |
                              |                           |     |
                              |                           |     V
                     RAM_START+---------------------------+ Low Address
    
---------------------------------------------------------------------------*/ 

/*
 *===========================================================================
 *                            FUNCTIONS
 *===========================================================================
 */

/**************************************************************************//**
 * @brief   scheduler, pick the TCB of the next to run task
 *
 * @return  TCB pointer of the next to run task
 * @post    gp_curret_task is updated
 *
 *****************************************************************************/

TCB *scheduler(void)
{
    TCB *task;
    switch(scheduling_status) {
        // Case where adding a task or changing the priority of another task
        case CREATE_PRIOCHANGE_UNBLOCK:
            // If priority of prio_head task is higher than current task
            if (prio_head < gp_current_task->prio) {
                task = prio_array[prio_head].first_task;
                
                if (prio_array[prio_head].first_task == prio_array[prio_head].last_task) {
                    prio_array[prio_head].last_task = NULL;
                    prio_array[prio_head].first_task = NULL;
                    prio_head = prio_array[prio_head].next_prio;
                }
                else {
                    prio_array[prio_head].first_task = task->next;
                }
                return task;
            }
            // If priority of prio_head task is less than or equal than current task
            else {
                return gp_current_task;
            }
        // Case where yielding or changing the priority of the current task
        case YIELD_PRIOCHANGE:
            // If priority of priority head task is higher or equal to current task
            if (prio_head <= gp_current_task->prio) {
                task = prio_array[prio_head].first_task;
                
                if (prio_array[prio_head].first_task == prio_array[prio_head].last_task) {
                    prio_array[prio_head].last_task = NULL;
                    prio_array[prio_head].first_task = NULL;
                    prio_head = prio_array[prio_head].next_prio;
                }
                else {
                    prio_array[prio_head].first_task = task->next;
                }
                return task;
            }
            // If priority of priority head task is less than or equal than current task
            else {
                return gp_current_task;
            }
        case EXIT:
            task = prio_array[prio_head].first_task;
            
            if (task->prio == PRIO_NULL) {
                return task;
            }
            else if (prio_array[prio_head].first_task == prio_array[prio_head].last_task) {
                prio_array[prio_head].last_task = NULL;
                prio_array[prio_head].first_task = NULL;
                prio_head = prio_array[prio_head].next_prio;
            }
            else {
                prio_array[prio_head].first_task = task->next;
            }
            return task;
            
        default:
            return NULL;
    }

}



/**************************************************************************//**
 * @brief       initialize all boot-time tasks in the system,
 *
 *
 * @return      RTX_OK on success; RTX_ERR on failure
 * @param       task_info   boot-time task information structure pointer
 * @param       num_tasks   boot-time number of tasks
 * @pre         memory has been properly initialized
 * @post        none
 *
 * @see         k_tsk_create_new
 *****************************************************************************/


// Improvements:
// - Make all tasks dormant tasks
// - Use the tid number for next and prev instead of TCB pointers
int k_tsk_init(RTX_TASK_INFO *task_info, int num_tasks)
{
    extern U32 SVC_RESTORE;

    // Initialize all prio_array values
    for (size_t i = 0; i < 256; i++) {
        prio_array[i].first_task = NULL;
        prio_array[i].last_task = NULL;
        prio_array[i].next_prio = NULL;
        prio_array[i].prev_prio = NULL;
    }

    RTX_TASK_INFO *p_taskinfo = &g_null_task_info;
    g_num_active_tasks = 0;

    if (num_tasks > MAX_TASKS) {
    	return RTX_ERR;
    }

    // create the first task
    TCB *p_tcb = &g_tcbs[0];
    p_tcb->prio     = PRIO_NULL;
    p_tcb->priv     = 1;
    p_tcb->tid      = TID_NULL;
    p_tcb->state    = RUNNING;
    p_tcb->ptask    = &task_null;
    g_num_active_tasks++;
    gp_current_task = p_tcb;

    // Add null task to priority array
    prio_array[PRIO_NULL].first_task = p_tcb; 
    prio_array[PRIO_NULL].last_task = p_tcb;
    prio_array[PRIO_NULL].next_prio = PRIO_NULL;
    prio_head = PRIO_NULL;

    // create the rest of the tasks
    p_taskinfo = task_info;
    task_t tid;
    int actual_num_tasks = num_tasks;
    for (int i = 0; i < num_tasks; i++ ) { // In the general case, i + 1 is the working TID
        // If kcd_task exists
        if (p_taskinfo->ptask == &kcd_task) {
            // If have less than 160 MAX_TASKS, let KCD tid be the current working TID
            if (MAX_TASKS < 160){
                actual_KCD_tid = i + 1;   
            }
            // If MAX_TASKS is 160 or more 
            else {
                // If not at correct TID for the KCD, skip and let the current working TID be used by other task 
                if (i + 1 != TID_KCD){
                    i--;
                    num_tasks--;
                }
                actual_KCD_tid = TID_KCD;
            }
            tid = actual_KCD_tid; // Create KCD task using actual KCD tid
        }
        else {
            // If reach TID_KCD, skip
            if (i + 1 == TID_KCD) {
                i++;
                num_tasks++;
            }

            // If reach end of g_tcbs array, use the reserved TID for KCD
            if (i + 1 == MAX_TASKS) {
                tid = TID_KCD;
            }
            // Use the working TID as the TID for the task
            else {
                tid = i + 1;
            }
            
        }

        TCB *p_tcb = &g_tcbs[tid];
        if (k_tsk_create_new(p_taskinfo, p_tcb, tid) == RTX_OK) {
            g_num_active_tasks++;

            // Add task to priority array
            k_add_task_scheduler(p_tcb);
        }
    
        p_taskinfo++;
    }
    // Sets tid_count to the first unused tid.
    tid_count = num_tasks + 1;

    // If no KCD task and number of tasks created less than MAX and have more than TID_KCD tasks, make dormant task for TID_KCD
    if (actual_KCD_tid == MAX_TASKS && actual_num_tasks < MAX_TASKS - 1 && tid_count > TID_KCD) {
        TCB *task_tcb = &g_tcbs[TID_KCD];
        task_tcb->tid = TID_KCD;
        task_tcb->state = DORMANT;
        task_tcb->next = dormant_head;
        dormant_head = task_tcb;
    }

    return RTX_OK;
}
/**************************************************************************//**
 * @brief       initialize a new task in the system,
 *              one dummy kernel stack frame, one dummy user stack frame
 *
 * @return      RTX_OK on success; RTX_ERR on failure
 * @param       p_taskinfo  task information structure pointer
 * @param       p_tcb       the tcb the task is assigned to
 * @param       tid         the tid the task is assigned to
 *
 * @details     From bottom of the stack,
 *              we have user initial context (xPSR, PC, SP_USR, uR0-uR12)
 *              then we stack up the kernel initial context (kLR, kR0-kR12)
 *              The PC is the entry point of the user task
 *              The kLR is set to SVC_RESTORE
 *              30 registers in total
 *
 *****************************************************************************/
int k_tsk_create_new(RTX_TASK_INFO *p_taskinfo, TCB *p_tcb, task_t tid)
{
#ifdef DEBUG_0
    printf("k_mem_create_new: creating a task with priv = %d\r\n", p_taskinfo->priv);
#endif /* DEBUG_0 */
    
    extern U32 SVC_RESTORE;

    U32 *sp;

    if (p_taskinfo == NULL || p_tcb == NULL)
    {
        return RTX_ERR;
    }

    p_tcb->tid = tid;
    p_tcb->state = READY;

    /*---------------------------------------------------------------
     *  Step1: allocate kernel stack for the task
     *         stacks grows down, stack base is at the high address
     * -------------------------------------------------------------*/

    ///////sp = g_k_stacks[tid] + (K_STACK_SIZE >> 2) ;
    sp = k_alloc_k_stack(tid);
    p_tcb->k_stack_hi = (U32) sp;
    
    // Unneeded due to current way kernal stack allocated
    // // 8B stack alignment adjustment
    // if ((U32)sp & 0x04) {   // if sp not 8B aligned, then it must be 4B aligned
    //     sp--;               // adjust it to 8B aligned
    // }

    /*-------------------------------------------------------------------
     *  Step2: create task's user/sys mode initial context on the kernel stack.
     *         fabricate the stack so that the stack looks like that
     *         task executed and entered kernel from the SVC handler
     *         hence had the user/sys mode context saved on the kernel stack.
     *         This fabrication allows the task to return
     *         to SVC_Handler before its execution.
     *
     *         16 registers listed in push order
     *         <xPSR, PC, uSP, uR12, uR11, ...., uR0>
     * -------------------------------------------------------------*/

    // if kernel task runs under SVC mode, then no need to create user context stack frame for SVC handler entering
    // since we never enter from SVC handler in this case
    // uSP: initial user stack
    if ( p_taskinfo->priv == 0 ) { // unprivileged task
        // xPSR: Initial Processor State
        *(--sp) = INIT_CPSR_USER;
        // PC contains the entry point of the user/privileged task
        *(--sp) = (U32) (p_taskinfo->ptask);

        //********************************************************************//
        //*** allocate user stack from the user space, not implemented yet ***//
        //********************************************************************//
        
        p_tcb->u_stack_size = p_taskinfo->u_stack_size;
        // printf("Size of user stack alloc in task create: %d\n\r", p_tcb->u_stack_size);
        *(--sp) = (U32) k_alloc_p_stack(tid);

        // Check if failed to allocate enough memory for user stack
        // -- Should equal to u_stack_size if malloc outputed NULL (0)
        if (*(sp) == p_tcb->u_stack_size) {
            return RTX_ERR;
        }
        p_tcb->u_stack_hi = *(sp);

        // uR12, uR11, ..., uR0
        for ( int j = 0; j < 13; j++ ) {
            *(--sp) = 0x0;
        }
    }


    /*---------------------------------------------------------------
     *  Step3: create task kernel initial context on kernel stack
     *
     *         14 registers listed in push order
     *         <kLR, kR0-kR12>
     * -------------------------------------------------------------*/
    if ( p_taskinfo->priv == 0 ) {
        // user thread LR: return to the SVC handler
        *(--sp) = (U32) (&SVC_RESTORE);
    } else {
        // kernel thread LR: return to the entry point of the task
        *(--sp) = (U32) (p_taskinfo->ptask);
    }

    // kernel stack R0 - R12, 13 registers
    for ( int j = 0; j < 13; j++) {
        *(--sp) = 0x0;
    }

    // kernel stack CPSR
    *(--sp) = (U32) INIT_CPSR_SVC;
    p_tcb->ksp = sp;

    // Set remaining tcb items
    p_tcb->ptask = p_taskinfo->ptask;
    p_tcb->prio = p_taskinfo->prio;
    p_tcb->priv = p_taskinfo->priv;
    p_tcb->mailbox = NULL;

    return RTX_OK;
}

/**************************************************************************//**
 * @brief       switching kernel stacks of two TCBs
 * @param:      p_tcb_old, the old tcb that was in RUNNING
 * @return:     RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre:        gp_current_task is pointing to a valid TCB
 *              gp_current_task->state = RUNNING
 *              gp_crrent_task != p_tcb_old
 *              p_tcb_old == NULL or p_tcb_old->state updated
 * @note:       caller must ensure the pre-conditions are met before calling.
 *              the function does not check the pre-condition!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * @attention   CRITICAL SECTION
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *****************************************************************************/
__asm void k_tsk_switch(TCB *p_tcb_old)
{
        PUSH    {R0-R12, LR}
        MRS 	R1, CPSR
        PUSH 	{R1}
        STR     SP, [R0, #TCB_KSP_OFFSET]   ; save SP to p_old_tcb->ksp
        LDR     R1, =__cpp(&gp_current_task);
        LDR     R2, [R1]
        LDR     SP, [R2, #TCB_KSP_OFFSET]   ; restore ksp of the gp_current_task
        POP		{R0}
        MSR		CPSR_cxsf, R0
        POP     {R0-R12, PC}
}


/**************************************************************************//**
 * @brief       run a new thread. The caller becomes READY and
 *              the scheduler picks the next ready to run task.
 * @return      RTX_ERR on error and zero on success
 * @pre         gp_current_task != NULL && gp_current_task == RUNNING
 * @post        gp_current_task gets updated to next to run task
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * @attention   CRITICAL SECTION
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *****************************************************************************/
int k_tsk_run_new(void)
{
    TCB *p_tcb_old = NULL;
    
    if (gp_current_task == NULL) {
    	return RTX_ERR;
    }

    p_tcb_old = gp_current_task;
    gp_current_task = scheduler();
    scheduling_status = CREATE_PRIOCHANGE_UNBLOCK;

    // Scheduler will always give us the NULL task so check unnecessary
    // if ( gp_current_task == NULL  ) {
    //     gp_current_task = p_tcb_old;        // revert back to the old task
    //     return RTX_ERR;
    // }

    // at this point, gp_current_task != NULL and p_tcb_old != NULL
    if (gp_current_task != p_tcb_old) {
        gp_current_task->state = RUNNING;   // change state of the to-be-switched-in  tcb
        p_tcb_old->state = READY;           // change state of the to-be-switched-out tcb
        k_add_task_scheduler(p_tcb_old);    // Add old tcb to priority list
        k_tsk_switch(p_tcb_old);            // switch stacks
    }

    return RTX_OK;
}

/**************************************************************************//**
 * @brief       yield the cpu
 * @return:     RTX_OK upon success
 *              RTX_ERR upon failure
 * @pre:        gp_current_task != NULL &&
 *              gp_current_task->state = RUNNING
 * @post        gp_current_task gets updated to next to run task
 * @note:       caller must ensure the pre-conditions before calling.
 *****************************************************************************/
int k_tsk_yield(void)
{
    // If NULL task, just continue running it
    if (prio_head == PRIO_NULL) {
        return RTX_OK;
    }

    scheduling_status = YIELD_PRIOCHANGE;
    return k_tsk_run_new();
}


/*
 *===========================================================================
 *                             TO BE IMPLEMETED IN LAB2
 *===========================================================================
 */

int k_tsk_create(task_t *task, void (*task_entry)(void), U8 prio, U16 stack_size)
{
#ifdef DEBUG_0
    printf("k_tsk_create: entering...\n\r");
    printf("task = 0x%x, task_entry = 0x%x, prio=%d, stack_size = %d\n\r", task, task_entry, prio, stack_size);
#endif /* DEBUG_0 */
    TCB *task_tcb;

    // Check if task or task_entry are invalid
    if (task == NULL || task_entry == NULL) {
        return RTX_ERR;
    }

    // Check for invalid prio
    if (prio == PRIO_NULL || prio == PRIO_RT) {
        return RTX_ERR;
    }
    // Check if stack size is divisable by 8 or less than min stack size
    if (stack_size < U_STACK_SIZE || stack_size & 7 != 0) { 
        return RTX_ERR;
    }

    // If dormant LL is not empty.
    if (dormant_head != NULL){
        // Moving current head to temp variable and setting head to next.
        task_tcb = dormant_head;
        dormant_head = dormant_head->next;
    }
    // If the dormant LL is empty.
    else {
        // If the tid_count is KCD_TID and the KCD task exists, skip current tid_count
        if (tid_count == actual_KCD_tid && actual_KCD_tid != MAX_TASKS){
            tid_count++;
        }

        // No more uninitialized tcbs. 
        if (tid_count == MAX_TASKS){
            return RTX_ERR;
        }
        // Initializing new tcb with tid and incrementing tid_count to next uninitalized tcb.
        task_tcb = &g_tcbs[tid_count];
        task_tcb->tid = tid_count;
        tid_count++;
    }

    // Setup task_info struct for new task based on parameters.
    RTX_TASK_INFO p_taskinfo;
    p_taskinfo.ptask = task_entry;
    p_taskinfo.u_stack_size = stack_size;
    p_taskinfo.prio = prio;
    p_taskinfo.priv = 0;
    
    if (k_tsk_create_new(&p_taskinfo, task_tcb, task_tcb->tid) == RTX_OK) {
        g_num_active_tasks++;

        // Add task to priority queue
        k_add_task_scheduler(task_tcb);
    }
    else {
        // Make current in-progress task dormant and place on dormant task queue
        task_tcb->state = DORMANT;
        task_tcb->next = dormant_head;
        dormant_head = task_tcb;
        return RTX_ERR;
    }

    // Update scheduler
    scheduling_status = CREATE_PRIOCHANGE_UNBLOCK;
    if (k_tsk_run_new() != RTX_OK) {
        return RTX_ERR;
    }

    *task = task_tcb->tid;
    
    return RTX_OK;

}


// Can make scheduler pop-out current task and place into priority queue/delete
void k_tsk_exit(void) 
{
#ifdef DEBUG_0
    printf("k_tsk_exit: entering...\n\r");
#endif /* DEBUG_0 */
    TCB *p_tcb_old = NULL;
    
    // if (gp_current_task == NULL || gp_current_task->prio == PRIO_NULL) {
    // 	return;
    // }

    p_tcb_old = gp_current_task;
    scheduling_status = EXIT;
    gp_current_task = scheduler();

    // at this point, gp_current_task != NULL and p_tcb_old != NULL
    gp_current_task->state = RUNNING;   // change state of the to-be-switched-in  tcb
    p_tcb_old->state = DORMANT;         // change state of the to-be-switched-out tcb

#ifdef DEBUG_0
    printf("k_tsk_exit: freeing user stack for tid = %d\r\n", p_tcb_old->tid);
#endif /* DEBUG_0 */
    // Change TID to kernel TID
    task_t old_tid = gp_current_task->tid;
    gp_current_task->tid = TID_NULL;
    
    // Deallocate user stack buffer
    k_mem_dealloc((void *)(p_tcb_old->u_stack_hi - p_tcb_old->u_stack_size));
    
    // Deallocate mailbox if present
    if (gp_current_task->mailbox != NULL) {
        k_mem_dealloc(gp_current_task->mailbox);
    }
    
    // Change TID back to current_task TID
    gp_current_task->tid = old_tid;

    // Add old task to dormant list
    p_tcb_old->next = dormant_head;
    dormant_head = p_tcb_old;

    g_num_active_tasks--;

    k_tsk_switch(p_tcb_old);            // switch stacks
}

int k_tsk_set_prio(task_t task_id, U8 prio) 
{
#ifdef DEBUG_0
    printf("k_tsk_set_prio: entering...\n\r");
    printf("task_id = %d, prio = %d.\n\r", task_id, prio);
#endif /* DEBUG_0 */
    
    // Check for invalid priority
    if (prio == PRIO_NULL || prio == PRIO_RT) {
        return RTX_ERR;
    }

    k_translate_tid(&task_id);

    TCB *target_task = &g_tcbs[task_id];

    // Check for invalid task_id
    if (task_id == TID_NULL || task_id >= tid_count || target_task->state == DORMANT) {
        return RTX_ERR;
    }

    U8 current_task_priv = gp_current_task->priv;
    U8 target_task_priv = target_task->priv;

    // If current running task is user task and targeting kernel task
    if (current_task_priv == 0 && target_task_priv == 1) {
        return RTX_ERR;
    }

    // If changing own priority
    if (task_id == gp_current_task->tid) {
        // Update priority
        target_task->prio = prio;

        // Update scheduling status
        scheduling_status = YIELD_PRIOCHANGE;
    }
    // If task is blocked, just change prio and return
    else if (target_task->state == BLK_MSG) {
        target_task->prio = prio;

        return RTX_OK;
    }
    // If changing other task priority
    else {
        U8 target_task_prio = target_task->prio;

        /* Remove task from priority list*/
        // If target_task is first task on priority_array entry
        if (prio_array[target_task_prio].first_task == target_task) {
            // If task is only task at prio_entry on priority_array
            if (prio_array[target_task_prio].last_task == target_task) {
                // If target prio_entry is prio_head
                if (target_task_prio == prio_head) {
                    prio_head = prio_array[prio_head].next_prio;
                }
                // If target prio_entry is in middle of priority_array
                else {
                    prio_array[prio_array[target_task_prio].prev_prio].next_prio = prio_array[target_task_prio].next_prio;
                    prio_array[prio_array[target_task_prio].next_prio].prev_prio = prio_array[target_task_prio].prev_prio;
                }
                prio_array[target_task_prio].last_task = NULL;
                prio_array[target_task_prio].first_task = NULL;   
            }
            // If other tasks exist at target prio_entry
            else {
                prio_array[target_task_prio].first_task = target_task->next;
            }
        }
        // If task is last task on target prio_entry linked list
        else if (prio_array[target_task_prio].last_task == target_task) {
            prio_array[target_task_prio].last_task = target_task->prev;
        }
        // If task is in middle of target prio_entry linked list
        else {
            target_task->next->prev = target_task->prev;
            target_task->prev->next = target_task->next;
        }

        // Update priority
        target_task->prio = prio;

        // Add task back to priority list with new priority
        k_add_task_scheduler(target_task);

        // Update scheduling status
        scheduling_status = CREATE_PRIOCHANGE_UNBLOCK;
    }

    // Schedule new task to run
    if (k_tsk_run_new() != RTX_OK) {
        return RTX_ERR;
    }

    return RTX_OK;    
}

int k_tsk_get_info(task_t task_id, RTX_TASK_INFO *buffer)
{
#ifdef DEBUG_0
    printf("k_tsk_get_info: entering...\n\r");
    printf("task_id = %d, buffer = 0x%x.\n\r", task_id, buffer);
#endif /* DEBUG_0 */    
    if (buffer == NULL) {
        return RTX_ERR;
    }

    k_translate_tid(&task_id);

    TCB *task = &g_tcbs[task_id];

    // Check for invalid task_id
    if (task_id >= tid_count || task->state == DORMANT) {
        return RTX_ERR;
    }

    // Fill buffer
    buffer->tid = task_id;
    buffer->prio = task->prio;
    buffer->state = task->state;
    buffer->priv = task->priv;
    buffer->ptask = task->ptask;
    buffer->k_stack_size = K_STACK_SIZE;
    buffer->u_stack_size = task->u_stack_size;
    buffer->k_stack_hi = task->k_stack_hi;
    buffer->u_stack_hi = task->u_stack_hi;

    return RTX_OK;     
}

task_t k_tsk_get_tid(void)
{
#ifdef DEBUG_0
    printf("k_tsk_get_tid: entering...\n\r");
#endif /* DEBUG_0 */ 
    // When trying to access the TID of the KCD, give the correct TID
    if (gp_current_task->tid == actual_KCD_tid) {
        return TID_KCD;
    }
    return gp_current_task->tid;
}

int k_tsk_ls(task_t *buf, int count){
#ifdef DEBUG_0
    printf("k_tsk_ls: buf=0x%x, count=%d\r\n", buf, count);
#endif /* DEBUG_0 */
    return 0;
}

// Add task to scheduler priority list
// -- Could replace task->prio with stored variable
void k_add_task_scheduler(TCB *task) {
    const U8 task_prio = task->prio;
    prio *prio_entry = &prio_array[task_prio];
    
    // If task NULL task, just add
    // -- Could just remove and let code continue to "task priority lower than prio_head" section
    //    If assuming that last priority entry always points to null priority entry
    // if (task_prio == PRIO_NULL) {
    //     prio_entry->first_task = task;
    //     prio_entry->last_task = task;
    // }
    // If no tasks at current priority level
    if (prio_entry->first_task == NULL) {
        prio_entry->first_task = task;
        prio_entry->last_task = task;

        // If task priority higher than current prio_head
        if (task_prio < prio_head) {
            prio_entry->next_prio = prio_head;
            prio_array[prio_head].prev_prio = task_prio;
            prio_head = task_prio;
        }
        // If task priority lower than prio_head
        else {
            U8 current_prio = prio_head;

            // Loops through prio_entry array list
            while (current_prio < task_prio) { // There will always exist a lower priority due to NULL task
                current_prio = prio_array[current_prio].next_prio;
            }
            
            const U8 previous_prio = prio_array[current_prio].prev_prio;
            // Update previous priority entry
            prio_array[previous_prio].next_prio = task_prio;

            // Update current priority entry
            prio_entry->prev_prio = previous_prio;
            prio_entry->next_prio = current_prio;

            // Update next priority entry
            prio_array[current_prio].prev_prio = task_prio;
        }
    }
    // Tasks present at current priority level
    else {
        prio_entry->last_task->next = task;
        task->prev = prio_entry->last_task; // Update prev task
        prio_entry->last_task = task;
    }
}

// Translates TID to correct TID to take in account KCD_TID
extern inline void k_translate_tid(task_t *tid) {
    if (*tid == TID_KCD && actual_KCD_tid != MAX_TASKS) {
        *tid = actual_KCD_tid;
    }
    else if (*tid == actual_KCD_tid) {
        *tid = MAX_TASKS;
    }
}

// Reverse translate correct TID to formal TID
extern inline void k_reverse_translate_tid(task_t *tid) {
    if (*tid == actual_KCD_tid && actual_KCD_tid != MAX_TASKS) {
        *tid = TID_KCD;
    }
}

/*
 *===========================================================================
 *                             TO BE IMPLEMETED IN LAB4
 *===========================================================================
 */

int k_tsk_create_rt(task_t *tid, TASK_RT *task)
{
    return 0;
}

void k_tsk_done_rt(void) {
#ifdef DEBUG_0
    printf("k_tsk_done: Entering\r\n");
#endif /* DEBUG_0 */
    return;
}

void k_tsk_suspend(TIMEVAL *tv)
{
#ifdef DEBUG_0
    printf("k_tsk_suspend: Entering\r\n");
#endif /* DEBUG_0 */
    return;
}

/*
 *===========================================================================
 *                             END OF FILE
 *===========================================================================
 */
