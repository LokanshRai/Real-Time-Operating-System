/**
 * @file:   k_msg.c
 * @brief:  kernel message passing routines
 * @author: Yiqing Huang
 * @date:   2020/10/09
 */

#include "k_msg.h"

#ifdef DEBUG_0
#include "printf.h"
#endif /* ! DEBUG_0 */

typedef struct Mailbox_Header {
    U32     *front;
    U32     *tail;
    size_t  buffer_size;
} mailbox_hdr;

typedef struct Message_Header {
    task_t      tid;
    RTX_MSG_HDR base_hdr;
} message_hdr;

enum scheduling_states{CREATE_PRIOCHANGE_UNBLOCK, YIELD_PRIOCHANGE, EXIT};

int k_mbx_create(size_t size) {
#ifdef DEBUG_0
    printf("k_mbx_create: size = %d\r\n", size);
#endif /* DEBUG_0 */

    // Check if mailbox already present or requested size too small
    if (gp_current_task->mailbox != NULL || size < MIN_MBX_SIZE) {
        return RTX_ERR;
    }

    // Change tid to kernel tid an alloc memory
    task_t old_tid = gp_current_task->tid;
    gp_current_task->tid = TID_NULL;
    mailbox_hdr *mailbox = k_mem_alloc(size + sizeof(mailbox_hdr));
    gp_current_task->tid = old_tid;

    // Check if not enough malloced memory available
    if (mailbox == NULL) {
        return RTX_ERR;
    }
    
    // Fill in mailbox header
    mailbox->buffer_size = size;
    mailbox->front = (U32 *)((U32)mailbox + sizeof(mailbox_hdr));
    mailbox->tail = mailbox->front;

    gp_current_task->mailbox = (U32 *)mailbox;

    return RTX_OK;
}

int k_send_msg(task_t receiver_tid, const void *buf) {
#ifdef DEBUG_0
    printf("k_send_msg: receiver_tid = %d, buf=0x%x\r\n", receiver_tid, buf);
#endif /* DEBUG_0 */
    RTX_MSG_HDR *og_buff_head = (RTX_MSG_HDR *)buf;
    k_translate_tid(&receiver_tid);
    TCB *reciever_tcb = &g_tcbs[receiver_tid];

    // Check for valid TID
    if (receiver_tid >= tid_count || reciever_tcb->state == DORMANT) {
        return RTX_ERR;
    }

    // Check if reciever doesn't have a mailbox or buf does not exist
    if (reciever_tcb->mailbox == NULL || buf == NULL) {
        return RTX_ERR;
    }

    // Check if buf length appropriate size
    if (og_buff_head->length < MIN_MSG_SIZE) {
        return RTX_ERR;
    }
    
    mailbox_hdr *reciever_mailbox_hdr = (mailbox_hdr *)(reciever_tcb->mailbox);
    U32 size_needed = og_buff_head->length + sizeof(task_t);
    U32 size_free = 0;
    U8 *message_data = (U8 *)buf;
    U8 *mailbox_message_data;
    message_hdr *revised_message_hdr;

    // If front of queue is behind the tail
    if ((U32)(reciever_mailbox_hdr->front) < (U32)(reciever_mailbox_hdr->tail)) {
        size_free = reciever_mailbox_hdr->buffer_size - ((U32)(reciever_mailbox_hdr->tail) + 1 - (U32)(reciever_mailbox_hdr->front));

        // Check if enough space in mailbox
        if (size_free < size_needed) {
            return RTX_ERR;
        }

        U32 *reciever_mailbox_end_addr = (U32 *)((U32)reciever_mailbox_hdr + sizeof(mailbox_hdr) + reciever_mailbox_hdr->buffer_size);

        // If don't need to loop around queue
        if ((U32)reciever_mailbox_end_addr >= (U32)(reciever_mailbox_hdr->tail) + 1 + size_needed) { // Maybe need equivalency?
            // Add message to reciever mailbox
            revised_message_hdr = (message_hdr *)((U32)(reciever_mailbox_hdr->tail) + 1);
            revised_message_hdr->tid = gp_current_task->tid;
            
            // Copy data to mailbox
            mailbox_message_data = (U8 *)(&revised_message_hdr->base_hdr);
            for (size_t i = 0; i < og_buff_head->length; i++) {
                mailbox_message_data[i] = message_data[i];
            }

            // Update tail
            reciever_mailbox_hdr->tail = (U32 *)((U32)mailbox_message_data + og_buff_head->length - 1);
        }
        // If need to loop around queue
        else {
            mailbox_message_data = (U8 *)((U32)(reciever_mailbox_hdr->tail) + 1);
            U8 *tid_data = (U8 *)(&(gp_current_task->tid)); 

            // Copy data at end of mailbox
            U32 i;
            for (i = 0; i < (U32)(reciever_mailbox_end_addr) - ((U32)(reciever_mailbox_hdr->tail) + 1); i++){
                if (i < sizeof(task_t)) {
                    mailbox_message_data[i] = tid_data[i];
                }
                else {
                    mailbox_message_data[i] = message_data[i - sizeof(task_t)];
                }
            }

            // Loop, and copy data to start of mailbox
            U8* mailbox_start_addr = (U8 *)((U32)reciever_mailbox_hdr + sizeof(mailbox_hdr));
            U32 split_index;
            for (split_index = i; i < size_needed; i++){
                if (i < sizeof(task_t)) {
                    mailbox_start_addr[i - split_index] = tid_data[i];
                }
                else {
                    mailbox_start_addr[i - split_index] = message_data[i - sizeof(task_t)];
                }
            }

            // reciever_mailbox_hdr->tail = (U32 *)((U32)mailbox_start_addr + (size_needed - split_index) - 1);
            reciever_mailbox_hdr->tail = (U32 *)(&mailbox_start_addr[i - split_index - 1]);
        }

    }
    // If tail of queue is behind front
    else if ((U32)(reciever_mailbox_hdr->front) > (U32)(reciever_mailbox_hdr->tail)) {
        size_free = (U32)(reciever_mailbox_hdr->front) - ((U32)(reciever_mailbox_hdr->tail) + 1);

        // Check if enough space in mailbox
        if (size_free < size_needed) {
            return RTX_ERR;
        }

        // Add message to reciever mailbox
        revised_message_hdr = (message_hdr *)((U32)(reciever_mailbox_hdr->tail) + 1);
        revised_message_hdr->tid = gp_current_task->tid;
        
        mailbox_message_data = (U8 *)(&revised_message_hdr->base_hdr);
        for (size_t i = 0; i < og_buff_head->length; i++) {
            mailbox_message_data[i] = message_data[i];
        }
        reciever_mailbox_hdr->tail = (U32 *)((U32)mailbox_message_data + og_buff_head->length - 1);
    }
    // If tail and front are equal and thus the mailbox is empty
    else {
        size_free = reciever_mailbox_hdr->buffer_size;
        
        // Check if enough space in mailbox
        if (size_free < size_needed) {
            return RTX_ERR;
        }

        // Add message to reciever mailbox
        revised_message_hdr = (message_hdr *)(reciever_mailbox_hdr->tail);
        revised_message_hdr->tid = gp_current_task->tid;
        
        mailbox_message_data = (U8 *)(&revised_message_hdr->base_hdr);
        for (size_t i = 0; i < og_buff_head->length; i++) {
            mailbox_message_data[i] = message_data[i];
        }
        reciever_mailbox_hdr->tail = (U32 *)((U32)mailbox_message_data + og_buff_head->length - 1);
    }

    // If reciever state is BLK_MSG
    if (reciever_tcb->state == BLK_MSG) {
        // Update state to ready and add to scheduler
        reciever_tcb->state = READY;
        k_add_task_scheduler(reciever_tcb);
        
        // Schedule reciever task
        scheduling_status = CREATE_PRIOCHANGE_UNBLOCK;
        return k_tsk_run_new();
    }

    return RTX_OK;
}

int k_recv_msg(task_t *sender_tid, void *buf, size_t len) {
#ifdef DEBUG_0
    printf("k_recv_msg: sender_tid  = 0x%x, buf=0x%x, len=%d\r\n", sender_tid, buf, len);
#endif /* DEBUG_0 */

    // Check if mailbox or buf don't exist
    if (gp_current_task->mailbox == NULL) {
        return RTX_ERR;
    }

    mailbox_hdr *mailbox = (mailbox_hdr *)(gp_current_task->mailbox);

    // If no messages in mailbox, block
    if (mailbox->front == mailbox->tail) {
        TCB *p_tcb_old = gp_current_task;
        gp_current_task = scheduler();      // Get new task
        gp_current_task->state = RUNNING;   // change state of the to-be-switched-in  tcb
        p_tcb_old->state = BLK_MSG;         // Change state to blocked
        k_tsk_switch(p_tcb_old);            // switch stacks
    }

    U8 *mailbox_start_addr = (U8 *)((U32)mailbox + sizeof(mailbox_hdr));
    U32 *mailbox_end_addr = (U32 *)((U32)mailbox + sizeof(mailbox_hdr) + mailbox->buffer_size);
    U8 *buf_data = (U8 *)buf;

    U8 *data_addr;
    U32 i;
    U32 split_index;
    
    // If mailbox doesn't loop for mailbox_hdr
    if ((U32)mailbox_end_addr - (U32)(mailbox->front) >= sizeof(mailbox_hdr)) {
        message_hdr *recv_message_hdr = (message_hdr *)(mailbox->front);
        U8 *mailbox_data_addr = (U8 *)(recv_message_hdr);

        // If data doesn't loop
        if ((U32)mailbox_end_addr >= (U32)(mailbox->front) + recv_message_hdr->base_hdr.length + sizeof(task_t)) {
            // If don't have enough space in buffer or have no buffer, pop message and update mailbox front
            if (recv_message_hdr->base_hdr.length > len || buf == NULL) {
                mailbox->front = (U32 *)((U32)recv_message_hdr + recv_message_hdr->base_hdr.length + sizeof(task_t));
                
                // Check if mailbox will be empty
                if ((U32)(mailbox->front) == (U32)(mailbox->tail) + 1) {
                    mailbox->front = mailbox->tail;
                }
                // Loop front if reached end of mailbox
                else if ((U32)(mailbox->front) == (U32)mailbox_end_addr) {
                    mailbox->front = (U32 *)mailbox_start_addr;
                }
                
                return RTX_ERR;
            }

            // Copy message data to buffer
            for (i = 0; i < recv_message_hdr->base_hdr.length; i++){
                buf_data[i] = mailbox_data_addr[i + sizeof(task_t)];
            }

            // Update front
            mailbox->front = (U32 *)((U32)recv_message_hdr + recv_message_hdr->base_hdr.length + sizeof(task_t));
                
            // Check if mailbox will be empty
            if ((U32)(mailbox->front) == (U32)(mailbox->tail) + 1) {
                mailbox->front = mailbox->tail;
            }
            // Loop front if reached end of mailbox
            else if ((U32)(mailbox->front) == (U32)mailbox_end_addr) {
                mailbox->front = (U32 *)mailbox_start_addr;
            }

        }
        // If data loops around mailbox
        else {
            // If don't have enough space in buffer or have no buffer, pop message and update mailbox front
            if (recv_message_hdr->base_hdr.length > len || buf == NULL) {
                mailbox->front = (U32 *)((U32)mailbox_start_addr + (recv_message_hdr->base_hdr.length + sizeof(task_t) - ((U32)mailbox_end_addr - (U32)(mailbox->front))));
                
                // Check if mailbox will be empty
                if ((U32)(mailbox->front) == (U32)(mailbox->tail) + 1) {
                    mailbox->front = mailbox->tail;
                }
                
                return RTX_ERR;
            }

            // Copy message data to reciever buffer
            for (i = 0; i < (U32)mailbox_end_addr - (U32)(mailbox->front); i++) {
                buf_data[i] = mailbox_data_addr[i + sizeof(task_t)];
            }

            // Loop, copy message data to reciever buffer
            for (split_index = i; i < recv_message_hdr->base_hdr.length; i++){
                buf_data[i] = mailbox_start_addr[i - split_index];
            }

            mailbox->front = (U32 *)((U32)mailbox_start_addr + (recv_message_hdr->base_hdr.length + sizeof(task_t) - ((U32)mailbox_end_addr - (U32)(mailbox->front))));
                
            // Check if mailbox will be empty
            if ((U32)(mailbox->front) == (U32)(mailbox->tail) + 1) {
                mailbox->front = mailbox->tail;
            }

        }

        // Update sender_tid
        if (sender_tid != NULL) {
            k_reverse_translate_tid(&recv_message_hdr->tid);
            *sender_tid = recv_message_hdr->tid;
        }
    
    }
    // If not enough space for header at end of mailbox, loop
    else {
        message_hdr temp_message_hdr;

        U8 *temp_message_hdr_data_addr =(U8 *)(&temp_message_hdr); // Maybe issues
        U8 *mailbox_data_addr = (U8 *)(mailbox->front);
        
        // Copy message header from bottom of mailbox
        for (i = 0; i < (U32)(mailbox_end_addr) - (U32)(mailbox->front); i++){
            temp_message_hdr_data_addr[i] = mailbox_data_addr[i];
        }
        
        // Copy message header from start of mailbox
        for (split_index = i; i < sizeof(mailbox_hdr); i++){
            temp_message_hdr_data_addr[i] = mailbox_start_addr[i - split_index];
        }

        data_addr = &mailbox_start_addr[i];

        // If don't have enough space in buffer or have no buffer, pop message and update mailbox front
        if (temp_message_hdr.base_hdr.length > len || buf == NULL) {
            mailbox->front = (U32 *)((U32)data_addr + (temp_message_hdr.base_hdr.length - sizeof(RTX_MSG_HDR)));
            
            // If reach end of mailbox, update front and last to reflect that
            if (mailbox->front > mailbox->tail){
                mailbox->front = mailbox->tail;
            }
            return RTX_ERR;
        }

        // Update sender_tid
        if (sender_tid != NULL) {
            k_reverse_translate_tid(&temp_message_hdr.tid);
            *sender_tid = temp_message_hdr.tid;
        }

        // Copy message header to reciever buffer
        for (i = 0; i < sizeof(RTX_MSG_HDR); i++) {
            buf_data[i] = temp_message_hdr_data_addr[i + sizeof(task_t)];
        }

        // Copy message data to reciever buffer
        for (split_index = i; i < temp_message_hdr.base_hdr.length; i++){
            buf_data[i] = data_addr[i - split_index];
        }

        // Update front
        mailbox->front = (U32 *)((U32)data_addr + (temp_message_hdr.base_hdr.length - sizeof(RTX_MSG_HDR)));
            
        // If reach end of mailbox, update front and last to reflect that
        if (mailbox->front > mailbox->tail){
            mailbox->front = mailbox->tail;
        }
    }

    return RTX_OK;
}

int k_recv_msg_nb(task_t *sender_tid, void *buf, size_t len) {
#ifdef DEBUG_0
    printf("k_recv_msg_nb: sender_tid  = 0x%x, buf=0x%x, len=%d\r\n", sender_tid, buf, len);
#endif /* DEBUG_0 */
    return 0;
}

int k_mbx_ls(task_t *buf, int count) {
#ifdef DEBUG_0
    printf("k_mbx_ls: buf=0x%x, count=%d\r\n", buf, count);
#endif /* DEBUG_0 */
    return 0;
}
