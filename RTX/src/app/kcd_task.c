/* The KCD Task Template File */

#include "common.h"
#include "Serial.h"
#include "rtx.h"


static inline BOOL is_alphanumeric(char letter) {
    return ('0' <= letter && letter <= '9') || ('A' <= letter && letter <= 'Z') || ('a' <= letter && letter <= 'z');
}

// Convert cmd character to index in command array
static inline U8 cmd_index(char cmd) {
    if ('a' <= cmd) {
        return cmd - 97 + 10 + 26;
    }
    else if ('A' <= cmd) {
        return cmd - 65 + 10;
    }
    else {
        return cmd - 48;
    }
}

void kcd_task(void)
{
    task_t cmd_tids[26 + 26 + 10] = {0};

    if (mbx_create(KCD_MBX_SIZE) == RTX_ERR) {
        tsk_exit();
    }

    U8 buf[1 + sizeof(RTX_MSG_HDR)] = {0};
    U8 full_message[64 + sizeof(RTX_MSG_HDR)];
    U8 message[64] = {0};
    U8 message_index = 0;
    char str_cmd_not_processed[] = "Command cannot be processed\n\r";
    char str_invalid_cmd[] = "Invalid Command\n\r";
    task_t tid;

    while (TRUE) {
        if (recv_msg(&tid, buf, 1 + sizeof(RTX_MSG_HDR)) == RTX_OK) {
            RTX_MSG_HDR *msg_hdr = (RTX_MSG_HDR *)buf;
            char *data = (char *)((U32)msg_hdr + sizeof(RTX_MSG_HDR));

            // If registering a new command
            if (msg_hdr->type == KCD_REG) {
                // Check if only 1 character long and alphanumeric
                if (msg_hdr->length == 1 && is_alphanumeric(data[0])) {
                    cmd_tids[cmd_index(data[0])] = tid;
                }
            }
            // If recieving a key in from the UART IRQ handler
            else if (msg_hdr->type == KEY_IN && tid == TID_UART_IRQ) {
                // If didn't recieve enter key, fill message buffer
                if (data[0] != '\n' || data[0] != '\r') {
                    // Fill message buffer
                    if (message_index < 64) {
                        message[message_index] = data[0];
                        message_index++;
                    }
                    // Exceeded message buffer size
                    else {
                        message_index = 65;
                    }
                }
                // If recieved enter key, check message
                else {
                    // If recieved % character at beginning and didn't exceed message buffer length
                    if (message[0] == '%' && message_index > 0 && message_index < 65) {
                        // If recieved a valid message
                        if (message_index >= 2) {
                            RTX_TASK_INFO task_info;
                            task_t cmd_tid = cmd_tids[cmd_index(message[1])];
                            tsk_get_info(cmd_tid, &task_info);

                            // CMD doesn't exist or task related to CMD doesn't exist
                            if (cmd_tid == TID_NULL || task_info.state == DORMANT) {
                                // COMMAND CAN NOT BE PROCESSED
                                message_index = 0;
                                SER_PutStr(1, str_cmd_not_processed);
                            }
                            else {
                                // Create message to sent to cmd reciever task
                                RTX_MSG_HDR *full_message_hdr = (RTX_MSG_HDR *)full_message;
                                full_message_hdr->type = KCD_CMD;
                                full_message_hdr->length = message_index - 1 + sizeof(RTX_MSG_HDR);

                                for (size_t i = 0; i < message_index - 1; i++) {
                                    full_message[sizeof(RTX_MSG_HDR) + i] = message[i + 1];
                                }

                                if (send_msg(cmd_tid, (void *)full_message) == RTX_ERR) {
                                    // COMMAND CAN NOT BE PROCESSED
                                    message_index = 0;
                                    SER_PutStr(1, str_cmd_not_processed);
                                }
                                else {
                                    // Clear message buffer
                                    message_index = 0;
                                }
                            }
                        }
                        // If recieved only a % character
                        else {
                            // COMMAND CAN NOT BE PROCESSED
                            message_index = 0;
                            SER_PutStr(1, str_cmd_not_processed);
                        }  
                    }
                    // If didn't recieve % character or exceeded message buffer length
                    else {
                        // SEND INVALID COMMAND
                        message_index = 0;
                        SER_PutStr(1, str_invalid_cmd);
                    }
                }
                
            }
        }
    }
}