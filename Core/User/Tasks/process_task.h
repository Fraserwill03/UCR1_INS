/*
 * process_task.h
 *
 *  Created on: Nov 2, 2024
 *      Author: Will
 */

#ifndef PROCESS_TASK_H
#define PROCESS_TASK_H

/**
 * Public defines
 * ----------------------------------------- */
#define MAX_RX_BUF 128
#define CRC_LENGTH 4
#define SHORT_HEADER_LENGTH 12


/**
 * Public Function Declarations
 * ----------------------------------------- */
void ProcessLogTask(void* argument);

#endif // PROCESS_TASK_H
