/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "uart.h"
#include "freertos_task.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
#define CMD_LENGTH 16
char cmd[CMD_LENGTH]= {0};
/* USER CODE BEGIN Variables */
osThreadId defaultTaskHandle;
osThreadId Task1Handle;
osThreadId Task2Handle;
/* USER CODE END Variables */
#define DISPLAY_MENU			'0'
#define SUSPEND_TASK_1		'1'
#define	RESUME_TASK_1			'2'
/* Function prototypes -------------------------------------------------------*/

/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void const * argument);
void Task1Function(void const * argument);
void Task2Function(void const * argument);
void DisplayExampleMenu(void);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* USER CODE BEGIN Application */
void DisplayExampleMenu(void)
{
    printf("	***********************************	\r\n");
		printf("	Type number correspond to execute commands	\r\n");
		printf("	0. Display Menu \r\n");
		printf("	1. Suspend Task 1 \r\n");
		printf("	2. Resume Task 1 \r\n");
    printf("	***********************************	\r\n");
}


void createTasks( void )
{
		DisplayExampleMenu();
	
    osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osThreadDef(Task1, Task1Function, osPriorityAboveNormal, 0, 128);
    Task1Handle = osThreadCreate(osThread(Task1), NULL);

    osThreadDef(Task2, Task2Function, osPriorityAboveNormal, 0, 128);
    Task2Handle = osThreadCreate(osThread(Task2), NULL);
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
		osStatus osstatus;
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
        osDelay(100);

        if( uart_read((uint8_t*)cmd,0xff) == 0)
        {
            printf("Receive %s \r\n",cmd);
            switch(cmd[0])
            {
						case DISPLAY_MENU:
							DisplayExampleMenu();
							break;
            case SUSPEND_TASK_1:
                osstatus = osThreadSuspend(Task1Handle);
								if(osstatus == osOK)
								{
									printf("Suspend Task 1 done \r\n");
								}
                break;
						case RESUME_TASK_1:
								osstatus = osThreadResume(Task1Handle);
								if(osstatus == osOK)
								{
									printf("Resume Task 1 done \r\n");
								}
								break;
            default:
                break;


            }
            memset(cmd,0,CMD_LENGTH);

        }

    }
    /* USER CODE END 5 */
}

/* StartDefaultTask function */
void Task1Function(void const * argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
        printf("Task1 runs \r\n");
        osDelay(1000);
    }
    /* USER CODE END 5 */
}

/* StartDefaultTask function */
void Task2Function(void const * argument)
{

    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
        printf("Task2 runs \r\n");
        osDelay(1000);
    }
    /* USER CODE END 5 */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
