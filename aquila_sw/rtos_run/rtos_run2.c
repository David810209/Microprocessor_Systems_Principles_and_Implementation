// =============================================================================
//  Program : rtos_run.c
//  Author  : Chun-Jen Tsai
//  Date    : Dec/11/2021
// -----------------------------------------------------------------------------
//  Description:
//  This is a multi-thread program to demo the usage of FreeRTOS and shared
//  resource protection using a mutex.
//
//  This program is designed as one of the homework project for the course:
//  Microprocessor Systems: Principles and Implementation
//  Dept. of CS, NYCU (aka NCTU), Hsinchu, Taiwan.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Nov/14/2023, by Chun-Jen Tsai:
//    Add a random number generating task to the second thread to balance
//    the load.
// =============================================================================

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

void Task_Handler(void *pvParameters);
#define ARRAY_SIZE  8192
#define USE_MUTEX 1

#if USE_MUTEX
xSemaphoreHandle xMutex; // a mutex used to protect shared variable.
#endif

int n;
int arr[ARRAY_SIZE];
int ans[ARRAY_SIZE];
int jobQueue[16];
int front = 0, rear = 0;
int bound[16][2];
int done[16];

//queue function
void enqueue(int value) {
    jobQueue[rear++] = value;
}

int dequeue() {
    return jobQueue[front++];
}

int isQueueEmpty() {
    return front == rear;
}

void generate_array(int n){
    for (int i = 0; i < n; i++)
        arr[i] = rand() % 10000;
}

void vPrintString(char *s)
{
    /* Print a string, the UART device is protected in critical section. */
    taskENTER_CRITICAL();
    printf("%s", s);
    taskEXIT_CRITICAL();
}

void vPrintNumber(int n)
{
    /* Print a string, the UART device is protected in critical section. */
    taskENTER_CRITICAL();
    printf("%d ", n);
    taskEXIT_CRITICAL();
}

// --------------------------------------------------------------------
//  The following function implements non-recursive Quick Sort.

void bubblesort(int l, int r) {
    for (int i = l; i < r; ++i) {
        for (int j = i + 1; j < r; ++j) {
            if (arr[i] > arr[j]) {
                int temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }
    }
}

void quickSortIterative(int* arr, int low, int high) {
    int *stack = (int *)malloc((high - low + 1) * sizeof(int));
    int top = -1;

    // 初始化堆疊，將初始範圍壓入
    stack[++top] = low;
    stack[++top] = high;

    while (top >= 0) {
        // 彈出範圍
        high = stack[top--];
        low = stack[top--];

        // 分區操作
        int pivot = arr[high];
        int i = low - 1;
        for (int j = low; j < high; j++) {
            if (arr[j] < pivot) {
                i++;
                int temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }
        int temp = arr[i + 1];
        arr[i + 1] = arr[high];
        arr[high] = temp;

        int pivotIndex = i + 1;

        // 將左右範圍壓入堆疊
        if (pivotIndex - 1 > low) {
            stack[++top] = low;
            stack[++top] = pivotIndex - 1;
        }
        if (pivotIndex + 1 < high) {
            stack[++top] = pivotIndex + 1;
            stack[++top] = high;
        }
    }
    free(stack);
}

void merge(int begin, int mid, int end) {
    int size = end - begin;
    int *tmp = (int *)malloc(size * sizeof(int));

    if (tmp == NULL) {
        vPrintString("Memory allocation failed in merge function.\n");
        exit(1);
    }
    int l = begin, r = mid, idx = 0;

    while (l < mid && r < end) {
        if (arr[l] < arr[r])
            tmp[idx++] = arr[l++];
        else
            tmp[idx++] = arr[r++];
    }
    while (l < mid)
        tmp[idx++] = arr[l++];
    while (r < end)
        tmp[idx++] = arr[r++];
    for (int i = 0; i < idx; i++)
        arr[begin + i] = tmp[i];
    free(tmp);
}
//  End of the sorting functions.
// --------------------------------------------------------------------

int main(void)
{
#if USE_MUTEX
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) return 1;
#endif

    front = 0;
    rear = 0;
    n = ARRAY_SIZE;
    generate_array(ARRAY_SIZE);
    memcpy(ans, arr, sizeof(arr));
    vPrintString("The Array Size is:");
    vPrintNumber(n);
    vPrintString("\nStarting generating Answer...\n");
    quickSortIterative(ans, 0, n-1);
    vPrintString("Finishing generating Answer...\n");
    for (int i = 15; i >= 0; --i) {
        done[i] = 0;
        if (i > 7) { 
            bound[i][0] = (i - 8) * (n / 8);
            bound[i][1] = (i == 15) ? n : bound[i][0] + n / 8;
        } else { 
            bound[i][0] = bound[i * 2][0];
            bound[i][1] = bound[i * 2 + 1][1];
        }
    }
    for (int i = 8; i <= 15; i++) {
        enqueue(i);
    }
    xTaskCreate(Task_Handler, "worker 1", 2048, NULL, 3, NULL);
    xTaskCreate(Task_Handler, "worker 2", 2048, NULL, 3, NULL);
    // tick = clock();
    vPrintString("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();
    return 0;
}

void Task_Handler(void *pvParameters) {
    int job;

    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY); 
        // taskENTER_CRITICAL();
        if (isQueueEmpty()) {
            if(done[1]){
                xSemaphoreGive(xMutex);
                // taskEXIT_CRITICAL();
                break;
            }
            else{
                xSemaphoreGive(xMutex);
                // taskEXIT_CRITICAL();
                continue;
            }
            // taskEXIT_CRITICAL();
        }
        job = dequeue();
        // printf("%s received job %d\n", pcTaskGetName(NULL), job);
        xSemaphoreGive(xMutex); 
        // taskEXIT_CRITICAL();

        

        if (job > 7) {
            // taskENTER_CRITICAL();
            // printf("%s Performing sorting from index %d to %d.\n",task_name, bound[job][0], bound[job][1]);
            // taskEXIT_CRITICAL();
            bubblesort(bound[job][0], bound[job][1] );
        } else {
            // taskENTER_CRITICAL();
            // printf("%s Performing merging from index %d to %d with mid %d.\n", task_name, bound[job][0], bound[job][1], bound[job * 2][1]);
            // taskEXIT_CRITICAL();
            merge(bound[job][0], bound[job * 2][1], bound[job][1]);
        }

        // taskENTER_CRITICAL();
        // printf("%s Current sorted array: ", task_name);
        // for (int i = 0; i < ARRAY_SIZE; i++) {
        //     printf("%d ", arr[i]);
        // }
        // printf("\n");
        // taskEXIT_CRITICAL();

        xSemaphoreTake(xMutex, portMAX_DELAY); 
        // taskENTER_CRITICAL();
        done[job] = 1;
        if (job == 1) { // 最後一個工作完成
            printf("%s finish the job\n", pcTaskGetName(NULL));
            int flag = 1;
            for(int i = 0; i < n; i++){
                if(arr[i] != ans[i]){
                    flag = 0;
                    break;
                }
            }
            if (flag) {
                printf("The sorting is correct\n");
            } else {
                printf("The sorting is incorrect\n");
            }
            // tick = (clock() - tick)/ticks_per_msec;
            // printf("\nIt took %ld msec to compute the sorting\n\n", tick);
            xSemaphoreGive(xMutex); 
            // taskEXIT_CRITICAL();
            break;
        }
        // printf("%s complete job %d\n", pcTaskGetName(NULL), job);
        if ((job % 2 && done[job - 1]) || (job % 2 == 0 && done[job + 1])) {
            enqueue(job >> 1); 
        }
        xSemaphoreGive(xMutex); 
        // taskEXIT_CRITICAL();
    }

    vTaskDelete(NULL); 
}



void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
       configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
       function that will get called if a call to pvPortMalloc() fails.
       pvPortMalloc() is called internally by the kernel whenever a task, queue,
       timer or semaphore is created.  It is also called by various parts of the
       demo application.  If heap_1.c or heap_2.c are used, then the size of the
       heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
       FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
       to query the size of free heap space that remains (although it does not
       provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
       to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
       task.  It is essential that code added to this hook function never attempts
       to block in any way (for example, call xQueueReceive() with a block time
       specified, or call vTaskDelay()).  If the application makes use of the
       vTaskDelete() API function (as this demo application does) then it is also
       important that vApplicationIdleHook() is permitted to return to its calling
       function, because it is the responsibility of the idle task to clean up
       memory allocated by the kernel to any task that has since been deleted. */
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    /* Run time stack overflow checking is performed if
       configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
       function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    printf("Stack overflow error.\n");
    for (;;);
}

void vApplicationTickHook(void)
{
    /* vApplicationTickHook */
}

void vAssertCalled(void)
{
    taskDISABLE_INTERRUPTS();
    while (1)
    {
        __asm volatile ("NOP");
    }
}

void vExternalISR( uint32_t cause )
{
}

