#include<Arduino.h>
#include<unity.h>
#include<ch.h>
#include<ChRt.h>


#define COMPILE_TARGET

#include "FifoBuffer.h"


// Define working area
static THD_WORKING_AREA(producerPerfThreadWorkingArea1, 128);
static THD_WORKING_AREA(consumerPerfThreadWorkingArea1, 128);

/******************************************************************************/
/* PRODUCER-CONSUMER THREAD TEST                                              */

SEMAPHORE_DECL(start_producer_perf_thread, 0);
SEMAPHORE_DECL(done_producer_perf_thread, 0);

SEMAPHORE_DECL(start_comsumer_perf_thread, 0);
SEMAPHORE_DECL(done_consumer_perf_thread, 0);

int thread_perf_data[10];
GenericFifoBuffer thread_perf_fifo(thread_perf_data, 10, sizeof(int));

static THD_FUNCTION(producer_perf_thread, arg) {

    chSemWait(&start_producer_perf_thread);

    int i = 0;

    for(i=0;i<=10000;i++){
        while (!thread_perf_fifo.push(&i)){
        };
    }
    

    chSemSignal(&done_producer_perf_thread);
}


static THD_FUNCTION(consumer_perf_thread, arg) {

    chSemWait(&start_comsumer_perf_thread);

    int i,j;

    for(j=0;j<=10000;j++){

        while(!thread_perf_fifo.pop(&i)){
            //let the other thread get the lock if the buffer is empty
            chThdSleep(1);
        };
        
        TEST_ASSERT_EQUAL(i, j);
    }

    chSemSignal(&done_consumer_perf_thread);
}

void test_producer_performance_perf(){
    chSemSignal(&start_producer_perf_thread);
    chSemSignal(&start_comsumer_perf_thread);
    auto begin = chVTGetSystemTime();

    
    chSemWait(&done_producer_perf_thread);
    chSemWait(&done_consumer_perf_thread);
    auto end = chVTGetSystemTime();
    Serial.printf("PERF: 10000 pushes in %d ms\n", end - begin);
    TEST_ASSERT_LESS_THAN(1500, end - begin);
}






/******************************************************************************/
/* RUN THREADED TEST CASES                                                    */

void run_performance_test_cases(){
    chThdCreateStatic(producerPerfThreadWorkingArea1,
                sizeof(producerPerfThreadWorkingArea1),
                NORMALPRIO,  /* Initial priority.    */
                producer_perf_thread,    /* Thread function.     */
                NULL);       /* Thread parameter.    */
    chThdCreateStatic(consumerPerfThreadWorkingArea1,
                sizeof(consumerPerfThreadWorkingArea1),
                NORMALPRIO,  /* Initial priority.    */
                consumer_perf_thread,    /* Thread function.     */
                NULL);       /* Thread parameter.    */

    //let the two threads start so thread startup time does not impact perf testing
    chThdSleep(100);

    RUN_TEST(test_producer_performance_perf);
}

