#include<Arduino.h>
#include<unity.h>
#include<ch.h>
#include<ChRt.h>


#define COMPILE_TARGET

#include "FifoBuffer.h"


// Define working area
static THD_WORKING_AREA(myThreadWorkingArea, 128);

static THD_WORKING_AREA(producerThreadWorkingArea1, 128);
static THD_WORKING_AREA(consumerThreadWorkingArea1, 128);

static THD_WORKING_AREA(popThreadWorkingArea1, 128);
static THD_WORKING_AREA(pushThreadWorkingArea1, 128);

static THD_WORKING_AREA(popThreadWorkingArea2, 128);
static THD_WORKING_AREA(pushThreadWorkingArea2, 128);

/******************************************************************************/
/* SIMPLE PUSHING IN THREAD TEST                                              */

SEMAPHORE_DECL(start_thread, 0);
SEMAPHORE_DECL(thread_done, 0);
int thread_data[10];
GenericFifoBuffer thread_fifo(thread_data, 10, sizeof(int));

static THD_FUNCTION(myThread, arg) {
    chSemWait(&start_thread);

    int i = 5;
    thread_fifo.push(&i);

    chSemSignal(&thread_done);
}

void test_thread_push(){
    chSemSignal(&start_thread);
    chSemWait(&thread_done);


    int i;
    TEST_ASSERT(thread_fifo.pop(&i));
    TEST_ASSERT_EQUAL(i, 5);
}


/******************************************************************************/
/* PRODUCER-CONSUMER THREAD TEST                                              */

SEMAPHORE_DECL(start_producerThread1, 0);
SEMAPHORE_DECL(done_producerThread1, 0);

SEMAPHORE_DECL(start_consumerThread1, 0);
SEMAPHORE_DECL(done_consumerThread1, 0);

int thread_data2[10];
GenericFifoBuffer thread_fifo2(thread_data2, 10, sizeof(int));

static THD_FUNCTION(producerThread1, arg) {

    chSemWait(&start_producerThread1);

    int i = 0;

    for(i=0;i<=100;i++){
        while (!thread_fifo2.push(&i)){
            //let the other thread get the lock if the buffer is full
            chThdSleep(1);
        }
    }
    

    chSemSignal(&done_producerThread1);
}

static THD_FUNCTION(consumerThread1, arg) {
    chSemWait(&start_consumerThread1);

    int i,j;

    for(j=0;j<=100;j++){
        while(!thread_fifo2.pop(&i)){
            //let the other thread get the lock if the buffer is empty
            chThdSleep(1);
        }
        
        TEST_ASSERT_EQUAL(i, j);
    }

    chSemSignal(&done_consumerThread1);
}

void test_producer_consumer(){
    chSemSignal(&start_consumerThread1);
    chSemSignal(&start_producerThread1);

    chSemWait(&done_producerThread1);
    chSemWait(&done_consumerThread1);
}

/******************************************************************************/
/* THREAD POP FAILING TEST                                              */

SEMAPHORE_DECL(start_popThread1, 0);
SEMAPHORE_DECL(done_popThread1, 0);

SEMAPHORE_DECL(start_pushThread1, 0);
SEMAPHORE_DECL(done_pushThread1, 0);

int thread_data3[10];
GenericFifoBuffer thread_fifo3(thread_data3, 10, sizeof(int));

static THD_FUNCTION(pushThread1, arg) {

    chSemWait(&start_pushThread1);

    int i = 0;

    while (!thread_fifo3.push(&i));

    chSemSignal(&done_pushThread1);
}

static THD_FUNCTION(popThread1, arg) {

    chSemWait(&start_popThread1);

    int i;

    // Push the first time and it should work.
    TEST_ASSERT_TRUE(thread_fifo3.pop(&i));
        
    TEST_ASSERT_EQUAL(i, 0);

    // Push the second time and it should fail
    TEST_ASSERT_FALSE(thread_fifo3.pop(&i))

    chSemSignal(&done_popThread1);
}

void test_thread_pop_fail(){
    chSemSignal(&start_pushThread1);
    chSemWait(&done_pushThread1);

    chSemSignal(&start_popThread1);
    chSemWait(&done_popThread1);

}

/******************************************************************************/
/* THREAD PUSH FAILING TEST                                              */

SEMAPHORE_DECL(start_pushThread2,0);
SEMAPHORE_DECL(done_pushThread2,0);

SEMAPHORE_DECL(start_pushThread3,0);
SEMAPHORE_DECL(done_pushThread3,0);

int thread_data4[1];
GenericFifoBuffer thread_fifo4(thread_data4, 1, sizeof(int));

static THD_FUNCTION(pushThread2, arg){
    chSemWait(start_pushThread2);

    int i = 1;
    TEST_ASSERT_TRUE(thread_fifo4.push(&i));

    chSemSignal(done_pushThread2);
}

static THD_FUNCTION(popThread2, arg){
    chSemWait(start_);

}



/******************************************************************************/
/* RUN THREADED TEST CASES                                                    */

void run_threaded_test_cases(){


    chThdCreateStatic(myThreadWorkingArea,
                sizeof(myThreadWorkingArea),
                NORMALPRIO,  /* Initial priority.    */
                myThread,    /* Thread function.     */
                NULL);       /* Thread parameter.    */


    chThdCreateStatic(producerThreadWorkingArea1,
                sizeof(producerThreadWorkingArea1),
                NORMALPRIO,  /* Initial priority.    */
                producerThread1,    /* Thread function.     */
                NULL);       /* Thread parameter.    */
    chThdCreateStatic(consumerThreadWorkingArea1,
                sizeof(consumerThreadWorkingArea1),
                NORMALPRIO,  /* Initial priority.    */
                consumerThread1,    /* Thread function.     */
                NULL);       /* Thread parameter.    */

    chThdCreateStatic(popThreadWorkingArea1,
                sizeof(popThreadWorkingArea1),
                NORMALPRIO,  /* Initial priority.    */
                popThread1,    /* Thread function.     */
                NULL);       /* Thread parameter.    */
                
    chThdCreateStatic(pushThreadWorkingArea1,
                sizeof(pushThreadWorkingArea1),
                NORMALPRIO,  /* Initial priority.    */
                pushThread1,    /* Thread function.     */
                NULL);       /* Thread parameter.    */


    // New tests
    RUN_TEST(test_thread_push);
    RUN_TEST(test_producer_consumer);
    RUN_TEST(test_thread_pop_fail);
}

