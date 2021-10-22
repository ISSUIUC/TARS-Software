#include<Arduino.h>
#include<unity.h>
#include<ch.h>
#include<ChRt.h>

#define COMPILE_TARGET


void run_static_test_cases();
void run_threaded_test_cases();

void setup(){
    delay(2000);


	// Run tests that do not use threading
	run_static_test_cases();

	// Run tests using threads!
    chBegin(run_threaded_test_cases);

}

void loop(){

}
