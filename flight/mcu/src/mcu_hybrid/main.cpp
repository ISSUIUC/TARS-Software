/* main.cpp
 *   ______  ___     ___    ____
 *  /_  __/ / _ |   / _ \  / __/
 *   / /   / __ |  / , _/ _\ \
 *  /_/   /_/ |_| /_/|_| /___/
 *
 * Hybrid Engine Control Program
 *
 * Illinois Space Society - Avioinics Team
 *
 * Anshuk Chigullapalli
 * Josh Blustein
 * Ayberk Yaraneri
 * David Robbins
 * Matt Taylor
 * James Bayus
 * Ben Olaivar
 * TODO: add missing names if any
 */

#include <Arduino.h>
#include <ChRt.h>

//------------------------------------------------------------------------------
// chSetup - let there be threads
void chSetup() {}

void setup() {
    chBegin(chSetup);

    while (true) {
    }
}

void loop() {}
