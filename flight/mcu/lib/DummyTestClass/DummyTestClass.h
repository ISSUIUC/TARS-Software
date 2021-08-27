/**
 * Stupid simple test class to mess with GoogleTest/GMock
 */

#ifndef __LIB_TESTCLASS_H__
#define __LIB_TESTCLASS_H__

class DummyTestClass {
   public:
    int getValue();
    void setValue(int val);

   private:
    int value;
};

#endif
