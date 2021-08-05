
#include <gtest/gtest.h>

#include <DummyTestClass.h>

TEST(DummyTestClassTest, DummyTestThatPasses) {

  DummyTestClass obj;

  obj.setValue(5);

  ASSERT_EQ(obj.getValue(), 5);
}

TEST(DummyTestClassTest, DummyTestThatAlsoPasses) {

  DummyTestClass obj;

  obj.setValue(5);

  ASSERT_EQ(obj.getValue(), 5);
}

/*
TEST(DummyTestClassTest, DummyTestThatFails) {

  DummyTestClass obj;

  obj.setValue(69);

  ASSERT_EQ(obj.getValue(), 5);
}
*/

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
