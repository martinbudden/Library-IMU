#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

void test_dummy()
{
    TEST_ASSERT_TRUE(0 == 0);
}

void setup()
{
    UNITY_BEGIN();

    RUN_TEST(test_dummy);

    UNITY_END();
}

void loop()
{
}
