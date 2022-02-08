#include "../Unity/src/unity.h"
#include "../include/GS450H.h"
#include "../libopeninv/include/params.h" 

GS450HClass inv;

void setUp(void) {
    inv.SetGS450H();
}


void tearDown(void)
{
}

void test_setTorqueTarget(void) {
    inv.setTorqueTarget(0);
    TEST_ASSERT_EQUAL_INT(0,inv.scaledTorqueTarget);
}