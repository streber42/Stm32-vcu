#define __TEMP_LU_TABLES
#include "../Unity/src/unity.h"
#include "../include/temp_meas.h"
#include "../libopeninv/include/params.h"


void setUp(void) {
}


void tearDown(void)
{
}

static const uint16_t Toyota[] = { TOYOTA_M };


void test_Lookup(void) {
    Param::SetFlt(Param::tmpmmax,FP_FROMINT(20));
    TEST_ASSERT_EQUAL_INT32(FP_FROMINT(20),Param::Get(Param::tmpmmax));
    TEST_ASSERT_EQUAL_FLOAT(200,FP_FROMFLT(6.25f));
    TEST_ASSERT_EQUAL_INT32(200,FP_TOINT(TempMeas::Lookup(1, TempMeas::Sensors::TEMP_TOYOTA)));
    TEST_ASSERT_EQUAL_INT32(-20,FP_TOINT(TempMeas::Lookup(4010, TempMeas::Sensors::TEMP_TOYOTA)));
    // static const uint16_t Toyota[] = { TOYOTA_M };
    TEST_ASSERT_EQUAL_INT32(4009, Toyota[0]);
}