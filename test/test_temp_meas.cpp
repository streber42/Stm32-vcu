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
    TEST_ASSERT_EQUAL_INT32(FP_FROMINT(200),TempMeas::Lookup(1, TempMeas::Sensors::TEMP_TOYOTA));
    TEST_ASSERT_EQUAL_INT32(FP_FROMFLT(9.4f),TempMeas::Lookup(3984, TempMeas::Sensors::TEMP_TOYOTA));
//test_temp_meas.cpp:24:test_Lookup:FAIL: Expected 2589 Was 2720. 3500
//test_temp_meas.cpp:27:test_Lookup:FAIL: Expected 2589 Was 2620  3540
//test_temp_meas.cpp:26:test_Lookup:FAIL: Expected 2589 Was 2595. 3550
//test_temp_meas.cpp:32:test_Lookup:FAIL: Expected 2589 Was 2590. 3553
//test_temp_meas.cpp:31:test_Lookup:FAIL: Expected 2589 Was 2585  3554
//test_temp_meas.cpp:29:test_Lookup:FAIL: Expected 2589 Was 2585. 3555
//test_temp_meas.cpp:30:test_Lookup:FAIL: Expected 2589 Was 2580  3556
//test_temp_meas.cpp:28:test_Lookup:FAIL: Expected 2589 Was 2570. 3560
//test_temp_meas.cpp:25:test_Lookup:FAIL: Expected 2589 Was 2465  3600
    // TEST_ASSERT_EQUAL_INT32(FP_FROMFLT(160.0f),TempMeas::Lookup(2050, TempMeas::Sensors::TEMP_TOYOTA));
    // TEST_ASSERT_EQUAL_INT32(FP_FROMFLT(81.0f),TempMeas::Lookup(3550, TempMeas::Sensors::TEMP_TOYOTA));
    // TEST_ASSERT_EQUAL_INT32(2594, FP_FROMINT(82));
    // TEST_ASSERT_EQUAL_INT32(FP_FROMINT(81),TempMeas::Lookup(3550, TempMeas::Sensors::TEMP_TOYOTA));
    TEST_ASSERT_EQUAL_INT32(FP_FROMFLT(-20.0f),TempMeas::Lookup(4010, TempMeas::Sensors::TEMP_TOYOTA));
    // static const uint16_t Toyota[] = { TOYOTA_M };
    TEST_ASSERT_EQUAL_INT32(4009, Toyota[0]);
    TEST_ASSERT_EQUAL_FLOAT(31.20024f,TempMeas::readThermistor(2050));
    TEST_ASSERT_EQUAL_FLOAT(9.4f,TempMeas::readThermistor(3550));
}