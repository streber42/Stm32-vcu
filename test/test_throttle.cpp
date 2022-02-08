#include "../Unity/src/unity.h"
#include "../include/throttle.h"
#include "../include/utils.h"
#include "../libopeninv/include/params.h"

extern void parm_Change(Param::PARAM_NUM paramNum);

void setUp(void) {
    Param::SetInt(Param::potmax,3600);
    Param::SetInt(Param::potmin,400);
    Param::SetInt(Param::pot2max,1800);
    Param::SetInt(Param::pot2min,200);
    Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL);
    Param::SetInt(Param::dir, 1);
    parm_Change(Param::PARAM_LAST);
}


void tearDown(void)
{
}


void test_CheckAndLimit(void) {
    Throttle::potmax[0] = 3600;
    Throttle::potmin[0] = 400;
    Throttle::potmax[1] = 1800;
    Throttle::potmin[1] = 200;
    // potvall doesn't change if it is in the range
    int potval = 1024;
    TEST_ASSERT_TRUE(Throttle::CheckAndLimitRange(&potval,0));
    TEST_ASSERT_EQUAL(1024,potval);
    // potval gets set to potmin-1 if its out of range
    potval=10000;
    TEST_ASSERT_FALSE(Throttle::CheckAndLimitRange(&potval,0));
    TEST_ASSERT_EQUAL(399,potval);
    potval=100;
    TEST_ASSERT_FALSE(Throttle::CheckAndLimitRange(&potval,0));
    TEST_ASSERT_EQUAL(399,potval);
}

void test_CheckAndLimitHigh(void) {
    int potval=3801;
    Throttle::potmin[0] = 400;
    TEST_ASSERT_FALSE(Throttle::CheckAndLimitRange(&potval,0));
    TEST_ASSERT_EQUAL(399,potval);
}


void test_CheckDualThrottle(void) {
   // return true and don't change potval if pot2 is sane
   int potval = 1024;
   TEST_ASSERT_TRUE(Throttle::CheckDualThrottle(&potval,512));
   TEST_ASSERT_EQUAL(1024,potval);
   // return false and set to potmin if it isn't
   TEST_ASSERT_FALSE(Throttle::CheckDualThrottle(&potval,5000));
   TEST_ASSERT_EQUAL(400,potval);
   potval = 1024;
   TEST_ASSERT_FALSE(Throttle::CheckDualThrottle(&potval,-1000));
   TEST_ASSERT_EQUAL(400,potval);
}

//s32fp Throttle::CalcThrottle(int potval, int pot2val, bool brkpedal)
void test_CalcThrottle(void) {
    // potnom is 0 with potmin and no brake
    TEST_ASSERT_EQUAL_FLOAT(-0.1f,Throttle::CalcThrottle(Throttle::potmin[0],Throttle::potmin[1],false));
    // potnom is 100 with potmax and no brake
    TEST_ASSERT_EQUAL_FLOAT(100.0f,Throttle::CalcThrottle(Throttle::potmax[0],Throttle::potmin[1],false));
    // potnom is -1 with brake on
    TEST_ASSERT_EQUAL_FLOAT(-0.1f,Throttle::CalcThrottle(Throttle::potmax[0],Throttle::potmin[1],true));
    // potnom is 50 with half throttle
    TEST_ASSERT_EQUAL_FLOAT(35.0f,Throttle::CalcThrottle(((Throttle::potmax[0]-Throttle::potmin[0])/2)+Throttle::potmin[0],Throttle::potmin[1],false));
}

void test_RampThrottle(void) {
    TEST_IGNORE(); /* Like This */
}

void test_utils_change(void){
    TEST_ASSERT_EQUAL_INT(0,utils::change(0,0,3040,0,3500));
    TEST_ASSERT_EQUAL_INT(57,utils::change(50,0,3040,0,3500));
    TEST_ASSERT_EQUAL_INT32(736,utils::change(FP_FROMINT(20),0,3040,0,3500));
}

void test_utils_GetUserThrottleCommand(void) {
    Param::SetInt(Param::dir,1);
    TEST_ASSERT_EQUAL_FLOAT(36.95f,utils::GetUserThrottleCommand());
}

void test_utils_ProcessThrottle(void) {
    TEST_ASSERT_EQUAL_INT(20000,Param::GetInt(Param::throtramprpm));
    TEST_ASSERT_EQUAL_FLOAT(100.0f,Param::GetFloat(Param::throtramp));
    TEST_ASSERT_EQUAL_FLOAT(100.0f,Throttle::throttleRamp);
    TEST_ASSERT_EQUAL_FLOAT(100.0f,Param::GetFloat(Param::throtmax));
    TEST_ASSERT_EQUAL_FLOAT(100.0f,Throttle::throtmax);
    TEST_ASSERT_EQUAL_FLOAT(-100.0f,Param::GetFloat(Param::throtmin));
    TEST_ASSERT_EQUAL_FLOAT(-100.0f,Throttle::throtmin);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,Throttle::throttleRamped);
    // TEST_ASSERT_EQUAL_INT(0,RAMPUP(0,640,3200));
    // TEST_ASSERT_EQUAL_INT32(0,FP_FROMINT(-100));
    Throttle::throttleRamped = FP_FROMINT(0);
    TEST_ASSERT_EQUAL_FLOAT(50.0f,Throttle::RampThrottle(50.0f));
    TEST_ASSERT_EQUAL_FLOAT(50.0f,Throttle::throttleRamped);
    TEST_ASSERT_EQUAL_FLOAT(20.0f,Throttle::RampThrottle(20.0f));
    // Param::SetFlt(Param::idcmax,FP_FROMINT(10));
    Param::SetFloat(Param::idc,1.0f);
    Param::SetFloat(Param::udc,500.0f);
    parm_Change(Param::PARAM_LAST);
    Throttle::idcmax = FP_FROMINT(10);
    // TEST_ASSERT_EQUAL()
    // Throttle::udcmin = 
    TEST_ASSERT_EQUAL(1,Param::GetFloat(Param::idc));
    TEST_ASSERT_EQUAL_FLOAT(36.95f,utils::ProcessThrottle(100));
    TEST_ASSERT_EQUAL_INT(20,FP_TOINT(FP_FROMINT(20)));
}

void test_IgnoredTest(void)
{
    TEST_IGNORE_MESSAGE("This Test Was Ignored On Purpose");
}

void test_AnotherIgnoredTest(void)
{
    TEST_IGNORE_MESSAGE("These Can Be Useful For Leaving Yourself Notes On What You Need To Do Yet");
}

void test_ThisFunctionHasNotBeenTested_NeedsToBeImplemented(void)
{
    TEST_IGNORE(); /* Like This */
}
