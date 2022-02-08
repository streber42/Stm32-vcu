#include "../Unity/src/unity.h"
#include "../include/throttle.h"
#include "../include/utils.h"
#include "../libopeninv/include/params.h"

Throttle t;

void setUp(void) {
    Throttle::potmax[0] = 3600;
    Throttle::potmin[0] = 400;
    Throttle::potmax[1] = 1800;
    Throttle::potmin[1] = 200;
    Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL);
}


void tearDown(void)
{
}


void test_CheckAndLimit(void) {
    // potvall doesn't change if it is in the range
    int potval = 1024;
    TEST_ASSERT_TRUE(t.CheckAndLimitRange(&potval,0));
    TEST_ASSERT_EQUAL(1024,potval);
    // potval gets set to potmin-1 if its out of range
    potval=10000;
    TEST_ASSERT_FALSE(t.CheckAndLimitRange(&potval,0));
    TEST_ASSERT_EQUAL(399,potval);
    potval=100;
    TEST_ASSERT_FALSE(t.CheckAndLimitRange(&potval,0));
    TEST_ASSERT_EQUAL(399,potval);
}

void test_CheckDualThrottle(void) {
   // return true and don't change potval if pot2 is sane
   int potval = 1024;
   TEST_ASSERT_TRUE(t.CheckDualThrottle(&potval,512));
   TEST_ASSERT_EQUAL(1024,potval);
   // return false and set to potmin if it isn't
   TEST_ASSERT_FALSE(t.CheckDualThrottle(&potval,5000));
   TEST_ASSERT_EQUAL(400,potval);
   potval = 1024;
   TEST_ASSERT_FALSE(t.CheckDualThrottle(&potval,-1000));
   TEST_ASSERT_EQUAL(400,potval);
}

//s32fp Throttle::CalcThrottle(int potval, int pot2val, bool brkpedal)
void test_CalcThrottle(void) {
    // potnom is 0 with potmin and no brake
    TEST_ASSERT_EQUAL_INT32(0,FP_TOINT(t.CalcThrottle(Throttle::potmin[0],Throttle::potmin[1],false)));
    // potnom is 100 with potmax and no brake
    TEST_ASSERT_EQUAL_INT32(100,FP_TOINT(t.CalcThrottle(Throttle::potmax[0],Throttle::potmin[1],false)));
    // potnom is -1 with brake on
    TEST_ASSERT_EQUAL_INT32(-1,FP_TOINT(t.CalcThrottle(Throttle::potmax[0],Throttle::potmin[1],true)));
    // potnom is 50 with half throttle
    TEST_ASSERT_EQUAL_INT32(50,FP_TOINT(t.CalcThrottle(((Throttle::potmax[0]-Throttle::potmin[0])/2)+Throttle::potmin[0],Throttle::potmin[1],false)));
}

void test_RampThrottle(void) {
    TEST_IGNORE(); /* Like This */
}

void test_utils_change(void){
    TEST_ASSERT_EQUAL_INT(0,utils::change(0,0,3040,0,3500));
    TEST_ASSERT_EQUAL_INT(50,utils::change(50,0,3040,0,3500));
}

void test_utils_GetUserThrottleCommand(void) {
    Param::SetInt(Param::dir,1);
    TEST_ASSERT_EQUAL_INT(FP_FROMINT(20),utils::GetUserThrottleCommand());
}

void test_utils_ProcessThrottle(void) {
    Param::SetInt(Param::dir,1);
    TEST_ASSERT_EQUAL_INT(20000,Param::GetInt(Param::throtramprpm));
    TEST_ASSERT_EQUAL_INT(FP_FROMINT(100),Param::Get(Param::throtramp));
    TEST_ASSERT_EQUAL_INT(FP_FROMINT(100),Param::Get(Param::throtmax));
    TEST_ASSERT_EQUAL_INT(FP_FROMINT(-100),Param::Get(Param::throtmin));
    TEST_ASSERT_EQUAL_INT32(0,t.RampThrottle(FP_FROMINT(20)));
    TEST_ASSERT_EQUAL_INT32(0,t.throttleRamped);
    TEST_ASSERT_EQUAL_INT32(0,t.RampThrottle(FP_FROMINT(20)));
    TEST_ASSERT_EQUAL_INT32(FP_FROMINT(100),utils::ProcessThrottle(100));
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
