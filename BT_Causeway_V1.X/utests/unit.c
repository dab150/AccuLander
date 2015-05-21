#include "seatest.h"
#include "unit.h"

//
// create a test...
//
void test_example()
{
    char *s = "hello world!";
    assert_string_equal("hello world!", s);
    assert_string_contains("hello", s);
    assert_string_doesnt_contain("goodbye", s);
    assert_string_ends_with("!", s);
    assert_string_starts_with("hell", s);
}

//
// put the test into a fixture...
//
void test_fixture_example( void )
{
    test_fixture_start();
    run_test(test_example);
    test_fixture_end();
}


//
// put the fixture into a suite...
//
void all_tests( void )
{
#ifdef __RUN_UNIT_TESTS__
    
    test_fixture_example();
    __gps_UnitTest_Fixture();

#endif
}

//
// run the suite!
//
void run_all_tests()
{
#ifdef __RUN_UNIT_TESTS__
    run_tests(all_tests);
#endif
}
