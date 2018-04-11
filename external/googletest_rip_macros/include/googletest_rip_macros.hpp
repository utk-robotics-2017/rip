#include <gtest/gtest.h>

#define RIP_ASSERT_NO_THROW(statement) \
try \
{ \
    statement; \
} \
catch (std::exception const& e) \
{ \
    printf("Exception Message: \"%s\"\n", e.what()); \
    GTEST_FATAL_FAILURE_("Expected: " #statement " doesn't throw an exception.\n" \
                         "  Actual: it throws."); \
}

#define RIP_ASSERT_THROW(statement, expected_exception) \
{ \
    bool caught = false; \
    try \
    { \
        statement; \
    } \
    catch (expected_exception const& e) \
    { \
        caught = true; \
    } \
    catch (std::exception const& e) \
    { \
        printf("Exception Message: \"%s\"\n", e.what()); \
        GTEST_FATAL_FAILURE_("Expected: " #statement " doesn't throw an exception.\n" \
                             "  Actual: it throws a different type."); \
    } \
    if (!caught) \
    { \
        GTEST_FATAL_FAILURE_("Expected: " #statement " to throw an exception.\n" \
                             "  Actual: it throws nothing."); \
    } \
}
