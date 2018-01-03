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

