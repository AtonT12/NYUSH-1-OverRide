#pragma once

// Minimal host-side test harness. No STL containers, no exceptions, no
// allocation: the test and the firmware share the same language subset, so a
// test that compiles here also compiles for the VEX target.

namespace test {

typedef void (*TestFn)();

void addTest(const char* name, TestFn fn);
int runAll();

bool within(double actual, double expected, double eps);
void reportFailure(const char* file, int line, const char* expression, const char* detail);
void reportNearFailure(const char* file, int line, double actual, double expected,
                       double eps);
void reportIntFailure(const char* file, int line, long long actual, long long expected);

}  // namespace test

struct TestRegistrar {
  TestRegistrar(const char* name, test::TestFn fn) { test::addTest(name, fn); }
};

#define TEST_CASE(fn_name)                                        \
  static void fn_name();                                          \
  static TestRegistrar registrar_##fn_name(#fn_name, &fn_name);   \
  static void fn_name()

#define CHECK_TRUE(expression)                                  \
  do {                                                          \
    if (!(expression)) {                                        \
      test::reportFailure(__FILE__, __LINE__, #expression, ""); \
    }                                                           \
  } while (0)

#define CHECK_FALSE(expression) CHECK_TRUE(!(expression))

#define CHECK_INT(actual, expected)                                          \
  do {                                                                       \
    const long long check_actual_ = static_cast<long long>(actual);          \
    const long long check_expected_ = static_cast<long long>(expected);      \
    if (check_actual_ != check_expected_) {                                  \
      test::reportIntFailure(__FILE__, __LINE__, check_actual_,              \
                             check_expected_);                               \
    }                                                                        \
  } while (0)

#define CHECK_NEAR(actual, expected, eps)                                    \
  do {                                                                       \
    const double check_actual_ = (actual);                                   \
    const double check_expected_ = (expected);                               \
    if (!test::within(check_actual_, check_expected_, (eps))) {              \
      test::reportNearFailure(__FILE__, __LINE__, check_actual_,             \
                              check_expected_, (eps));                       \
    }                                                                        \
  } while (0)
