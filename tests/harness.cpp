#include "harness.h"

#include <cmath>
#include <cstdio>

namespace {

const int kMaxTests = 256;

struct Entry {
  const char* name;
  test::TestFn fn;
};

Entry gTests[kMaxTests];
int gCount = 0;
int gFailures = 0;

}  // namespace

namespace test {

void addTest(const char* name, TestFn fn) {
  if (name == 0 || fn == 0 || gCount >= kMaxTests) {
    return;
  }
  gTests[gCount].name = name;
  gTests[gCount].fn = fn;
  ++gCount;
}

bool within(double actual, double expected, double eps) {
  const double tolerance = eps < 0.0 ? -eps : eps;
  return std::fabs(actual - expected) <= tolerance;
}

void reportFailure(const char* file, int line, const char* expression,
                   const char* detail) {
  ++gFailures;
  std::printf("FAIL %s:%d  %s %s\n", file, line, expression, detail);
}

void reportNearFailure(const char* file, int line, double actual, double expected,
                       double eps) {
  ++gFailures;
  std::printf("FAIL %s:%d  actual=%.9f expected=%.9f eps=%.9f\n", file, line, actual,
              expected, eps);
}

void reportIntFailure(const char* file, int line, long long actual,
                      long long expected) {
  ++gFailures;
  std::printf("FAIL %s:%d  actual=%lld expected=%lld\n", file, line, actual, expected);
}

int runAll() {
  std::printf("running %d test case(s)\n\n", gCount);
  for (int i = 0; i < gCount; ++i) {
    const int before = gFailures;
    gTests[i].fn();
    if (gFailures == before) {
      std::printf("ok   %s\n", gTests[i].name);
    }
  }
  std::printf("\n%d test case(s), %d failure(s)\n", gCount, gFailures);
  return gFailures == 0 ? 0 : 1;
}

}  // namespace test
