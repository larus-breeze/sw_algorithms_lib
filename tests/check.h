// Minimal test helpers shared by the test programs in tests/.
//
// check(ok, what)                     a normal assertion
// check_known_issue(ok, what, issue)  an assertion that is known to fail
//     until the named issue is fixed. A failure prints XFAIL and does not
//     fail the run; a pass prints XPASS and DOES fail the run, so the fix
//     has to remove the known-issue marker and the check guards it from
//     then on.
#ifndef TESTS_CHECK_H_
#define TESTS_CHECK_H_

#include <stdio.h>

struct test_counts_t { unsigned checks, failures, known; };
static test_counts_t test_counts = { 0, 0, 0 };

static void check( bool ok, const char * what)
{
  ++test_counts.checks;
  if( ! ok)
    {
      ++test_counts.failures;
      printf( "FAIL: %s\n", what);
    }
}

static void check_known_issue( bool ok, const char * what, const char * issue)
{
  ++test_counts.checks;
  if( ok)
    {
      ++test_counts.failures;
      printf( "XPASS: %s (known issue %s seems fixed: remove the known-issue marker)\n", what, issue);
    }
  else
    {
      ++test_counts.known;
      printf( "XFAIL: %s (known issue %s)\n", what, issue);
    }
}

//! print the summary line and return the process exit code
static int test_summary( void)
{
  printf( "%u checks, %u failed, %u known issues\n", test_counts.checks, test_counts.failures, test_counts.known);
  return test_counts.failures == 0 ? 0 : 1;
}

#endif /* TESTS_CHECK_H_ */
