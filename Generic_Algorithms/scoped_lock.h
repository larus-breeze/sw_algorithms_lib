#ifndef GENERIC_ALGORITHMS_SCOPED_LOCK_H_
#define GENERIC_ALGORITHMS_SCOPED_LOCK_H_

#include "stdio.h"

class mutex
{
public:
  void lock( void)
  {
    printf("lock ... ");
  }
  void unlock( void)
  {
    printf("release\n");
  }
};

class ScopedLock {
public:
    explicit ScopedLock( mutex& _m)
        : m(_m)
    {
        m.lock();          // acquire in constructor
    }

    ~ScopedLock()
    {
        m.unlock();        // release in destructor
    }

    // make this thing non-copyable
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;

private:
    mutex& m;
};

extern mutex my_mutex;

#define LOCK_SECTION() ScopedLock lock( my_mutex)

#endif /* GENERIC_ALGORITHMS_SCOPED_LOCK_H_ */
