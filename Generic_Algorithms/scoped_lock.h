#ifndef GENERIC_ALGORITHMS_SCOPED_LOCK_H_
#define GENERIC_ALGORITHMS_SCOPED_LOCK_H_

class ScopedLock {
public:
    explicit ScopedLock( Mutex_Wrapper_Type& _m)
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
    Mutex_Wrapper_Type& m;
};

#endif /* GENERIC_ALGORITHMS_SCOPED_LOCK_H_ */
