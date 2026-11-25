#ifndef PLATFORM_STATIC_ASSERT_H
#define PLATFORM_STATIC_ASSERT_H

/* C99 has no _Static_assert.  This macro yields a compile error when the
 * condition is false, and it is valid at file scope. */
#define STATIC_ASSERT_CAT_(a, b) a##b
#define STATIC_ASSERT_CAT(a, b) STATIC_ASSERT_CAT_(a, b)
#ifdef __COUNTER__
#define STATIC_ASSERT(cond, msg) \
    typedef char STATIC_ASSERT_CAT(static_assert_, __COUNTER__)[(cond) ? 1 : -1]
#else
#define STATIC_ASSERT(cond, msg) \
    typedef char STATIC_ASSERT_CAT(static_assert_, __LINE__)[(cond) ? 1 : -1]
#endif

#endif /* PLATFORM_STATIC_ASSERT_H */
