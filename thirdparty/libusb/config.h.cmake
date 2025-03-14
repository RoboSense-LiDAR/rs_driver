#ifndef __CONFIG_H_
#define __CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_VISIBILITY __attribute__ ((visibility ("default")))
#define PRINTF_FORMAT(a, b) __attribute__ ((__format__ (__printf__, a, b)))

#define _GNU_SOURCE 1

#cmakedefine PLATFORM_POSIX @PLATFORM_POSIX@
#cmakedefine PLATFORM_WINDOWS @PLATFORM_WINDOWS@

#cmakedefine ENABLE_LOGGING @ENABLE_LOGGING@
#cmakedefine ENABLE_DEBUG_LOGGING @ENABLE_DEBUG_LOGGING@

#cmakedefine HAVE_CLOCK_GETTIME @HAVE_CLOCK_GETTIME@
#cmakedefine HAVE_EVENTFD @HAVE_EVENTFD@
#cmakedefine HAVE_LIBUDEV @HAVE_LIBUDEV@
#cmakedefine HAVE_PTHREAD_SETNAME_NP @HAVE_PTHREAD_SETNAME_NP@

#cmakedefine HAVE_NFDS_T @HAVE_NFDS_T@

#cmakedefine HAVE_INTTYPES_H @HAVE_INTTYPES_H@

#ifdef __cplusplus
}
#endif

#endif
