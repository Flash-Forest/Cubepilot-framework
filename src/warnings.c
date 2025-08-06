#include <ch.h>

#if __GNUC__ != 10 || __GNUC_MINOR__ != 3 || __GNUC_PATCHLEVEL__ != 1
#error Please use arm-none-eabi-gcc 6.3.1.
#endif

#if !CH_DBG_SYSTEM_STATE_CHECK
#warning Consider enabling CH_DBG_SYSTEM_STATE_CHECK in framework_conf.h.
#endif

#if !CH_DBG_ENABLE_CHECKS
#warning Consider enabling CH_DBG_ENABLE_CHECKS in framework_conf.h.
#endif

#if !CH_DBG_ENABLE_ASSERTS
#warning Consider enabling CH_DBG_ENABLE_ASSERTS in framework_conf.h.
#endif

#if !CH_DBG_ENABLE_STACK_CHECK
#warning Consider enabling CH_DBG_ENABLE_STACK_CHECK in framework_conf.h.
#endif

#ifndef MODULE_FREEMEM_CHECK_ENABLED
#warning Consider enabling module freemem_check.
#endif
