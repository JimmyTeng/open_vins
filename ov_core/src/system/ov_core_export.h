#ifndef OV_CORE_EXPORT_H
#define OV_CORE_EXPORT_H

// Cross-platform symbol visibility macro for shared library APIs.
// - Build shared library: define OV_CORE_BUILD
// - Build/consume static library: define OV_CORE_STATIC (exports become empty)
#if defined(OV_CORE_STATIC)
  #define OV_CORE_API
#elif defined(_WIN32) || defined(__CYGWIN__)
  #ifdef OV_CORE_BUILD
    #define OV_CORE_API __declspec(dllexport)
  #else
    #define OV_CORE_API __declspec(dllimport)
  #endif
#elif defined(__GNUC__) && (__GNUC__ >= 4)
  #define OV_CORE_API __attribute__((visibility("default")))
#else
  #define OV_CORE_API
#endif

// Backward/compat aliases for C APIs in this repo.
#ifndef VIO_API
  #define VIO_API OV_CORE_API
#endif

#ifndef SS_VIO_API
  #define SS_VIO_API OV_CORE_API
#endif

#endif // OV_CORE_EXPORT_H
