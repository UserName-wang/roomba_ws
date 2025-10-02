#ifndef ROOMBA_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define ROOMBA_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROOMBA_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define ROOMBA_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROOMBA_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define ROOMBA_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROOMBA_HARDWARE_INTERFACE_BUILDING_DLL
    #define ROOMBA_HARDWARE_INTERFACE_PUBLIC ROOMBA_HARDWARE_INTERFACE_EXPORT
  #else
    #define ROOMBA_HARDWARE_INTERFACE_PUBLIC ROOMBA_HARDWARE_INTERFACE_IMPORT
  #endif
  #define ROOMBA_HARDWARE_INTERFACE_PUBLIC_TYPE ROOMBA_HARDWARE_INTERFACE_PUBLIC
  #define ROOMBA_HARDWARE_INTERFACE_LOCAL
#else
  #define ROOMBA_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define ROOMBA_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define ROOMBA_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define ROOMBA_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROOMBA_HARDWARE_INTERFACE_PUBLIC
    #define ROOMBA_HARDWARE_INTERFACE_LOCAL
  #endif
  #define ROOMBA_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // ROOMBA_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_