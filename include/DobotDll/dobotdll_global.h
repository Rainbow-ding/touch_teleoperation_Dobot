#ifndef DOBOTDLL_GLOBAL_H
#define DOBOTDLL_GLOBAL_H



#if defined(DOBOTDLL_LIBRARY)
#  define Q_DECL_EXPORT __declspec(dllexport)
#  define DOBOTDLLSHARED_EXPORT Q_DECL_EXPORT
#else
#  define Q_DECL_IMPORT __declspec(dllimport)
#  define DOBOTDLLSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // DOBOTDLL_GLOBAL_H
