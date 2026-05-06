#pragma once

#if (defined(_WIN32) || defined(_WIN64))
#ifdef AXIOM_PHYS_BUILD_DLL
#define AXIOM_PHYS_API __declspec(dllexport)
#elif AXIOM_PHYS_IMPORT_DLL
#define AXIOM_PHYS_API __declspec(dllimport)
#else
#define AXIOM_PHYS_API
#endif
#else
#define AXIOM_PHYS_API
#endif
