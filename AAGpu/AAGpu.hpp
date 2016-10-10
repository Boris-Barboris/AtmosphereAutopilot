#pragma once

#ifdef AAGPU_EXPORTS
#define AAGPU_EXPORTS_API __declspec(dllexport) 
#else
#define AAGPU_EXPORTS_API __declspec(dllimport) 
#endif

namespace AAGpu
{

    AAGPU_EXPORTS_API int execute();

}