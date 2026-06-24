#pragma once

//this is needed as otherwise the host compiler will complain about unknown declarations for DS_HOSTDEVICE macros

#if defined(__CUDACC__) || defined(__CUDABE__)
//#  define DS_HOST __host__ // Note: All functions are implicitly host functions unless specified otherwise.
#  define DS_DEVICE __device__
#  define DS_DEVICE_INLINE __device__ static __forceinline__
#  define DS_HOSTDEVICE __host__ __device__
#  define DS_KERNEL extern "C" __global__
#else
//#  define DS_HOST
#  define DS_DEVICE [[deprecated("Should not call device only functions on host.")]]
#  define DS_DEVICE_INLINE [[deprecated("Should not call device only functions on host.")]]
#  define DS_HOSTDEVICE
#  define DS_KERNEL
#endif
