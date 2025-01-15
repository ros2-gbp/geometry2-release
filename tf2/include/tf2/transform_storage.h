// Copyright 2010, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/** \author Tully Foote */

#ifndef TF2__TRANSFORM_STORAGE_H_
#define TF2__TRANSFORM_STORAGE_H_

#ifdef __GNUC__
#define TF2_TRANSFORM_STORAGE_H_PRAGMA(X) _Pragma(#X)
#define TF2_TRANSFORM_STORAGE_H_DEPRECATED(MSG) TF2_TRANSFORM_STORAGE_H_PRAGMA(GCC warning MSG)
#elif defined(_MSC_VER)
#define TF2_TRANSFORM_STORAGE_H_STRINGIZE_(MSG) #MSG
#define TF2_TRANSFORM_STORAGE_H_STRINGIZE(MSG) TF2_TRANSFORM_STORAGE_H_STRINGIZE_(MSG)
#define TF2_TRANSFORM_STORAGE_H_DEPRECATED(MSG) \
  __pragma(message(__FILE__ "(" TF2_TRANSFORM_STORAGE_H_STRINGIZE(__LINE__) ") : Warning: " MSG))
#else
#define TF2_TRANSFORM_STORAGE_H_DEPRECATED(MSG)
#endif

TF2_TRANSFORM_STORAGE_H_DEPRECATED(  // NOLINT
  "This header is obsolete, please include 'tf2/transform_storage.hpp' instead")

#include <tf2/transform_storage.hpp>

#endif  // TF2__TRANSFORM_STORAGE_H_
