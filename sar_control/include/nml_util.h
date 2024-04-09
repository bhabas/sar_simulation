/**
Copyright 20201 Andrei N. Ciobanu

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifdef __cplusplus
extern "C" {
#endif

#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

#define DEBUG_MODULE "NML"
#include <debug.h>



// -----------------------------------------------------------------------------

// Enable this to allow debugging messages
#define DEBUG_TRUE 1

#define BLACK "\033[0;30m"
#define RED "\033[0;31m"
#define GREEN "\033[0;32m"
#define BLUE "\033[0;34m"
#define PURPLE "\033[0;35m"
#define CYAN "\033[0;36m"
#define WHITE "\033[0;37m"
#define YELLOW "\033[0;33m"
#define RESET "\033[0m"

float nml_rand_interval(float min, float max);
void nml_log(const char* str);
void NP_CHECK(void* ptr);


#define NML_FLOG(fmt) nml_log(fmt)


#define NML_FINFO(fmt) nml_log(fmt)


#define NML_FERROR(fmt) nml_log(fmt)


#define NML_ERROR(fmt) nml_log(fmt)




#ifdef __cplusplus
}
#endif
