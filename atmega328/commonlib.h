#ifndef COMMONLIB_H // this is a header guard
#define COMMONLIB_H

#include <Arduino.h>
#include <limits.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#define SUCCESS 0
#define FAILURE -1
#define CRITICAL_FAILURE -2

long long_pow(int base, int index);

//function overloading for printing
void print_var_full(String variableName, String variableValue, String functionName);
void print_var_full(String variableName, float variableValue, String functionName);
void print_var_full(String variableName, double variableValue, String functionName);
void print_var_full(String variableName, int variableValue, String functionName);
void print_var_full(String variableName, long variableValue, String functionName);
void print_var_full(String variableName, unsigned long variableValue, String functionName);
void print_var_full(String variableName, bool variableValue, String functionName);
#define PRINT_VAR(variableName, variableValue) print_var_full(variableName, variableValue, "")

void print_class_name(String className);

#endif
