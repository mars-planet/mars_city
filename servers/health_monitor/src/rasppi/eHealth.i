%module eHealth
%{
	/* Includes the header in the wrapper code */
	#include "eHealth.h"
	#include "arduPi.h"
%}
// autogenerate docstrings
%feature("autodoc", "1");
/* Parse the header file to generate wrappers */
%include "eHealth.h"
