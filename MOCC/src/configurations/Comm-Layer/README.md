# Communication Layer


## Onboarding Info

Background on what we're building
http://marscity.readthedocs.io/en/latest/MOCC/docs/MOCC_design.html
http://marscity.readthedocs.io/en/latest/MOCC/docs/MOCC_implementation.html

We're trying to replace our communications layer, based on Tango-Controls http://www.tango-controls.org/ (not really important, just here for completeness).
Tango has some Python bindings http://pytango.readthedocs.io/en/stable/ which we want to replace that with our own library, based on REST.
We're using Flask, plus a module called flasgger https://github.com/rochacbruno/flasgger/ that lets you define OpenAPI-compliant services.

The objective is to have a drop-in replacement for PyTango.
