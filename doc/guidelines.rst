==============================================================
Engineering Practices Guidelines for the ERAS Project Software
==============================================================

This document intends to provide an overview of the Software Engineering (SE) practices for the ERAS Project Software. The main areas covered are: Coding Standards, Version Control, Change Management, Development Environment, Static and Dynamic Verification and Documentation.

If an answer cannot be found here, use the existing code as an example or ask on the mailing list: erasproject@googlegroups.com

Reference Documents
===================

* [1]   PEP 8 -- Style Guide for Python Code, http://www.python.org/dev/peps/pep-0008
* [2]   Mercurial -- http://mercurial.selenic.com/
* [3]   pep8 -- Python style guide checker, https://pypi.python.org/pypi/pep8
* [4]   radon -- Python static code analysis tool, https://github.com/rubik/radon
* [5]   pyunit -- Python Unit Testing Framework, http://pyunit.sourceforge.net/pyunit.html
* [6]   coverage -- Code coverage measurement for Python, https://pypi.python.org/pypi/coverage

Coding Standards
================

ERAS software must be readable, easy to maintain and as least error prone as possible.  It will have to use official standards as much as possible.  To this purpose, coding standards should be applied systematically.
Coding standards range from proper code documentation to file naming conventions and in general help in preventing a certain category of software bugs. They do not address specific implementation details like algorithms or programming methodologies.
The application of coding standards is deemed crucial to any modern software undertaking.

The main coding standards reference for Python can be found in [1].
Here follow the most important recommendations:

* don't write more than 80 chars per line

* use spaces around operators like ``=``, ``+``, ``==``

* don't put spaces between the function name and the ``(``

* don't write more statements on the same line.

* use ``CamelCase`` for classes names, ``lowercase`` or ``lower_with_underscores`` for the rest

* avoid ``mixedCase``.

* use clear names for variables, functions, and classes.


Version Control
===============

Software developers use a Version Control tool to baseline configurations, prepare releases and to deal with synchronous update of items. For release preparation in particular, code freezing or “tagging” is required i.e. a simple notification by the package responsible, that a certain version of the package has achieved its planned objectives and may be taken from the repository for the release.
The Version Control tool  must enable:

* identification of items

* traceability of changes (who did what, when)

* accessibility of previous configurations for any item

Both source code and documentation must be subject to strict version control.
The tool chosen to support ERAS Software Version Control is Mercurial (HG). Please refer to [2] for a more exhaustive description of the tool.

Configuration Management Item: Software Package
-----------------------------------------------

A software package is a piece of software (code and documentation) able to perform functions and having an interface available to an external user to access the functions provided.
Technically a package is a way to organize functions in homogeneous groups. The interface hides the implementation and system dependencies from the user.
Managerially the package is the basic unit for planning, software project control and configuration control.
There is no rule to define how big a package shall be. Common sense and programming experience should be enough to identify what can be gathered and treated as a unique item.
Each package shall have one responsible person, who will be entitled to delegate activities on the package but shall retain responsibility at all times.

Package identification
----------------------

A package is identified by its name and its version mnemonics. The version identification model is based on:
a version is defined with a mnemonic comprising at least two numbers for the major (v) id and the minor ( r) id (a third number can be present representing a patch (p) id.
The increase of these two numbers during a development has to follow those rules:
if the changes in the libraries of package A are not backward compatible (it is necessary to change something in packages which are using A) then the major id number must be increased on one unit.
if the change does not imply any modifications in the other packages, the minor id number must be increase by one unit.
A patch version is sometimes useful to isolate a change in the code during a temporary test for instance. The package version is then labeled vXrYpZ and is not meant to be included into an ERAS software release.

Package structure
-----------------

Each package will correspond to a TANGO server or client and should contains the following:

* an ``xxx.py`` file that implements the functionality of the server, but it's independent from TANGO.

* an ``xxx`` file that implements the actual TANGO server/client by importing
  ``xxx.py`` and defining TANGO-specific classes.  This file should not
  have an extension and should be set as executable, otherwise TANGO
  won't be able to use it.

* a ``test_xxx.py`` that includes unittests for ``xxx.py``.

* an ``doc`` directory containing:

    - ``xxx.rst`` file that includes generic documentation for ``xxx.py``.

    - ``swrs_xxx.rst`` User Requirements Specification for ``xxx.py``.

    - ``swds_xxx.rst`` Design Study for ``xxx.py``.

    - ``swum_xxx.rst`` User and Maintenance Manual for ``xxx.py``.

* possibly additional files required by the server/client.



HG Repository structure
-----------------------

The ``servers`` dir will contain all the TANGO servers/clients.  Every server/client has its own directory.
The ``doc`` directory contains the general documentations, such as instructions about TANGO installation and setup, templates, etc.


Branches and heads
------------------

Don't push new branches or heads on the repository.  Before committing make
sure that there are no incoming changesets (``hg incoming``), and if there are
use ``hg pull`` to pull them.  If you accidentally commit before pulling and
create a new head (you can check with ``hg heads .``), you will have to use
``hg merge`` and ``hg commit`` to merge the heads before being able to push.

Commits, commit messages and tags
---------------------------------

One commit per issue.  Adding a new class with tests and documentation is OK.
Fixing a bug and adding a new feature in the same changeset is *not* OK.
Fixing two unrelated bugs or adding two unrelated features in the same
changeset is *not* OK.
"Work in progress" changesets should be avoided -- the code should work at
every changeset (it's OK to make a commit for a basic but still incomplete
class that works, and add more features afterwards).

Before committing use ``hg diff`` and ``hg status`` to make sure that what
you are committing is OK and that all the files are included and that there
are no unrelated changes.  If necessary you can update the ``.hgignore`` file.

Descriptive, non-empty comments are required for each commit. They must be complete and readable, making reference to SPR entries when applicable and explaining briefly what the changeset does in the present tense.  ``"Implement new feature XXX."``,
``"Fix bug XXX by using YYY."``, ``"Add tests for the XXX class."``,
``"Improve documentation for XXX."`` are *good* commit messages.
``Fix a bug.``, ``fix a bug``, ``improve the code`` are *bad* commit messages.

Before pushing into the central repository your changeset must be tagged using the version identification model (vXrYpZ) previusly mentioned.

Change Management
=================
In ERAS we will be using the Issue Tracker embedded into the Bitbucket software repository as Change Management tool (https://bitbucket.org/italianmarssociety/eras/issues?status=new&status=open).

The tool will allow internal or external users of the ERAS Software to report problems/errors, submit change requests or to require clarification on software, hardware or documentation.

Here we briefly summarize the basic workflow of the system:

- Issue submitted and all relevant people add themselves as monitoring users
- Notes added by any user
- A  Responsible Person is assigned for the issue
- Responsible works on issue
- Responsible add a final remark on the issue and software manager close it.   


Static and Dynamic Verification
===============================

Code Inspections (Static Verification)
--------------------------------------

Adoption of approved coding standards must be periodically monitored and this can be achieved by inspections of the code. Both manual (human) and automatic inspections are possible. Source code will be subject to scrutiny (at package level) by suitable software tools which will rate the code according to compliance to predefined guidelines.
Human inspections will be done for certain packages of special relevance or for those code segments which exhibit a remarkably high algorithmic complexity.
Tools measuring standards metrics (like McCabe cyclomatic complexity) will be used to identify which software packages are more prone to exhibit faulty behavior, and should therefore be tested more thoroughly.

More specifically for the Python language, developers must use the tool ``pep8`` [3] in order to check compliance with the PEP 8 standards before pushing on the main repository.
Once notified of a package release, Software Mentors will make use of ``pep8`` [3] and ``radon`` [4] to identify the code segments to be reviewed, review them and provide feedback to developers. Developers will then commit required modifications.


Testing (Dynamic Verification)
------------------------------

The amount of software faults or incorrect behaviors in the ERAS software  must be kept to a minimum and the system must be validated, i.e. it must be guaranteed that it is working according to its specifications. The application of a consistent testing scheme and the diffusion of a “testing culture” will help to achieve this goal.
Although the developer is encouraged to delegate test code writing to someone else, it is his/her final responsibility to make sure that his/her package has achieved a sufficient degree of testing. 
A formal testing scheme will be adopted to ensure developers push only packages, which have been previously tested. During integration software packages versions may be rejected if they do not provide sufficient testing certification.
Developers are required to start working on their test suites as a result of design, prior to implementation (i.e. use test-driven development). The responsible for each software subsystem will make sure that two types of regression tests are performed:

* Unit tests: the smallest unit is tested under isolation. If needed, the behaviour of other code units interacting with the unit under test will be mimicked by building stubs.

* System tests: the system (or subsystem) as a whole is tested against its functional specifications

Tests should be defined for each release and based on the Use Cases which have been implemented. This will permits to trace the requirements through the whole process. All test procedures must be fully automatic or, when this is not possible, based on a detailed checklist.

For development in Python:

* unit tests should be developed using ``pyunit`` (``unittest``).

* In order to determine the amount of code coverage of each test suite and thus its sensibility the use of the ``coverage`` [5] tool is mandatory. At each package release the obtained coverage report must be provided. 

Documentation
=============

The appropriate documentation has to be written together with the code.
We can individuate those levels and types of documentation:
 
1.    Comments inside the code
2.    Release Notes
3.    Manuals
 
Release Notes
-------------
For every major release of the ERAS Software, the Release Notes for all the ERAS applications and programs has to be produced. It is up to the ERAS software manager to organize the Release Notes, their delivery with the ERAS distribution and publishing on the web.
 
Manuals
-------
Software documentation must cover the entire software process, from the Requirements phase (User Requirements Specification) to the Design (Design Study) to the User documentation (Software User Manual and Software Maintenance Manual).
The documents should go under configuration control in the software repository within the software package.
All the documentation is written in reStructuredText. Before committing
it should be checked that the documentation builds without errors or warnings, by running
``make html``.  After building the documentation it should be open with a browser and check that it looks OK.

