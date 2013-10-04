===========================================
Software Design Study for Voice Interface
===========================================

:Author: Eric Meinhardt


Change Record
=============

2013.08.02 - Draft 0.5 posted.


Introduction
============

Purpose & Scope
---------------

This document is intended to detail for developers of the :term:`ERAS` voice
interface library and other :term:`ERAS C3` components how ERAS's voice
interface library is predicted to achieve the features laid out in the system
requirements document [1], now and projected into the future as more details
about the compatibility of different open source spoken dialogue systems with
the broader hardware and software requirements of the :term:`ERAS C3`
prototype.


Reference Documents
-------------------

- [1] -- `Software Requirements Specification for Voice Interface Library. <https://eras.readthedocs.org/en/latest/servers/voicelib/doc/voicelib-sw-reqs.html>`_
- [2] -- `Building application with pocketsphinx. <http://cmusphinx.sourceforge.net/wiki/tutorialpocketsphinx>`_
- [3] -- `Gstreamer: open source multimedia framework. <http://gstreamer.freedesktop.org>`_
- [4] -- `Dependencies. <http://gstreamer.freedesktop.org/data/doc/gstreamer/head/faq/html/chapter-dependencies.html>`_
- [5] -- `Phoenix - Olympus. <http://wiki.speech.cs.cmu.edu/olympus/index.php/Phoenix_Server>`_
- [6] -- `SEMAFOR: Semantic Analyzer of Frame Representations. <http://www.ark.cs.cmu.edu/SEMAFOR/>`_
- [7] -- `semafor-semantic-parser: A natural language shallow semantic parser developed at Carnegie Mellon University. <https://code.google.com/p/semafor-semantic-parser>`_
- [8] -- `A python wrapper for Semaphore, a Shallow Semantic Parser that identifies roles in a text. <https://github.com/jac2130/semaphore-python>`_
- [9] -- Forthcoming wiki entry.
- [10] -- `Sphinx-4 - A speech recognizer written entirely in the JavaTM programming language. <http://cmusphinx.sourceforge.net/10/>`_
- [11] -- `Software Engineering Practices Guidelines for the ERAS Project. <https://eras.readthedocs.org/en/latest/doc/guidelines.html>`_
- [12] -- `Integration and Field Testing. <https://bitbucket.org/italianmarssociety/eras/wiki/Integration%20and%20Field%20Testing>`_
- [13] -- `CMU Sphinx FAQ. <http://cmusphinx.sourceforge.net/wiki/faq>`_
- [14] -- `Know When to Stop Designing, Quantitatively. <http://humanized.com/weblog/2006/07/22/know_when_to_stop_designing_quantitatively>`_
- [15] -- `Language-Based Interfaces, part 1: The problem. <http://jonoscript.wordpress.com/2008/07/21/language-based-interfaces-part-1-the-problem/>`_
- [16] -- `Notes on Prior Art in Rover Voice Control. <https://bitbucket.org/italianmarssociety/eras/wiki/Notes%20on%20Prior%20Art%20in%20Rover%20Voice%20Control>`_
- [17] -- `Opensource - Mission Simulation Toolkit. <http://ti.arc.nasa.gov/opensource/projects/mission-simulation-toolkit/>`_

Glossary
--------

.. To create a glossary use the following code (dedent it to make it work):

.. glossary::

    ``ERAS``
      European Mars Analogue Station

    ``ERAS C3``
       European Mars Analogue Station Command, Control, and Communication

    ``CMU``
       Carnegie-Mellon University.

    ``ASR``
       Automatic speech recognition.

    ``SLU``
       Spoken language understanding.

    ``SDS``
       Spoken dialogue system.

.. Use the main :ref:`glossary` for general terms, and :term:`Term` to link
   to the glossary entries.


Design Considerations
=====================

Interface requirements and planning with future feature growth in mind are
the two chief sources of design considerations:

* spoken-language command interfaces, as opposed to e.g. systems for coarse-
  grained understanding of large /text/s or telephone dialog systems for
  database query or customer service transactions, pose unique and less-
  studied challenges for user interface design - opacity of system state,
  difficulties with error handling, and uncertainty over user intent.
* the currently underspecified nature of the software architecture for semi-
  autonomous control (planning and decision making) of the :term:`ERAS` analogue rover
  together with the desire to let the complexity of that architecture evolve
  over time means that the semantic (wordform sequence -> action request
  mapping) component of the voice interface library must be designed with an
  eye to modularity and some sense of more or less likely avenues for the
  coevolution of the voice interface library's semantic component and the
  rover's automated planner/central executive. Crucially, growth in features,
  especially past the early stage, will involve large additions of code for
  modest growth of features.


Assumptions and dependencies
----------------------------

Forseeable long-term dependencies:

* Python 2/3
* TANGO / PyTango bindings
* Ubuntu 12.04 LTS
* :term:`CMU` Sphinx toolkit

Early/current dependencies:

* Pocketsphinx [2] and Gstreamer [3], including all of Gstreamer's
  dependencies ([4]).
* Either the Phoenix semantic parser (as maintained by the :term:`CMU` Olympus
  project [5]), which is written in C (and for which ctypes Python bindings
  would have to be written), or the SEMAFOR semantic parser [6-7] (for which
  Python bindings [8] do exist), which is written in Java. A comparison is
  available in [9].

Later dependencies:

* All available open-source spoken dialogue systems that meet basic project
  interface requirements (see [9]) appear to depend on Java, and further,
  generally make use of Sphinx 4 [10] rather than Pocketsphinx [2], but it is
  not clear at this point whether this is good, bad, or if bad, how
  resolvable.

Probable and future changes in features are described in [1]. Options for
spoken dialogue systems are detailed in a review ([9]).

General Constraints
-------------------

* ERAS Software Engineering Practices Guidelines [11]
* The ambient noise level in a helmet is quite high [12]; the :term:`CMU`
  Sphinx FAQ [13] offers some suggestions.
* According to the field testing notes [12], power consumption may be an
  important issue; Pocketsphinx is the preferred :term:`CMU` Sphinx library for
  mobile (e.g. fast, low-battery use) automatic speech recognition, but Sphinx
  4 may be an easier option for development.

Objectives
----------
As an interface

#. Achieve high **information efficiency** for each voice command. Per
   reference [14], this measure is chiefly useful as a heuristic
   for measuring room for improvement (or lack thereof). NB Cruder estimates
   of this will have to do until a larger database of training data is
   available.
#. **Appropriate expressivity**, primarily for the user (the number of forms
   the finite set of commands a rover understands can take), but also for
   feedback. The amount and variety of expressivity allowed will grow as
   feature requirements 5 and 6 are met more extensively - e.g. for users,
   more complex grammars and more complex pragmatic understanding on the part
   of the rover voice agent; for the voice agent, text, synthesized speech,
   and some amount of supplementary graphical interface will each allow more
   flexibility in feedback quantity and quality, as situations and users
   demand.
#. **Learnability** is the final interface priority, albeit the lowest one,
   given difficulty of measurement and that users should have sufficient time
   to master the capabilities and limitations of the voice interface library,
   whether in its early, evolving stages or its future, more sophisticated
   ones.

(See reference [15].)

As software,

* maximizing ease of feature growth (see requirements 6-8 in [1]) by

  * making the major components (automated speech recognition, spoken language
    understanding, dialogue management, task management, and natural language
    generation/feedback) as modular as possible

    * each module should have well-defined and as stable as possible core
      methods as possible for interfacing modules to call and use;

  * making minimal unwarranted assumptions about the general planning
    capabilities or architecture of the rover
  * not letting a lightweight system slowly build up more complexity than it
    can handle, thus making the later move to a spoken dialogue system more
    painful than it should be: upgrading the spoken dialogue system
    (particularly its approach to task management/adding new domain-specific
    agents) to handle generally complex queries and dialogue situations should
    take priority over (over)extension of a lightweight architecture.

* meeting the Performance Requirements outlined in [1].


Software Architecture
=====================

The core of voice interface library architecture consists of five components:

1. an automated speech recognition object (responsible for Requirements 1-3)
2. a spoken language understanding object (responsible for Requirement 4)
3. a dialogue manager (responsible for Requirements 4, 5)
4. a task manager (responsible for Requirement 4)
5. a natural language generation object (the front-end of Requirement 5)

Optionally,

6. a text-to-speech synthesizer (part of Requirement 6)

may also be a component that should require a minimal amount of additional
maintenance or work once it is in place; it would be part of the front-end of
Requirement 5, and is part of most (if not all) open source spoken dialogue
systems or frameworks.

For the purpose of development, two additional components

7. a test object
8. a dummy rover executive - responsible for checking for reasons why a
   requested action cannot be begun or completed)

are also part of the architecture of the project.

The current plan is for at least component 1 (and possibly also 2) - and
if/when it exists, component 6 - to reside on a Tango server onboard an
analogue astronaut's suit, with all other components residing onboard any
particular rover, for reasons of minimizing network bandwidth use.

Architecture Development Plan
-----------------------------
As mentioned above, the voice interface library is designed with growth in
mind. Below is an approximate development ordering, less reliable the further
from phase 1 the further forward one goes.

First Phase - ASR and Testing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Implement components 1 and 7.

Draft test scenarios, using use cases in the system requirements document as a
guide [1]; record database of plausibly useful commands with different levels
of complexity.

Test :term:`ASR` in Wizard-of-Oz setups (the user's audio gets piped into
the :term:`ASR`, the 'person behind the curtain' sees the resulting recognized
text, and relays/requests feedback to the user), asking multiple users to
specify as many reasonable variations of plausible commands at each stage as
possible (i.e. elicit commands from people unfamiliar with the training
corpus). Make few or no assumptions about the rover's natural language
understanding abilities. A simulator would be nice for this; the scenarios
mentioned in the prior art research [16] (especially the transcripts studied
in Clancey, 2004) may be a good starting place; note also for future reference
(1) that ROS comes with a simulation package (2) NASA's open source Mission
Simulation Toolkit [17] may also be of future use here.

In the course of iterative testing (revising the language model and parameters
to the :term:`ASR` model), attempt to optimize all relevant performance
requirements - everything except lag-to-feedback and experiment with (or at
least record) the amount of training required (or used) for speaker-dependent
recognition models.

Second Phase
~~~~~~~~~~~~
Focus on component 2 and extend test component appropriately.

The decision must be made whether to (a) start fulfilling (or at least
facilitating later fulfillment of) Requirements 6-7 by picking an existing
open source dialogue system (which come with components 1-6, and usually
Sphinx 4 for component 1) to start integrating (beginning with :term:`SLU`), or
(b) to defer choosing a spoken dialogue system until later and instead start
using one of the two identified lightweight spoken language understanding
systems (see [9]), with the intention of interfacing it with a later
chosen :term:`SDS` or switching altogether to the native :term:`SLU` framework
and code for the chosen :term:`SDS`.

Once the :term:`SLU` system is in place, another round of Wizard-of-Oz tests
should resume, documenting and iteratively improving the performance of
the :term:`SLU` component.

Third/Fourth Phases
~~~~~~~~~~~~~~~~~~~
The next phase should pursue basic dialog management capabilities (component
3) as well as task management and basic feedback (components 4 and 5), with
the exact order and degree of detail depending on what seems a more pressing
need in test scenarios. Wizard-of-Oz contests should continue as needed. (The
test component should continue to expand appropriately.)

Fourth/Fifth Phases
~~~~~~~~~~~~~~~~~~~
Component 8 should be implemented, requiring further extension of components 5
and 7.

Fifth/Sixth Phases
~~~~~~~~~~~~~~~~~~
Requirements 6 and/or 7 can be worked on as needed.
