=========================================================
Software Requirements Specification for kim
=========================================================

:Author: Eric Meinhardt

Change Record
=============

2013.06.20 - Draft 1 completed.

2013.06.27 - Draft 2 - first revisions - completed. Typos fixed, hyperlinks
changed, Franco's architecture comments integrated, added example of macro
use.

Introduction
============

Purpose & Scope
---------------

This document is intended to detail for developers of kim and other
:term:`ERAS C3` components what ERAS's voice interface library is predicted
to be needed for, relevant details about who is predicted to have those
needs, and what features kim has (or will have) to meet those anticipated
needs. This requirements specification is intended to cover a software
library and associated documentation.

Reference Documents
-------------------

- [1]  -- `Dowding, J., Alena, R., Clancey, W. J., Sierhuis, M., & Graham, J.
  (2006). Are you talking to me? dialogue systems supporting mixed teams of
  humans and robots. In AAAI Fall Symposium on Aurally Informed Performance:
  Integrating Machine Listening and Auditory Presentation in Robotic Systems,
  Arlington, Virginia.
  <http://ti.arc.nasa.gov/m/pub-archive/archive/1240.pdf>`_
- [2] -- `Software Engineering Practices Guidelines for the ERAS Project.
  <https://eras.readthedocs.org/en/latest/doc/guidelines.html>`_
- [3] -- `ERAS 2013 GSoC Strategic Plan. <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202013>`_

Glossary
--------

.. To create a glossary use the following code (dedent it to make it work):

.. glossary::

``ELIZA effect`` The human tendency to ascribe human qualities and behaviors
  to computers or software.

``ERAS C3`` European Mars Analogue Station Command, Control, and Communication

``EVA`` Extra-vehicular activity. Doing stuff in a space suit.

``HUD`` heads-up display.

``IMS`` Italian Mars Society.

``OeWF`` The Austrian Space Forum ("Österreichisches Weltraum Forum", auf
  Deutsch.)

.. Use the main :ref:`glossary` for general terms, and :term:`Term` to link
   to the glossary entries.

General Description
===================

Problem Statement
-----------------

Surface-exploration tasks may well require astronauts conducting extra-
vehicular activity to give direct instructions to rovers

* without anything resembling a keyboard-pointer-screen interface
* without requiring the assistance of a human intermediary teleoperating the
  rover via something like a keyboard-pointer-screen interface

Voice interaction allows for natural real-time, hands-free command of a rover
with minimal learning curve and without a need for dedicated, single-use
hardware.

Functional Description
----------------------

The kim library will support the creation of software agents capable of

#. accepting an audio signal,
#. deciding whether the audio signal is addressed to the software agent,
#. attempting to map a relevant signal onto a word sequence,
#. determining the rover action request associated with the understood word
   sequence,
#. and providing appropriate feedback to the user.

Kim's first release should allow for the simplest possible solution to these
requirements; features that involve detailed (or more speculative) knowledge
of the capabilities of the rover software agent(s) that control actuators
(effectors) or take input from sensors (e.g. complex inference about context,
richer feedback to the speaker about the internal state of the rover software,
etc) are *not* part of the design goals for an initial release, although
creating modularity and abstractions that allow for easier addition or
interface with such code in later updates *is*.

For now, advanced features (in approximately this order of priority) include

#. different types, degrees, and mediums of feedback (text, artificial speech,
   and more demanding graphical displays), with implementation dependent in
   part on additional details about the Aouda.X :term:`HUD` and/or MARVIN.
#. support for a dialog manager (for managing conversation-related inference)
   and other more advanced natural language processing capabilities built on
   top of other components of the rover software executive
#. easy-to-use, low maintenance learning mechanisms, starting with the
   capacity for simple user-definable macros that can be 'written' entirely in
   the field and 'on the fly'. For example, suppose an astronaut decides, in
   the field, that she or he wants the rover to take 2 pictures (each with
   different camera settings) with, say, him- or herself at the center, at
   multiple locations. Without macros and without pre-EVA scripting of this
   task, the astronaut will have to go through this loop

      #. With the rover following, proceed to the next location where pictures
         are desired.
      #. Tell the rover to take a picture of the astronaut with parameter set
         1.
      #. Tell the rover to take a picture of the astronaut with parameter set
         2.
   every time a pair of pictures at a new location is desired. With the
   ability to record simple macros, the astronaut can instead tell the rover
   to 'start recording', give instructions to the rover - in the case of the
   example above, 'Follow closely.'...'Stop.'...'Take a picture of me using
   <settings abc>.'...'Take a picture of me using <settings xyz>.'...'Stop
   recording. Label this macro <macro-name>.'

Environment
-----------

Kim is intended to be written in Python, with an instance hosted on the
onboard computer of the analogue space suit (Aouda.X), (wrapped in a Tango
distributed control system object) running Ubuntu 12.04 (LTS), and to interact
well with other elements of the :term:`ERAS C3` Prototype, namely the rover
executive / planning agent.

User classes & objectives
-------------------------
(Analogue) Astronaut
~~~~~~~~~~~~~~~~~~~~
Speech will be used by astronauts to direct the rover, ideally, as astronauts
are used to using speech - as with other human beings (see the :term:`ELIZA
effect`), but probably have had enough experience talking to phone-based
dialog agents and/or smartphone assistants to lower their expectations.

In more detail, this means a kim instance must provide feedback (answering the
user question "Did the rover hear me and understand what I asked?") and
require a minimum of extra explicitness that a conversation with a human being
would be unlikely to contain: i.e. a kim instance should have some means of
modeling conversational context.

As well, as a control interface, astronauts want as clearly as possible to
know what options they have (i.e. what the rover is listening for) to direct
the rover at any given moment (e.g. the rover might understand a request to
turn, but isn't sure what direction or how far, etc.) and what the limits are
on how they can pursue those options (i.e. what they can reasonably expect the
rover will or will not understand).

Users will be expected to have extensive opportunities to learn the
capabilities and limitations of the kim library and also to provide more than
enough training data for speech recognition models prior to field testing.

Functional Requirements
=======================

Requirement 1: Receive audio stream
-----------------------------------
Description
~~~~~~~~~~~
The kim instance should be able to receive a local audio stream.

Criticality
~~~~~~~~~~~
High. This is an essential feature.

Dependency
~~~~~~~~~~
This functional requirement depends on an interface requirement - interfacing
with other Tango (ERAS C3) objects. (See the software interface requirement.)

Requirement 2: Classify audio signal addressee
----------------------------------------------
Description
~~~~~~~~~~~
A kim agent (instance) needs to be able to determine whether or not the stream
it's receiving contains linguistic content directed at it.

Criticality
~~~~~~~~~~~
High. This is an essential feature.

Dependency
~~~~~~~~~~
This functional requirement depends on receiving an audio stream (the first
functional requirement).

Requirement 3: Map relevant signal to word sequence
---------------------------------------------------
Description
~~~~~~~~~~~
A kim software agent needs to infer from the audio signal what a matching word
sequence is; library availability and efficiency vs. effectiveness trade-offs
will determine how complex this needs to be (e.g. committing to a single most-
probable word stream from t=0 forward vs. holding some small number of
candidate word sequences in parallel and dynamically reranking them as the
signal unfolds).

Criticality
~~~~~~~~~~~
High. This is an essential feature.

Dependency
~~~~~~~~~~
Requirement 3 depends on requirement 2 (identifying whether a linguistic
utterance is a rover command).

NB Requirement 2 can be viewed as a strict subset (albeit one notable enough
to pick out) of requirement 3: in its simplest form (starting any command for
some rover with the rover's name), the spoken form corresponding to an address
is a command to 'listen carefully to the rest of what I [the current speaker]
have to say.'

Requirement 4: Map word sequence to action request
--------------------------------------------------
Description
~~~~~~~~~~~
Given a word sequence (or probability estimates over a small number of the
most probable word sequences), a kim software agent must attempt to determine
what action(s) is (are) being requested of the rover.

Criticality
~~~~~~~~~~~
High. This is an essential feature.

Dependency
~~~~~~~~~~
This functional requirement is dependent on feature 3 (mapping a signal deemed
relevant to a word sequence).

Requirement 5: User Feedback
----------------------------
Description
~~~~~~~~~~~
The voice recognition software agent may not recognize or understand some or
all of an utterance it believes directed at it; the kim instance ought, in
such cases, be able to provide appropriate feedback to users.

To start, a kim agent will be able to send text error messages more useful and
informative to an end-user who knows little or nothing about kim, Tango, or
how voice recognition works than what a developer would use for debugging
- stack traces and programmer/scientific jargon will NOT be acceptable. These
can either be transmitted (and viewed) as text or via synthesized speech.

Criticality
~~~~~~~~~~~
High. This is an essential feature.

Dependency
~~~~~~~~~~
This functional requirement is a real-time error recovery mechanism; at least
one of requirements 1-4 needs some minimum level of functionality before
development on feedback messages make much sense. That said, requirements 3
and 4 - mapping an audio signal to a word sequence and interpreting what the
requested action associated with that sequence is - will likely be the
functional requirement most subject to errors and that users therefore are
most likely to want feedback on.

Requirement 6: Rich Feedback
----------------------------
Description
~~~~~~~~~~~
Synthesized speech (minimally text-to-speech versions of the text error
messages), differential length/detail feedback, context-based-inference, and
non-verbal graphical feedback are variations in feedback that will allow an
astronaut to more easily able to understand why kim (or the rover) is not
understanding or complying with the astronaut's request and what they can do
to change this, as circumstances allow.

Criticality
~~~~~~~~~~~
Medium. Graceful recovery from failure will be important in avoiding
frustration on the part of users in the face of brittle technology.

Dependency
~~~~~~~~~~
This functional requirement is dependent on requirement 5 and the software
interface requirements.

Requirement 7: Enhanced Natural Language Processing & Understanding
-------------------------------------------------------------------
Description
~~~~~~~~~~~
The minimal specifications don't make use of any particularly complex natural
language technologies other than speech recognition (knowledge of a language's
phonetics and phonology); syntax is represented in a very simple,
impoverished, and inflexible form; 'conversation' is also a rather lop-sided
affair. A detailed and robust rover executive with an explicit ontology of
objects in the world, model of self and speakers, and more detailed grammar
(of the language in question, of the speech patterns of the astronauts
actually on the mission) would allow for a more natural interface with less of
a learning curve; astronauts would likely spend less time worrying about how
much they need to adjust their answers for the primitiveness of the rover and
what the recognized types and sequences of magic words are to make it do their
bidding.

Specifically, a part of speech tagger, proper name identification/named
entity-extraction, more complex syntactic and semantic parsers, and a dialog
manager, with the latter interfacing with a planning agent (and its associated
formal framework) are starting areas for growth. In particular, investing time
in developing a dialog manager (or the prerequisites thereof) may be the
single most worthwhile investment for additional functionality in the voice
interface, permitting more less code to do more work (instead of explicitly
and duplicatively hand-coding the edge-cases - e.g. error recovery
- for each type of task).

Criticality
~~~~~~~~~~~
Low/Medium.

Criticality depends in part on testing; if a simpler system is good enough for
intended uses, adding more complicated natural language processing components
may end up at worst compromising performance (NLP/NLU is CPU-intensive and
might be a bottleneck in voice command process), easy learning/training curve
(the system may take a long time to learn enough data from users to function
correctly where a simpler system may work well-enough 'out of the box'), and
of course add to the tasks of software development and maintenance.

Dependency
~~~~~~~~~~~
This functional requirement necessitates functional requirements 1-5, at
minimum, and potentially at least further knowledge of planned aspects of the
rover software executive.

Requirement 8: Learning mechanisms
----------------------------------
Description
~~~~~~~~~~~
After initial testing of each feature and use-case scenario, bottlenecks in
functionality (at least earlier in the data-flow, given the nature of
compounding errors and dependencies among functional requirements 1-5) should
become identifiable; the ability to learn from each episode of each feature
use and thereby both improve a kim instance's statistical models of speech,
language, and understanding as well as to add new 'vocabulary' items ("voice
macros") could be an important means of minimizing astronaut frustration and
effort while maximizing an astronaut's ability to direct a rover as they
please.

Specific areas of improvement are below:

* speech models
    * speaker-specific supervised training (having a user read aloud from a
      set of known texts) is normal for some speech recognition models;
      whether those used by models available in the open-source speech
      recognition libraries likely to be used are such speech recognition
      models is unknown at the time of writing; in any case, such training
      does not take very long for substantial gains in accuracy to be
      realized.
    * NB that language variety models (simplistically, "dialect") are
      *probably* not worth pursuing unless there are large numbers of people
      in testing or use that fall into language variety clusters where
      performance is sufficiently poor when accent is not modeled (at all or
      explicitly).
* classifying speech as rover-directed or not
* grammar extensions - more general, flexible models of language will permit
  astronauts to interact more naturally, rather than trying to remember the
  hyperspecific, stilted forms that the rover recognizes.
* vocabulary - astronauts will be able to add new atomic items (e.g. location
  names) to a kim agent's knowledgebase and more complex procedures (e.g. let
  the sequence of actions a, b, and then c be called 'X') composed of simpler
  actions each associated with a voice command.

Criticality
~~~~~~~~~~~
Medium/low; depends on how well or poorly the other features function and how
important extension of the grammar and/or vocabulary seem like they would be
in testing more primitive versions.

Dependency
~~~~~~~~~~~
Low/medium. This feature could plausibly be examined and worked on as each of
feature requirements 1-5, 6, and 7 are completed, although some analysis will
require the first five to be done.

Interface Requirements
======================

User Interfaces
---------------

The user is assumed to have a microphone and at least speakers; a visual
interface capable of displaying at least text is presumed but not required at
this point.

Software Interfaces
-------------------
The Tango object representing the server hosting the kim instance should have
access to appropriate (currently not well defined) Tango objects related to
the rover and a flexible number of slots for Tango objects for suit-related
interfaces, like receiving microphone audio and/or updates about the state of
the astronaut - useful for modeling utterance context.

Externally, a kim instance Tango object should have exposed methods for the
rover planner/executive to call for the purpose of deciding what feedback to
send to the user.

Performance Requirements
========================

Lag-to-Feedback (s)
-------------------
Time from end of speaker utterance to onset of voice agent feedback
transmission. A user ought to receive some feedback within no more than a few
seconds for particularly complex commands or noisy input; feedback time for
basic, short commands in typical conditions ought to be less than that.
Testing will firm up whether these performance times are too generous or
stringent.

Word recognition error rate on actual rover-directed speech (%)
---------------------------------------------------------------
A reasonable goal, based on consultation of a review of early/mid-2000s NASA
technology and field tests ([1]), is for around ~6.5% of actual rover-directed
words to be incorrectly recognized. A possible catch here is that the
:term:`IMS`/:term:`OeWF` volunteers may have varying types and degrees of
accents.

False accept rate (attending to non-rover-directed speech)
----------------------------------------------------------
'False accepts' occur when a rover voice agent misclassifies an utterance as a
request directed at it. A reasonable goal based on consultation of [1] is for
<10% of all utterances to be incorrectly classified by the rover voice agent
as directed at the rover.

An easy fix for this to start with is a prefix-keyword (think Star Trek's
"Computer, ..." - prefixing every command with the name of the rover);
depending on how annoying this is, a separate classifier can be trained later
to classify incoming utterances.

False reject rate (ignoring rover-directed speech)
--------------------------------------------------
'False rejects' occur when a rover voice agent misclassifies an utterance as
NOT directed at it. A reasonable goal based on consultation of [1] is for <10%
of all utterances to be incorrectly classified by the intended rover voice
agent as directed to someone else.

Development and Test Factors
============================

Standards Compliance
--------------------

All code will adhere to the guidelines outlined in the ERAS `Software
Engineering Practices Guidelines
<http://eras.readthedocs.org/en/latest/doc/guidelines.html/>`_

In addition, a kim instance ought to be able to support receiving audio in a
number of well-supported, non-proprietary audio formats - WAV, AAC, Ogg
vorbis.

Software validation and verification
------------------------------------

The kim library code will be unit-tested, behaviorally tested by cases, using
speech recorded on inexpensive consumer-model laptop microphones, possibly
tested in simulation (provided a simulation exists at some point), and later
field-tested by :term:`IMS`/:term:`OeWF` volunteers.

Planning
--------

The minimum schedule can be found in [3]. Voice library-salient minimum
milestones are below.

* June 27: First Draft of Design Study finished. Coding begins, moving through
  use cases with repository updates at least every two weeks.
* July 29: Design Study Review completed: Design Study doc frozen on
  repository, server prototype up and running in Tango.
* Aug 2: Mid-term evaluation.
* Aug 15: "GSoC on Mars" paper and presentation for 2013 Mars Society
  convention in Boulder ready.
* Sep 16: Final server version up and running, all validation tests OK with
  satisfactory coverage.
* Sep 23: User/Maintenance Manual frozen.
* Sep 27: Final evaluation.
* Oct 2013: Project integration on Bergamo C3 prototype.
* Within 2013?: Field testing with :term:`OeWF`.


The preferred schedule, intended to provide some slack for unanticipated
difficulties, is below.


Use-Case Models
===============

Use Case: Important features common to all use cases
----------------------------------------------------
Actors
~~~~~~
One or more astronauts/:term:`IMS` or :term:`OeWF` volunteers conducting
(mock) :term:`EVA` and using rovers to assist them.

Contextual Goals
~~~~~~~~~~~~~~~~
Direct the operation of the rover using naturalistic voice commands.

Priority
--------
Critical.

Preconditions
-------------
The kim instance needs a functioning audio stream input.

Course
------------
1. Audio is transmitted from the astronaut(s) to the server hosting the voice
   interface agent.
2. Language in the audio is classified as rover-directed or not.
3. Rover-directed speech is mapped onto words (the mapping mechanism is
   deliberately underspecified)
4. The kim instance decides what to do with the utterance and therefore what
   kind of feedback to give the user:

   1. Utterances the kim instance is confident it understood:

       1. The utterance is mapped onto an action request.
       2. The action request gets passed on to the rover executive (planning
          agent).
       3. The rover executive then passes on to the kim instance whether the
          request will be executed, if there's a conflict and the kim instance
          should ask for confirmation/clarification, or if the request cannot
          be completed.
       4. Whatever action the rover planning agent takes, the kim instance
          then decides appropriate feedback to pass onto the user.

            * If the request will be straightforwardly granted, a short
              restatement including parameters (e.g. distance to move or
              rotate, destination) will be forwarded by the kim instance to
              the astronauts on :term:`EVA`.
                * Alternately, to cut down on useless chatter, if there is
                  some kind of :term:`HUD` indicator of what each rover on
                  :term:`EVA` is doing (i.e. a short status summary), updating
                  this could be a better alternative than :term:`HUD` text or
                  synthesized speech.
            * If there's a conflict, the kim instance should pass on a message
              (via text-in-:term:`HUD` or via synthesized speech) as to what
              conflicts with the request (e.g. "CONFLICT: Travel to <name-of-
              requested- destination> conflicts with existing goal <goal
              id/description>.") and ask for confirmation of the request (e.g.
              "CONFIRM?: Travel to <destination-name>.")
            * If the request cannot be complied with (due to precondition
              violation distinct from a goal conflict), the kim agent should
              pass along a message explaining as much: "REQUEST DENIED:
              <explanation - precondition xyz violated.>"
            * If the request was only partially understood or understood with
              confidence less than a to-be-experimentally-determined
              threshold, then the kim instance should request clarification of
              the remaining parameters while clarifying what it already
              understands. For example, "Travel where?" "Move forward how
              far?" "Track what?" "Follow who?"

   2. Utterances the kim instance is NOT confident it understood:
       1. The kim instance requests clarification a limited number of times:

            * successful clarification puts the kim instance back at 4.1
              above.
            * before returning to a state where it waits for a new command or
              until the user decides to break the clarification dialog loop
              (e.g. via "No more questions.", "Start over.", "Shut up.")


Postconditions
--------------
The rover passes on the request as understood to the rover's planning agent,
waits for feedback from the planning agent, and passes it along to the user.

Notes
-----
Note that none of the trigger utterance example lists are intended to be
exhaustive.

Use Case: Directing rover movement
==================================
Priority
--------
Critical

Preconditions
-------------
The rover must be capable of the requested movement and the requested movement
should not conflict with other current or near-future goals.

Examples of naturalistic and realistic trigger utterances
---------------------------------------------------------

Examples of less definite duration, goal directed instructions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* “<Head/go> (back) to(wards) <named-landmark, the-habitat, the-next-waypoint,
  astronaut’s-name, other-rover’s-name>.”
* “Come here.”
* “Follow me (closely, exactly).”

Notably, the grammar template for this sort of command consists of some
movement word, a target phrase (possibly including prepositions or adverbs),
and optional arguments indicating the manner in which the rover should pursue
movement towards the target.

Examples of definite, direct instructions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* "Rotate <left, right> <# degrees>."
* "Go <forward, backward> <a certain number of meters or centimeters>."
* "Don't move."
* "Stop."
* "Halt."

Postconditions
--------------
An appropriate movement request is made to the rover planner and appropriate
feedback reaches the user.

Use Case: Image recording
=========================
Actors
------
Human user making an action request, rover voice interaction agent, and
(potentially) a target.

Priority
--------
Normal

Preconditions
-------------
The webcam must be operational and the requested use of it should not conflict
with other current or future goals.

Examples of realistic, naturalistic trigger utterances
------------------------------------------------------
(NB that almost all of these are of a goal-directed nature.)

* “<Watch/Record> <named-entity> (for-some-duration)."
* “Take a <photo, panorama, video, capture> of <named-entity> (and label it
  <name for photo/data capture>).”
* "End/Stop recording."
* "Delete the last <capture/image/recording>."

Postconditions
--------------
An appropriate webcam action request is passed on the planner and feedback
forwarded to the astronauts.

Use Case: Report details on rover state
=======================================

Actors
------
(Analogue) astronaut.

Priority
--------
Low.

Preconditions
-------------
The voice interface must be operational and able to get a response from the
rover executive.

Examples of trigger utterances
------------------------------
* "What's your current status, <rover name>?"
* "Run <name of diagnostic routine>."
* "What's the status of your <webcam, other rover-software-or-hardware-
  component>?"

Notes
-----
Anything more than a short list of simple requests is going to start
approaching menu-navigation - operating something like a console, all by
voice. The scope and feasability of this is only determinable via testing (in
simulation or otherwise) - what would astronauts do to diagnose or repair a
rover if one too heavy to drag back to safety breaks in the field?

Postconditions
--------------
A more or less detailed message of what is or isn't OK with the requested item
is sent via text to the astronaut's :term:`HUD` or via voice-synthesis to
the current common voice channel.

Notes
=====

.. notes can be handled automatically by Sphinx

