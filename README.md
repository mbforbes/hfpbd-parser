# hfpbd-parser

[![Build Status](https://travis-ci.org/mbforbes/hfpbd-parser.svg?branch=master)](https://travis-ci.org/mbforbes/hfpbd-parser)
[![Coverage Status](https://img.shields.io/coveralls/mbforbes/hfpbd-parser.svg)](https://coveralls.io/r/mbforbes/hfpbd-parser?branch=master)
[![license All Rights Reserved](http://b.repl.ca/v1/license-All%20Rights%20Reserved-red.png)](
https://github.com/mbforbes/hfpbd-parser/blob/master/LICENSE.txt)

The work in this repository is under publication review. **Please contact me for citations and usage.**

## Installation
You should have the root of this directory in your `PYTHONPATH` environment variable. In Unix-like systems, `cd` to the root of this directory after cloning and run
```bash
$ PYTHONPATH=`pwd`:$PYTHONPATH
```

or replace `pwd` with the actual path to the directory and stick that in your `.bashrc`.

## Running

### ROS
```bash
# Run the parser in basic ROS mode: listen to incoming language utterances and
# world updates, publish parses (commands) and groundings (object probabilities).
# Please see the ROSFrontend class in frontends.py for the topics and services.
$ python parser/core/frontends.py ros

# Run the parser in ROS mode with a web-interface. This displays robot and world
# state, allows input of language for commands and grounding, and also shows
# debug output for analysis. Note that there isn't AJAX, so you won't see state
# updates until the page loads again, e.g. for a query.
$ python parser/web/web_interface.py
```

### Web interface (without ROS)
```bash
# This will load a "default world" (specified in parser/data/world_default.yml)
# and allow you to hit it with parses and grounding requests.
$ python parser/web/web_interface.py noros
```

### Command line interface (without ROS)
```
# Run the parser with no robot or wold state. Useful for testing non-relative
# commands and sanity checking. Runs an interactive loop that parses typed
# input.
$ python parser/core/frontends.py interactive-simple

# Run the parser with the default static robot and world state (defined in
# parser/data/world_default.yml). Runs an interactive loop that parses typed
# input.
$ python parser/core/frontends.py interactive

# Generate exhaustive list of sentences. This separately generates commands from
# object-referring phrases, as generating both together causes a combinatorial
# explosion (the result is already quite large). This can be input to tools that
# than generate language models for speech recognizers.
$ python parser/core/frontends.py sentences

# Sets the default world (defined in parser/data/world_default.yml) and runs an
# interactive loop that takes input an object-referring phrase and returns, for
# each object, the probability it refers to that object.
$ python parser/core/frontends.py ground
```

## Grammar
The grammar is defined in `parser/data/commands.yml`. Please contact me if you'd like more information.

## Language input
A general note for the language input: speech is supported, but we have not fine-tuned the system to be demo-able with it.

Here are the steps we've taken so far:

- We've combined certain word pairs like `left-hand` and `in-front-of` in the grammar for better speech recognition (notably at the expense of typing input)
- We've implemented a dedicated mode in the parser that will generate a full corpus of training sentences that can be used to generate settings files for speech recognition programs like Pocketsphinx
- We've tested using standard speech input and the system functions

With that said, we have not yet worked to make the system demo-able and rebust enough to handle naive users. While we use a vastly restricted vocabulary, its domain is large enough, and its impact on the system great enough (i.e. complete control), that getting reliable speech input without a dialog manager is a nontrivial task.

## Questions
This guide omits a great number of implementation deals vital to understanding how the system works (for example, where do robot state updates happen? Are they inferred in the parser, or do they come from the robot? Answer: robot). Please do not hesitate to contact me or open an issue for clarification.
