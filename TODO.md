# TODO

## Original list
- [HIGH] Better language scoring function:
	- NOTE: Step 1 for this is to write breaking tests to be specific about what cases I'm trying to fix.
	- Adjective skipping should be free
	- Should be penalty for other word skipping
	- Should be larger penalty for options whose parameters are matched with another option

- [HIGH] ROS service to resolve a referring phrase.

- [HIGH] A 'turn hand' (or something) command to have hand face a certain direction.

- [HIGH] Movement parameters (e.g. a little).

- [HIGH] Record objects command.

- [HIGH] Defaults for:
	- Movement parameters
	- Rotation direction (CW vs CCW)

- [HIGH] Generate most-likely phrases for all objects in scene (can just use similar machinery as ROS referring phrase service?? No, maybe this is just enumerating all object unique properties (easy, already done in phrase generation for RobotCommand) and deciding which to say (can check out Mike's referred work, may have info on how to do this)).

- [MEDIUM] Be able to quit `hfpw` without a core dump.

- [MEDIUM] Both hands. Probably not for all commands? Think through robot execution first!

- [MEDIUM] More (command) scoring hints:
	- Add action info to robot state.
	- Switch, execute unlikely when no actions
	- Switch unlikely only one action
	- Stop already unlikely when not executing, right?

- [LOW] Color log output (Info green, Error red, Debug white, e.g.)

- [LOW] AJAX for web interface to show ROS-received updates immediately.

- [LOW] Time parsing, have avg time (& num parses) displayed after tests. Maybe breakdown by:
	- scoring sentences with utterance
	- scoring commands
	- sorting to rank / selecting

- [LOW] Time cmd/sentence generation as well. Maybe breakdown for
	- phrases
	- options
	- parameters
	- command templates
	- commands
	- sentences
	- apply w
	- apply r

- [LOW] Refactor all strings in hybridbayes (+ elsewhere now?).

- [VERY LOW] The commads.yml file should generate all ROS messages. This will be somewhat of a major change and would be a huge amount of work just for maintainability of the overall system, so shouldn't be undertaken while there are still important things to do.

- [VERY LOW] Make tests more maintainable. Don't want to go so far as to auto-generate them, but right now the verbosity is just killing me.
