- [HIGH] Clarifying should check whether ALL top commands match the same
template, anod only clarify arguments if so. Otherwise, it needs to
clarify the command.

- [HIGH] More scoring hints:
	- Add action info to robot state.
	- Switch, execute unlikely when no actions
	- Stop already unlikely when not executing, right?

- [MEDIUM] Refactor all strings in hybridbayes

- [LOW] Color log output (Info green, Error red, Debug white, e.g.)

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
