- [HIGH]: This breaks, and shouldn't:

```
world objects
color: red
  has_distinct_color: false
  has_distinct_type: false
  is_above_reachable: [false, true]
  is_biggest: false
  is_farthest: false
  is_leftmost: false
  is_nearest: false
  is_nextto_reachable: [true, false]
  is_pickupable: [true, false]
  is_righttmost: false
  is_smallest: false
  name: obj0
  type: box


robot state:
can_move_down: [false, false]
can_move_toleft: [true, true]
can_move_toright: [false, true]
can_move_up: [true, true]
gripper_states: [closed_empty, closed_empty]
last_cmd_side: left_hand

input: move right hand up
response: move_abs: left_hand, up
```

- [MEDIUM] Refactor all strings in hybridbayes
- [LOW] Restructure project by
	- separating hybridbayes, web, and test into higher-level modules
	- refactoring some classes within hybridbayes into other modules
