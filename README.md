# sokoban-solver-01

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Klowor2002/sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Klowor2002/sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver. `main.ts` reads each level in `/input/<id>/level.txt` and writes the player's moves to `/output/<id>/solution.txt`. It peels the level's rooms off the hallway (a door is an articulation cell whose removal separates a balanced set of boxes and goals from the player), then solves every room push-optimally with an A* search over pushes: player-region canonicalisation, an admissible matching heuristic on precomputed push distances, static dead-square pruning, and freeze + 2x2 deadlock detection. A whole-level search stands by as a fallback. All budgets count expansions, never seconds, and no clock is read, so two executions write the same bytes. On the kit's practice set it solves all five instances in exactly the optimal push counts (13/16/29/47/60).

## Run it

```sh
node main.ts
```

---

<sub>Entered on [make.fun](https://make.fun/?ref=ebgtn94a&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
