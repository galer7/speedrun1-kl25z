### src folder:

## Current workflow:
  - race_functions.cpp contains the general, atomic functions used for all the race programs.
  - race_program.cpp contains all the race programs:
    * timed race == main race
    * speed limit - lower the speed when encountering a pattern on the race track, than go back to the original speed on the 2nd encounter
    * object avoidance - stop when encountering the cube, without the car crashing into it
    * eight race circuit - as many laps as possible