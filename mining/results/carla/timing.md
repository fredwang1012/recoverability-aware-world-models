## CARLA timing (this machine, headless, no sensors)

- empty tick: 0.7 ms
- one rollout (corridor, brake_hold, 166 ticks incl. warm-up/spawn): 1.09 s
- one full record (3 policies x 1 world): 2.01 s
- audit batch estimate (864 policy/world runs): 15.7 min on one server
