
# Motion planning (_correction_mode_ == auto) using a path selection policy
Motion planning is complex because multiple paths are possible and we are looking for the most 
_graceful_ movement (centralised head). However doing path switching movements to recover from 
an _ugly_ pose to a _graceful_ may introduce corrective movements that fight ad-hoc motion 
requests.

So a path selection policy is needed. 

This has been difficult to get right, so the solver has a few to tune and compare, 

    just ik-test --gate   # use a specific policy
    just ik-bench         # test all policies

Common to all of them:
  - The candidates are every pose in the table that gives the requested pitch with roll within the _graceful_ 
    allowance, yaw by driving all sleds evenly on top. Each candidate has a sled movement and a dest_offset.
  - *"shortest sled movements"* is the sum of all sled movements. In this way even a request for a yaw change 
    could be used to change path to a more graceful pose, as opposed to the simple drive all sleds evenly.
  - *"use the best"* of the candidates a gate lets in: the shortest sled movements, or the most central (_best_). 
    I will have to run an experiment using the most central solution instead.
  - The _corrected_ flag is set when the pick isn't the "shortest sled movements" pose. The caller resets _time_ to zero.
  - _none_ and _force_ are the other two correction modes, here for reference as they are the two ends of the trade off.

What each policy reads:

| policy      | gain       | gamma         | margin | lam        | penalty | time |
|-------------|------------|---------------|--------|------------|---------|------|
| none        |            |               |        |            |         |      |
| force       |            |               |        |            |         |      |
| first_blush | ramp slope | ramp length s |        |            |         | yes  |
| margin      |            | dwell s       | mm     |            |         | yes  |
| deadband    |            | dwell s       | mm     |            |         | yes  |
| cost        |            |               |        | deg per mm | deg     |      |

## none
Minimum joint displacement, the usual IK redundancy resolution: the candidate with the "shortest sled movements", 
nothing else considered. Fast and predictable, and what every gate falls back to.

What the simulator showed: this drifts off centre, even on a level sweep. Within the _graceful_ roll allowance 
a differential sled move yaws the head for less travel than driving all sleds evenly, so that gets picked every 
tick and nothing pulls the head back.

## force
The secondary objective alone: the most central candidate (least dest_offset), movement ignored, ties to the 
shortest. Meant for when the robot is idle: re-issue the current roll/pitch/yaw with _force_ and it moves to the 
"most central" version of its posture. Used continuously it keeps the head central but hops between paths.

## first_blush
My first idea: a time variable _gamma_ acting as a threshold for corrections. A relative threshold (a ratio of the 
current offset) with a refractory period, the threshold ramping open after each switch.

If multiple poses give a requested roll/pitch, the policy is as follows:
  - if all destination solutions are *less* central than current pos
    - use pose requiring the "shortest sled movements"
  - if any destination solution is *more* central than current pos
    - calculate _kappa_ as _gain_ * min(_time_, _gamma_)
    - find the set where dest_offset < _kappa_ * current_head_offset
      - straight after a correction _kappa_ is zero so nothing gets in, it opens up over _gamma_ seconds
      - if that's not empty "use the best"
        - if that isn't the "shortest sled movements" pose, a course correction has taken effect, so _time_ resets to zero.
      - if it's empty, use the pose requiring the "shortest sled movements"

Notes:
  - _gain_ * _gamma_ is the fraction of the current offset a correction has to beat once the gate is fully open, so it should be a little bit under 1.
  - current_head_offset is where the head is now (the sleds being commanded), not where the shortest pose would take it.

tldr; _gain_ controls how eagerly it jumps based on distance, and _gamma_ controls how often it will try.

What the simulator showed: it corrects, the drift (see _none_) undoes the correction within a second, and the 
gate reopens and corrects again. Five corrections a second and the head still ended up off centre.

## margin
Dwell-time switching plus hysteresis switching logic, the two standard switching rules from supervisory control. 
After a dwell of _gamma_ seconds, take a destination that beats the "shortest sled movements" pose by _margin_ mm 
(a real gain, not a ratio); within the dwell, or if nothing beats it by that much, use the shortest.

The comparison is against where the shortest pose would take the head, not where the head is now, so a candidate 
has to be better than the alternative actually on offer.

What the simulator showed: same as first_blush, fewer fidgets. Dwell 1 s, margin 3 mm made no difference to time 
_ugly_ at all; the corrections were undone by the drift.

## deadband
margin with a dead band (dead zone) on the offset: no correction at all while the "shortest sled movements" pose 
keeps the head _graceful_ (within 10 mm). Only once the shortest pose is _ugly_ does the margin rule apply.

The idea was to stop corrections that nobody would notice. It did that, and was otherwise the same as margin.

## cost
The default. Greedy one-step minimisation of a weighted sum, with a switching cost. No time variable: every 
destination is scored as

    sled movement + _lam_ * dest_offset (+ _penalty_ if it isn't the "shortest sled movements" pose)

and the lowest wins. Offset is weighed on every request, so the drift never gets going: a differential move that 
buys a bit of sled travel at the cost of centring doesn't score.

  - _lam_ is how many degrees of sled travel a mm of offset is worth (the weight between the two objectives).
  - _penalty_ is hysteresis in cost space, so it doesn't flip between near equal poses.
  - _corrected_ is set whenever the pick isn't the shortest, which with this policy is most of the time; count hops 
    (the answer moving more than 2 deg of sled in one tick) to measure fidgeting, not corrections.

Tuned on the bench: _lam_ 3, _penalty_ 1 halves the time _ugly_ at no cost to tracking; _lam_ 5 keeps the head 
within 10 mm throughout but tracking suffers.

TLDR; _lam_ controls how much it cares about centring over movement, and _penalty_ controls how reluctant it is to switch.

## bench results
`just ik-bench`, 16 s runs of figure8, sweep, loop, box and step on both poles. Time _ugly_ and off centre are 
means over the ten runs, hops the total. Gaze error is figure8's mean and max, in degrees.

| policy                    | ugly % | of that, off > 10 mm | off mm | hops | figure8 gaze |
|---------------------------|-------:|---------------------:|-------:|-----:|-------------:|
| none                      |     41 |                   34 |    6.8 |   28 |   0.33 / 6.5 |
| force                     |     23 |                    0 |    1.6 |   57 |   0.70 / 8.7 |
| first_blush gain .45, 2 s |     32 |                   24 |    5.3 |  846 |   0.51 / 6.5 |
| margin 1 s, 3 mm          |     41 |                   33 |    6.7 |  222 |   0.34 / 6.5 |
| deadband 1 s, 3 mm        |     40 |                   33 |    6.7 |  156 |   0.33 / 6.5 |
| cost lam 3, penalty 1     |     20 |                   10 |    3.7 |  129 |   0.34 / 6.5 |
| cost lam 5, penalty 1     |     11 |                    0 |    1.8 |   52 |  0.77 / 10.1 |

Gates that fire on events (the three with a clock) can't fight a drift that recurs every tick. The _ugly_ time 
that remains with cost is roll in transit: the sleds lag the solver and the head passes through poses nobody chose.

## Default (best policy)
_cost_ with _lam_ 3 and _penalty_ 1 is the default, in the solver and in `just ik-test`. It was the only policy that 
held the head central against the drift, and at those settings it tracks as well as _none_ while halving the time 
_ugly_.

Note:  use _lam_ 5 if being central matters more than minor hops and a little tracking error.

