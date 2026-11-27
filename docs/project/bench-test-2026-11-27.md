# All fifteen servos on the battery (2026-11-27)

The first time all fifteen SCS0009 servos were on the bus together, loose on the bench with nothing mounted on
them: an UNO Q and a Mini560 regulator (5 V) fed from a 3S LiPo through one toggle switch, the regulator's output
on thin jumper wires to the servo adapter, the servos on hub boards. Every run was recorded and is in the trainnr
project `microduck-nano`; the stamps below name the recordings.

## What was found

- **Every servo is good.** All fifteen answer under their own IDs and move. Six that were silent at first were
  on a hub port that carried nothing; moved to the other port, each answered at once.
- **The 5 V supply cannot run fifteen servos.** Unloaded, moving ±10° slowly, the lowest voltage any servo read
  fell from 5.0 V (one servo) to 4.4 V (fifteen). Stepping together, the supply holds four or five servos.
- **A long daisy chain loses a volt by itself.** With one leg on one chain, the servo at the far end read 4.0 V
  while servos on other cables still read 5.0 V. Giving the hip servos their own cables to the hub brought every
  servo of a leg to within 0.3 V of the others.
- **The camera gets no power on battery.** With the board fed from VIN, its USB-C port supplies no 5 V
  (see [build-and-flash.md](../robot/build-and-flash.md)).

## New for these tests

`duckd bench-set` and the firmware's `duck_benchset_*` calls: several joints armed together, all or none, each
kept inside its own limits and speed limit, all switched off when any one faults or the Linux side stops talking
(host test `test_bench_set_moves_together_and_stops_together`). `--excite steps` makes them all jump from one end
to the other at the same instant, which is the most they can ask of the supply at once without a load.

## Slow sine, ±10° unless noted, 0.5 Hz, 8 s

| Servos | Lowest voltage | Highest load | Recording |
|---|---|---|---|
| 9 (the first nine to answer), ±4° | 4.6 V | | `nine-servos-together-4deg-2026-11-27@58174d71c6f6` |
| 9 | 4.5 V | 63 % | `nine-servos-together-10deg-camera-2026-11-27@34cb3c7f0d73` |
| 6 | 4.6 V | 22.5 % | `six-4-7-8-9-13-14-together-2026-11-27@5a4917213c71` |
| 15, ±4° | 4.5 V | 22.5 % | `all-fifteen-together-4deg-2026-11-27@192a4281f5c5` |
| 15 | 4.4 V | 24 % | `all-fifteen-together-10deg-2026-11-27@857039e4f3af` |

## All stepping together, ±10°, there and back every second, 6 s

The firmware's under-voltage limit was 4.3 V (the repository's value). "Off" means the firmware saw a servo below
the limit and switched every servo off.

| Servos | Lowest voltage | Result | Recording |
|---|---|---|---|
| 1 (left knee) | 4.8 V | ran | `steps-10deg-1-servos-2026-11-27@dbf4732d8015` |
| 1 (left hip yaw, end of a chain) | 4.5 V | ran | `steps-10deg-left-hip-yaw-alone-2026-11-27@33665804b319` |
| 3 | 4.6 V | ran | `steps-10deg-3-servos-2026-11-27@7e2140edbce1` |
| 5, without left hip yaw | 4.3 V | ran | `steps-10deg-5-without-left-hip-yaw-2026-11-27@14caff6de329` |
| 5 (left leg) | 4.2 V | off in 0.1 s | `steps-10deg-5-servos-2026-11-27@2bcaf2aef72e` |
| 10 (both legs) | 4.0 V | off in 0.1 s | `steps-10deg-10-servos-2026-11-27@ba1519136e3a` |
| 15 | 3.7 V | off in 0.1 s | `all-fifteen-steps-10deg-period1-2026-11-27@dc6b34fd13de` |

With the limit lowered to 4.05 V on the test board only (the servo's own cut-out is 4 V), and the hip servos then
moved off the long chains one after another:

| Servos | One chain per leg | Left hip yaw on its own cable | Both hip rolls too |
|---|---|---|---|
| Left leg, 5 | off after 1.1 s, 4.0 V | ran, 4.1 V | ran, 4.3 V |
| Right leg, 5 | | | ran, 4.3 V |
| Both legs, 10 | off in 0.1 s, 4.0 V | off after 0.6 s, 3.9 V | off after 2.1 s, 3.9 V |
| All 15 | off in 0.1 s, 4.0 V | off in 0.1 s, 4.0 V | off in 0.1 s, 3.7 V |

(Recordings of the first column: `steps-10deg-5-servos-cutoff4.05-2026-11-27@69d47ac2b924`,
`steps-10deg-15-servos-cutoff4.05-2026-11-27@f033ab62f40c`. The later columns are on the test board only.)

With ten or more servos the low voltage is spread over every cable, so what is left is the supply itself: the
regulator (about 3 A before it runs hot) and its output wires. Fifteen servos at their rated load draw about 6 A
([scs0009.md](scs0009.md)).

## What follows

- Replace the regulator with one rated 8 A or more continuous, set to 6.0 V, on thick short wire to the servo
  adapter; then repeat the stepping table. The aim is fifteen servos completing the 6 s.
- Wire each leg as two or three short cables from the hub, never one chain.
- Put a fuse in the battery's positive wire. There is none.
- No weight-bearing test was tried: the supply fails before any load is on the servos.

## Not explained

- **The firmware refused to arm a set on the first attempt** ("fault latched") in most runs with five or more
  servos, and armed on the second. It does not yet say which servo or which check refused. A guess, not checked:
  the servos are switched on one after another and the supply dips as each starts to hold.
- **The microcontroller stopped answering once**, between two runs, with the servos already off. Restarting the
  app brought it back, and it did not happen again in more than twenty runs.
- **One exchange in a few hundred is lost**, as on earlier days.
