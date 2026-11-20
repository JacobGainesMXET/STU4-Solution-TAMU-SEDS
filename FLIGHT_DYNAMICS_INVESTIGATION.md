# Escape from Phobos flight dynamics and score investigation

## Clear-flight launch profile now in the team controller

Both final and no-debris configurations load the same team `Controller.py`. The previous flight sources are preserved at `5ea242d`. The investigation below this section describes that baseline. The new measurements are offline flights from the final launch pose using local gravity and the installed jet allocation equations.

The target starts about 94° from the shuttle nose. The controller lifts toward the target in world coordinates. Once closing speed exceeds 0.1 m/s and the predicted launch corridor has two clearance margins to spare, guidance allows up to 11°/s of pitch at large errors. It tapers the rate as the nose lines up and retains the prior angular braking limit. A threat can interrupt that turn and trigger spin damping. At the 20 s handoff the route planner can evaluate the onward path while the shuttle settles. A marginal burn or unresolved crossing can justify a short coast only if the coast is also safe. The existing score comparison chooses the end of the main burn, and target entry requires no braking.

Both simulation configurations request 60 Hz for the controller. The earlier rigid-body bench used 20 Hz for controller updates and 60 Hz for jet allocation. The bench now reads the configured controller rate when using final gravity. Each frequency below uses the same initial pose, physics, radar schedule and seeds for the two policies.

| Matched condition at the configured 60 Hz | Previous time, fuel, score | New time, fuel, score |
|---|---|---|
| Empty field | 198.15 s, 43.19 kg, 52.98 | 198.70 s, 41.00 kg, 55.73 |
| Distant radar returns with clear corridor | 199.30 s, 43.01 kg, 52.87 | 198.70 s, 41.00 kg, 55.73 |
| Same corridor with delayed radar | 199.73 s, 43.21 kg, 52.46 | 199.88 s, 41.03 kg, 55.33 |
| Mean of 15 paired debris flights with contact-free arrivals | 216.33 s, 47.15 kg, 43.22 | 212.96 s, 46.08 kg, 45.65 |

At 60 Hz, fuel at 20 s fell from 13.34 to 10.92 kg and pointing error fell from 15.8° to 0.7°. The shuttle reached 3 m/s at 27.40 s versus 30.88 s. A 10.5°/s maximum slew earned 55.94 in the empty field but intersected an asteroid in offset-crossing seed 13. The selected 11°/s profile cleared that geometry. Both the selected profile and the baseline made contact in the two separate late-detection launch cases at both frequencies. At 60 Hz the smallest positive gap was 0.90 m with the new controller and 0.96 m with the baseline. All **169** automated tests passed.

| Matched condition at 20 Hz | Previous time, fuel, score | New time, fuel, score |
|---|---|---|
| Empty field | 198.80 s, 43.10 kg, 52.89 | 199.45 s, 40.83 kg, 55.72 |
| Distant radar returns with clear corridor | 199.75 s, 42.96 kg, 52.80 | 199.45 s, 40.83 kg, 55.72 |
| Same corridor with delayed radar | 199.20 s, 43.36 kg, 52.43 | 199.75 s, 41.05 kg, 55.35 |
| Mean of 15 paired debris flights with contact-free arrivals | 215.96 s, 47.30 kg, 43.10 | 212.56 s, 46.10 kg, 45.76 |

In the empty field, fuel at 20 s fell from 13.32 to 10.90 kg, pointing error fell from 15.9° to 0.5°, and time to 3 m/s improved from 31.00 to 27.40 s. Peak launch spin rose from 4.99 to 10.44°/s. The score gain came mainly from avoiding prolonged simultaneous turning and broadside thrust. Fixed cutoffs, intermediate thrust axes, split burns and faster slews were evaluated. A candidate with stronger angular braking reached 55.92 but made contact in crossing seed 67. Early coasting and stronger braking gave no consistent advantage across cases.

The 15 debris pairs used head-on seeds 13, 17, 67 and 101, crossings 13, 29, 67 and 101, offset crossings 13, 43, 67 and 101, and launch crossings 13, 67 and 101. The results include regressions. Offset crossing 101 fell from 40.15 to 31.70 points after the shuttle spent more propellant. The smallest positive modeled clearance was 0.91 m. In two **additional** launch-crossing cases (seeds 13 and 67), the near contact first became observable after approximately ten seconds. Both controllers intersected the synthetic asteroid in both cases. Testing wider outward tilts up to 75° failed to clear both encounters. The rigid-body bench has no contact or damage response, so these two pairs have no defensible competition score.

Reproduce from the team checkout using the PowerShell interpreter and package paths listed below. The final-gravity bench now reads the controller frequency from the final configuration. Add `--controller-hz 20` to reproduce the 20 Hz comparisons here and in the earlier investigation. Use a separate process for each controller. `--controller-dir` selects a directory containing the four matching flight modules.

```powershell
& $py -B -m unittest discover -s tests
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages --scene clear-corridor
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages --scene crossings --seed 67 --delayed-radar
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages --scene late-launch-crossing --seed 13 --delayed-radar
```

**Space Teams validation.** Compare the official no-debris score with the previous two runs near 51.2, and inspect 20 s fuel, pointing error, spin and pad clearance. Then use the same controller in the final scenario and record every outcome, including damage and losses. The bench omits the launchpad and the full competition debris field. Establishing the live score gain requires simulator runs. The search covered a finite set of profiles.

## Earlier baseline investigation

**Submission decision, November 20, 2026.** Keep the regular team controller on `burn-coast-burn` at `5ea242d`. Its 12 official arrivals averaged **43.15 points**, 248.34 s and 40.87 kg. The best was 50.0. There was one additional damaged flight with no official result. The strongest clear-flight result from the real launch pose in the tested main-burn sweep was **53.07**, only **0.18 point** above the current controller. The clear-to-debris penalty in the same offline plant was **11.37 points**. This makes safe passage through encounters the first place to look for a material gain.

## Evidence and scoring

I matched the three recorded flight-source hashes to the team checkout, each diagnostics ID to its Space Teams controller log, and each result to its official scorer log. All 13 archived scenario files were byte-identical to the current final config. The scorer ran at 10 Hz. The values below are its terminal outputs. The controller's roughly one-second telemetry provides the trajectory evidence. [Final simulation](../STU4-Sim/Sims/FINAL_SUBMISSION.simconf) · [Installed scoring code](../STU4-Sim/Systems/Scoring.py) · [Official scoring description](https://github.com/SimDynamX/STU4-Solution/wiki/2.-mission-success,-failure,-and-scoring)

| Recording UTC | Score | Time (s) | Fuel (kg) | Damage (J) |
|---|---:|---:|---:|---:|
| 035118 | 42.9 | 258.85 | 40.06 | 0 |
| 035405 | 50.0 | 236.83 | 38.00 | 0 |
| 040031 | 45.0 | 280.13 | 35.89 | 0 |
| 040439 | Unknown | — | — | 6,793.58 at last sample |
| 040808 | 45.8 | 250.65 | 39.07 | 0 |
| 041616 | 43.5 | 267.44 | 38.54 | 0 |
| 054215 | 48.3 | 219.96 | 42.18 | 0 |
| 084105 | 42.1 | 260.33 | 40.46 | 0 |
| 084357 | 27.2 | 213.93 | 50.62 | 11,901.77 |
| 084601 | 32.3 | 257.31 | 48.25 | 0 |
| 084843 | 47.3 | 254.84 | 37.32 | 0 |
| 085117 | 49.2 | 234.05 | 39.08 | 0 |
| 085345 | 44.2 | 245.78 | 40.98 | 0 |

The 11 clean debris arrivals averaged 44.60. Flight 040439 ended its recording at 364.93 s with 445.52 m remaining. An all-attempt mean requires its official outcome. The controlled comparison below uses an offline plant. Two subsequent clean trials in Space Teams provide a separate calibration.

### Live debris-free calibration

The two archived no-debris configurations match each other and the open test configuration. Compared with the final scenario, the shuttle, cycler, gravity fields, epoch, jet actuator and scoring system match. The test removes 500 rocks, disables radar and spawning, and enables the controller's known-empty mode. Both recordings match the flown controller's three source hashes. [No-debris scenario](../STU4-Sim/Sims/COMPETITION_SIM_NO_DEBRIS.simconf)

| Recording UTC | Official score | Time (s) | Fuel (kg) | Damage (J) | First clear coast (s) | Closing speed at cutoff (m/s) |
|---|---:|---:|---:|---:|---:|---:|
| 101921 | 51.2 | 206.98 | 42.58 | 0 | 69.07 | 8.17 |
| 102148 | 51.1 | 210.46 | 41.97 | 0 | 68.45 | 7.99 |

Both reached the sphere with fuel flat from the first clear coast through arrival. Neither required a trim. Starting three seconds into those coasts, frozen-gravity forecasts of the 50 m crossing were within 0.28 s and 0.76 s of the official times. The full-field forecasts were within 0.09 s and 0.01 s. Differences this small approach the scorer's 10 Hz resolution. Repricing forward thrust from the first recorded clear coast gave another two seconds a score change of −0.06 and −0.03 points. Eight more seconds would save roughly 21–22 s and spend 5.50–5.74 kg, lowering the predicted score by 0.73–0.91 point. These are counterfactuals using the logged gravity and learned burn costs. [Burn repricing](tests/price_coast.py) · [First clear recording](diagnostics/20261120T101921_048168Z) · [Second clear recording](diagnostics/20261120T102148_073741Z)

The observed average is **51.15 points, 208.72 s, and 42.28 kg**. The current-policy offline clear case was 9.92 s faster and used about 0.82 kg more, scoring 1.74 points higher. The clear trials support the adaptive cutoff near 8 m/s under these conditions. They also set a measured comparison point 6.55 points above the clean debris arrivals. That gap includes necessary avoidance and differences in radar availability and scheduling.

### Six follow-up flights with debris

All six started after the clear trials, used the regular controller's same three source hashes, and ran with a byte-identical copy of `FINAL_SUBMISSION`. Three reached the sphere without damage. Their average was **43.37 points, 260.22 s and 39.61 kg**. The official loss and the two damaged flights without an official result make the successful-flight mean an incomplete measure of reliability.

| Recording UTC | Outcome | Official score | Official time (s) | Official fuel (kg) | Recorded damage (J) |
|---|---|---:|---:|---:|---:|
| 102754 | Clean arrival | 40.6 | 245.16 | 43.75 | 0 |
| 103031 | Recording ended at 33.92 s, 1,272 m remaining | — | — | — | 39,632.87 |
| 103059 | Clean arrival | 46.6 | 264.95 | 36.48 | 0 |
| 103334 | Official loss | −682.1 | 219.76 | 40.19 | 687,771.16 |
| 104221 | Recording ended at 223.67 s, 392 m remaining | — | — | — | 16,516.44 |
| 104431 | Clean arrival | 42.9 | 270.55 | 38.59 | 0 |

The clean debris arrivals took about **52 s longer** than the two clear flights while using **2.67 kg less fuel**. At 60 s their closing speeds were 2.3–3.6 m/s, versus about 6.8 m/s in the two clear flights. This points to a departure and acceleration delay through the debris field, although any gain from releasing that delay depends on the available clearance.

#### Replay of the three new impacts

Each flight was in `COAST_LIMITED_CLEARANCE` in the samples preceding its first damage. In that mode the planner already recognizes danger, but coasting wins among the alternatives it evaluated despite a negative safety margin.

| Recording UTC | What telemetry showed before damage | What changed in offline replay |
|---|---|---|
| 103031 | A newly reported threat at 21.05 s aged over the next nine seconds. The forecast hull gap was negative by 23.24 s. Shuttle spin exceeded the default 3°/s post-launch burn gate until shortly before impact. | Removing the spin gate exposed small forward burns but left a negative predicted hull gap. Increasing the 3 s maneuver observation limit did not yield a viable alternative at the sampled states. |
| 103334 | The track driving the reported risk was visible at least 23 s before impact. Two sampled `VECTOR_ESCAPE` intervals were followed by coasting. A fresh update at 216.13 s reduced estimated hull gap from about 9.2 m to 2.7 m while the uncertainty margin remained negative. | Increasing the observation age limit or removing the spin gate did not select a different action in the sampled pre-impact window. |
| 104221 | Two brief escape attempts were followed by coasting. At 175.08 s a radar update reduced the apparent hull gap from near zero to −6.27 m. Damage followed at 180.48 s. | At 160.37 s, increasing only the per-maneuver age limit from 3 s to 8 s exposed a vector burn that raised modeled hull gap from −3.0 m to +3.6 m. Its uncertainty margin was still −14.9 m. Similar limited improvements appeared at 168.79 s and 170.84 s. |

The replay used original radar receipt order and interpolated recorded ownship states. Each snapshot used the existing guidance and avoidance predictor with the tested gates varied in isolation. The replay cannot follow the later consequences of an alternate burn or identify the object that actually hit the shuttle. Changing an age or spin threshold across all flights would require stronger evidence. The next candidate to evaluate is an earlier, bounded escape that can be revalidated across radar updates during persistent risky coasting. Compare it against all three impact cases and the clean flights. Retain the existing safety and actuation constraints while building that comparison. [Departure replay tools](tests/audit_departure.py) · [Launch impact](diagnostics/20261120T103031_611657Z) · [Official loss](diagnostics/20261120T103334_230806Z) · [In-flight impact](diagnostics/20261120T104221_434315Z)

#### Consistency at 46–48 points

The follow-up at 105436 officially scored **46.5** in **253.21 s** with **38.20 kg** and no damage. The previous attempt, 105213, recorded first damage at 48.97 s and has no terminal result. The older 47.3, 48.3, 49.2 and 50.0 arrivals match the same source hashes as the current controller. Matching the remaining telemetry against the saved comparison folders did not turn up an older controller with a recorded cohort averaging 46–48.

Six of 15 recorded clean debris arrivals reached at least 46. In the six higher-scoring runs, closing speed at the first sample after 60 s averaged **4.53 m/s** versus **2.79 m/s** in the other nine. By then they had closed about **45 m more range**. They averaged **7.7 s** in limited-clearance coast between 20 and 80 s versus **15.8 s** in the other nine. This groups completed flights by observed score. The timing differences show where to investigate and do not imply that the additional coast was unnecessary.

At the earlier three-clean-flight average of 260.22 s and 39.61 kg, a 46–48 score requires **246–236 s at the same fuel**, or **37.59–36.09 kg at the same time**, assuming zero damage. At 39.61 kg, 50 points requires about 227 s. The 105436 flight shows a second route to the range by combining a moderate transit time with lower fuel use. [Latest completed recording](diagnostics/20261120T105436_280004Z)

I tested two isolated changes in memory with the rigid-body bench and 8.6 s synthetic radar revisit. One offered an aligned, two-second forward burn earlier when coast was already clear, an uncertainty assessment of the entire swept burn passed, and radar was fresh. The other let strategic search continue during a limited-clearance tactical coast while keeping the tactical veto. The eight matched scenes used head-on seeds 7, 13 and 67, crossing seeds 13, 23 and 67, and offset-crossing seeds 13 and 67. Every case produced the same observed arrival time, fuel, and minimum gap across the three policies. All averaged **43.77**, and three of eight reached 46. These tests do not justify flight changes to either gate.

I retested +Z propulsion in the same six delayed-radar scenes described below, adding 1.5 m to the shuttle radius used by the predictor. All six reached the sphere. The simulated physical radius for measuring actual clearance stayed at 2.5 m.

| Policy | Mean score | Lowest score | Flights at 46 or higher | Minimum modeled hull gap |
|---|---:|---:|---:|---:|
| Current +X | 43.06 | 36.49 | 2 of 6 | 2.44 m |
| +Z with original buffer | 45.64 | 43.43 | 2 of 6 | 1.24 m |
| +Z with 1.5 m more buffer | 45.46 | 41.99 | 4 of 6 | 2.25 m |

The buffered +Z case used **41.29 kg** on average, compared with **48.50 kg** in the same +X cases. Wider prediction geometry recovered 1.01 m of minimum modeled clearance compared with the original +Z policy. It also made three paired scores worse than +X.

On three additional delayed-radar pairs, buffered +Z scored **46.70, 46.26 and 43.07**. The +X scores were **46.96, 40.68 and 39.62**. These used head-on seed 17, crossing seed 29 and offset-crossing seed 43. Across all nine pairs, buffered +Z averaged **45.42** versus **42.85** for +X and met 46 in **six of nine** cases versus **three of nine**. All arrived without modeled contact. The buffered candidate's minimum modeled gap was **2.25 m**, compared with **2.44 m** for +X. Both means remain below 46, and the test plant omits the full debris field and true collision geometry. This is the strongest candidate for a separate flight comparison. The controller source remained at the verified baseline. All 166 automated tests passed.

The scorer checks damage first. Damage **above** 209,200 J triggers the loss formula. Otherwise center-to-center distance **at or below 50 m** wins. On arrival the score is `50 + 12000/T − 100F/75 − 200D/209200`. There is no docking, orientation, relative-speed or time-limit test in that win condition. The 75 kg value is a scoring normalization. A loss replaces the positive time term with a negative normalized remaining-distance term. A simulator collision can still occur before entry. [Scoring.py](../STU4-Sim/Systems/Scoring.py) · [Official vehicle model](https://github.com/SimDynamX/STU4-Solution/wiki/1.-vehicle-specs-and-model-details)

Another burn pays when `12000(1/T_new − 1/T_old) > (100/75) × extra_kg` at equal damage and success. Around 248 s, a kilogram costs about **6.85 s** of marginal time saving. Closing the 6.85-point gap from the observed arrival mean to 50 requires 5.14 kg at fixed time or approximately 31 s at fixed fuel for a representative 248 s flight. Reaching 60 would require 12.64 kg or about 64 s on the same terms. These are tradeoffs, not estimates of recoverable performance.

## Forces, frames and prediction

The final scenario enables spherical Mars and Phobos fields and disables the Sun and Deimos fields. It includes a base scenario named `MarsAndMoons_NoGravity`, then explicitly enables the two fields. The shuttle's `ApplyGravity` system samples the combined field at 1 Hz. Debris receives the same field through a round-robin update and initial random impulses. The optional hunting adjustment is disabled. Thrusters, contacts and the launchpad can also change the shuttle's motion. I found no configured aerodynamic or higher-order gravity model affecting this comparison. [Final simulation](../STU4-Sim/Sims/FINAL_SUBMISSION.simconf) · [Debris system](../STU4-Sim/Systems/SpawnDebris.py)

`Controller.py` reads both vehicle states relative to and expressed in the **Phobos body-fixed frame**. Its origin accelerates and its axes rotate. Body-axis rows are used to transform commands and the logged launch nose agrees with the transpose of the scenario quaternion's active rotation. The cycler uses custom propagation with zero velocity relative to Phobos. Its coordinates were exactly (−850, 120, −9530) m, with zero measured displacement and velocity in all 13 recordings. It moves with Phobos in inertial space. [Controller state and transforms](Controller.py#L438) · [Installed scenario](../STU4-Sim/Sims/FINAL_SUBMISSION.simconf)

For an offline reference, let `R` map Phobos axes to J2000, `r` be shuttle position from Phobos in body axes and `q = Rr`. In inertially oriented relative coordinates, `q″ = a_Phobos(q) + a_Mars(q − q_Mars) − a_origin`, using the SPICE acceleration of the Phobos origin relative to the solar-system barycenter. In rotating coordinates the additional terms are `−2 RᵀR′ v − RᵀR″ r`, which include Coriolis, centrifugal and angular-acceleration effects. The Phobos spin rate is about 0.000228 rad/s. The implementation uses the installed epoch, kernels and two configured gravitational parameters. This reference reconstructs the continuous external field independently of the live controller. Simulator force sampling, time stepping and contact physics can produce differences. [Offline reference model](tests/audit_coasts.py)

| Component in Phobos axes, flight 035405 (m/s²) | Launch sample | Near 100 s | Arrival |
|---|---|---|---|
| Direct Phobos | (+.00082, −.00012, +.01032) | (+.00077, −.00011, +.00933) | (+.00066, −.00008, +.00775) |
| Direct Mars minus ephemeris-origin acceleration | (+.00235, −.00092, +.00157) | (+.00232, −.00098, +.00159) | (+.00228, −.00106, +.00164) |
| Coriolis | (0, +.00003, 0) | (+.00003, +.00043, 0) | (−.00002, +.00025, 0) |
| Total including position-dependent frame terms | (+.00313, −.00101, +.01189) | (+.00308, −.00065, +.01092) | (+.00287, −.00089, +.00939) |

Mars's absolute field was about 0.488–0.489 m/s² here. Applying that directly to the shuttle in Phobos coordinates would double-count most of the origin's acceleration. The second table row includes the difference between the ephemeris acceleration and the configured fields, so it cannot be attributed solely to the Martian tide. The spin-position term was about 0.00004 m/s² on this route. Direct Phobos gravity fell about 25% as the shuttle moved from roughly 8.27 to 9.53 km radius. Over 22 separate quiet coast spans the measured magnitude of Δv/Δt ranged from **0.00996 to 0.01176 m/s²**, median **0.01075**. For scale, coasting for 180–220 s at 0.01075 m/s² would change velocity by 1.94–2.37 m/s and position by 174–260 m relative to constant velocity.

**What guidance already does.** Every tick it computes `−μr/|r|³`, then learns a residual from the measured velocity change while the preceding linear command is zero, rejecting large innovations and long sample gaps. Its time constant is 2 s. The estimate starts with central gravity alone. Ballistic entry and finite burns use the estimated full vector, held constant within each forecast. Each new decision gets an updated estimate. The residual can also absorb actuator transients and measurement timing. [Gravity observer](Controller.py#L459) · [Ballistic entry](Guidance.py#L335) · [Route propagation](RoutePlanner.py#L73)

I compared four forecasts initialized with the same measured shuttle state. Each horizon uses the first qualifying span per flight after 20 s, with 3 s to settle, flat fuel and damage, both commands off and telemetry gaps at most 2.5 s. Endpoints use the first actual sample at or beyond the horizon. Samples across horizons can overlap. The full-field integration used steps of at most 2 s. Reducing that to 0.5 s changed one 150 s endpoint by 0.0012 m. [Replay method](tests/audit_coasts.py)

| Horizon and flight count | Constant velocity | Current frozen gravity | Vary Phobos gravity, freeze residual | Full rotating-frame reference |
|---|---:|---:|---:|---:|
| 30 s, 12 | 5.24 m | 0.34 m | 0.36 m | 0.33 m |
| 90 s, 6 | 44.65 m | 2.13 m | 2.67 m | 0.69 m |
| 150 s, 2 | 125.22 m | 6.39 m | 5.87 m | 1.57 m |

Entries are median 3-D endpoint errors. At 150 s the two flown predictions missed by 10.43 and 2.34 m, versus 1.50 and 1.64 m with the reference. Simply updating central gravity made one of those forecasts worse, from 2.34 to 9.02 m. At 30 s, current-model maximum error was 1.14 m. Improving long-range position is feasible. The decision-level benefit is much smaller in these recordings.

* **Arrival time.** In nine uninterrupted terminal coasts lasting 38–163 s, both models predicted entry in all nine. Median absolute error to first recorded entry was **0.39 s** with frozen gravity and **0.15 s** with the full field. Maximum errors were 1.08 and 0.49 s. The reference is a sampled crossing, so subsecond differences approach the recording resolution.
* **Course correction.** Five official arrivals used `TRIM_BURN`. From the last sampled state before the first trim in each, both models predicted missing the official sphere in all five. Full-field closest ranges were **51.9, 72.2, 80.6, 116.4 and 61.2 m**. At the first sample already labeled trim, one prediction changed to 48.7 m versus the controller's 53.3 m. That sample may already include corrective impulse, and 48.7 m has only 1.3 m of entry margin. The data do not establish a removable first trim. These counterfactuals assume no future thrust or contact.
* **Fuel at stake.** Assigning each fuel increment to its preceding sampled mode gives **1.23 kg per arrival** for all trim intervals combined, including seven arrivals with zero sampled trim fuel. Even deleting that entire recorded category at fixed time would buy roughly **1.64 points** before paying for any replacement maneuvers. It cannot explain the full gap to 50.

## Capture geometry and propulsion

Guidance aims along the target line by default, can select a lateral flyby aim point, and explicitly tests intersection with a sphere. It starts or retains a quiet coast using 42 m or 46 m to allow prediction error and prevent command chatter. `ARRIVED` uses the official 50 m. There is no docking burn. A quiet coast shuts off linear and angular commands except spin recovery. In score mode the old 70% latch and fixed cruise-speed servo are disabled. Late maneuvering can still be justified by a lost intercept or a measured hazard. Hull dimensions matter for hazard clearance, while the scorer uses the shuttle reference point. In the earlier eight matched allocator flights, pricing 50 m while preserving the guidance margin changed the mean by −0.01 point. Keep the margin. [Arrival and cutoff logic](Guidance.py#L589) · [Actuation](Controller.py#L638) · [Prior paired experiment](CONTROLLER_AUDIT.md)

The installed no-mains jet array solves a minimum summed jet-time allocation for a six-axis force/torque request and rescales on-times to the actuator period. Positive body X has the highest measured pure-axis acceleration. At a 0.12 m/s² request and zero torque, delivered accelerations on +X, −X, +Y, −Y, +Z and −Z were respectively **0.175, 0.097, 0.143, 0.096, 0.096 and 0.117 m/s²**. All six cost about **4.32 kg per m/s**. The initial target is nearly along +Z, approximately (−0.062, −0.041, +0.997) in body coordinates. Holding that pose and thrusting along the actual line cost about 4.75 kg per m/s in the allocator check. Requests for rotation can change which jets fire, the delivered acceleration and the cost per useful forward impulse. The flight interface commands body accelerations rather than individual jets. [Installed allocator](../STU4-Sim/Systems/JetSelectionActuator.py) · [Jet configuration](../STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv) · [Allocator tests](tests/test_allocator_response.py) · [Official competitor API](https://github.com/SimDynamX/STU4-Solution/wiki/3.-competitor-python-api)

Across the 12 arrivals, fuel used by 20 s averaged **10.05 kg** (6.08–12.61). Intervals labeled avoidance averaged **10.39 kg** (0.04–27.63), and angular-on/linear-off intervals averaged **1.49 kg** (0.15–4.71). The avoidance category includes useful forward acceleration. The categories overlap. An average 0.64 kg was assigned to intervals beginning with both commands off, illustrating command transitions within the sample interval. There is insufficient information to separate simultaneous translation and torque fuel exactly. The geometric paths were already fairly straight. Sampled path length exceeded net range closure by a median **15.9 m** over roughly 1,256 m. Cutting distance alone leaves little room compared with changing when useful outward velocity is acquired.

## Controlled burn, coast and orientation comparisons

The new bench starts at the final scenario's actual location, attitude, zero initial velocity and epoch. It uses the stationary cycler, real jet geometry, mass/inertia, installed impulse/fuel calculation and reconstructed external field. It runs the team guidance loop at a deterministic 20 Hz with an ideal rigid body. The official actuator runs asynchronously, and the bench omits the launchpad contact constraint, Unreal collision response, damage, and full 500-object sensor traffic. In debris tests a positive modeled hull gap is required before reporting an undamaged score. All profile variants below are confined to the offline harness. [Bench](tests/check_launch_fuel.py) · [Flight loop fixture](tests/test_controller_loop.py)

| Clear flight from final pose | Time (s) | Fuel (kg) | Score |
|---|---:|---:|---:|
| Current adaptive guidance, +X | 198.80 | 43.10 | 52.89 |
| Main-burn cutoff at 40 s | 291.95 | 32.34 | 47.98 |
| Cutoff at 50 s | 246.70 | 36.07 | 50.55 |
| Cutoff at 60 s | 207.65 | 41.60 | 52.32 |
| Cutoff at 66 s | 196.05 | 43.67 | 52.98 |
| Cutoff at 68 s | 189.55 | 45.18 | **53.07** |
| Cutoff at 70 s | 186.10 | 46.08 | 53.04 |
| Cutoff at 80 s | 169.85 | 51.75 | 51.65 |
| Two phases, coast from 40–50 s, cutoff at 78 s | 206.50 | 42.68 | 51.21 |
| Align and use +Z for thrust | 231.40 | 36.75 | 52.86 |
| Hold the launch attitude and allocate translation along the demand | 228.20 | 39.27 | 50.23 |
| Counterfactual, +X already aligned at launch | 191.40 | 40.03 | 59.32 |

The cutoff sweep included 40, 50, 60, 62, 64, 66, 68, 70, 72, 74 and 80 s. It changes the discretionary cutoff only. Attitude limits, flyby corrections and other guidance remain active. The split example gives the same total *permitted* main-burn duration as the 68 s case. Its realized fuel and impulse differ. With fixed impulse on an unobstructed path, delivering Δv earlier by Δt advances position by approximately Δv × Δt. A later burn needs a reason such as a measured crossing, unavailable initial thrust direction, or a newly predicted miss. The single split test cost 1.86 points relative to the best tested cutoff. Earlier useful impulse helps, but an extra burn can still cost more points than it earns.

Current clear-flight peak closing speed was **8.43 m/s**, versus **8.90 m/s** at the best tested cutoff. Spending another 2.08 kg saved 9.25 s for just 0.18 point. Going from the 68 s to the 80 s cutoff spent 6.57 kg and saved 19.70 s, **losing 1.42 points**. The current code prices the next 2 s of acceleration through arrival using observed fuel per delivered Δv and planning acceleration. It uses a maneuver reserve and hysteresis. It projects the radial component to price discretionary acceleration when the unpowered path misses laterally. This is a local decision with an approximate actuator model. The live arrivals peaked at 6.35–8.76 m/s closing speed (mean 7.14). There is no universal optimal speed across different encounters. [Score cutoff](Guidance.py#L561) · [Fuel model learning](Guidance.py#L82)

**What the ceiling means.** 53.07 is a feasible clear result in this benchmark with the supplied start, and a lower bound on the best score in that plant. It is neither a proven upper bound nor a qualifying Space Teams result. The already-aligned 59.32 case isolates the value of a favorable initial pose that the competitor cannot choose. The older constant-field synthetic clear benchmark reproduced 58.58. It uses different gravity and a different synthetic departure geometry. The 5.69-point difference between those benchmarks cannot be assigned to gravity alone. A paired live clear run with this source would materially improve calibration.

### Identical debris conditions

I put one head-on contact or three crossing contacts in the same final-pose plant. Both the synthetic rocks and shuttle experience position-dependent gravity. I used the same geometry, measurement seeds (7, 23, 41, 101), cadence and initial shuttle state for each policy. The baseline clear plant permits known-empty operation. The debris comparison includes the cost of acquiring and using radar as well as maneuvering.

| Eight matched flights, four per geometry | Mean time (s) | Mean fuel (kg) | Mean score | Minimum modeled hull gap (m) |
|---|---:|---:|---:|---:|
| Current +X adaptive policy | 227.48 | 46.67 | 41.52 | 4.31 |
| +X with 68 s cutoff | 242.88 | 43.31 | 42.17 | 5.43 |
| +Z thrust axis, adaptive policy | 264.32 | 37.64 | **45.27** | 4.57 |

The **11.37-point** gap between current clear (52.89) and current debris (41.52) is the largest controlled penalty here. It includes necessary avoidance. A clock cutoff gained 0.65 on average but lost 1.39 in crossing seed 101. Using +Z gained 3.75 on average. Most of its gain came from crossings (47.96 versus 40.92), with a smaller head-on result (42.57 versus 42.13). It lost 0.72 in head-on seed 7. +Z reaches useful thrust attitude with less turning, but its lower acceleration can also change *when* a rock crosses the shuttle's path. These comparisons cannot attribute all savings to reduced torque. They do show that a slower flight can score better at the same launch pose. The bench implements +Z by rotating the control axes and transforming both the physical jet matrix and inertia into those axes. No jet is added and the actual shuttle attitude at launch is preserved.

**Independent stress comparison.** I then used two new seeds (13 and 67), added an offset-crossing geometry, lengthened per-object revisit to 8.6 s, delayed delivery by 1 s, and supplied ten distant benign returns so the radar could remain fresh between observations of a threat. These are synthetic timing assumptions informed by the recorded throughput, not reconstructed flight scenes. Across six matched cases, the current policy averaged **43.06** (209.88 s, 48.50 kg) and +Z averaged **45.64** (230.43 s, 42.58 kg). All reached the sphere without modeled contact. In head-on seed 13, however, +Z reduced hull clearance from **5.32 m to 1.24 m** and slightly reduced score. In head-on seed 67 it lost 2.14 points. A 1.24 m modeled gap is smaller than the radar's single-axis 2 m position noise. This prevents recommending a blanket thrust-axis change for the submission.

## Radar, corridor selection and the two recorded impacts

The radar provides noisy relative center position in Phobos axes, surface range, scalar radial velocity, diameter and measurement time. The tracker interpolates historical ownship position and velocity, rejects inconsistent readings, and uses one-to-one gated association and a position/velocity Kalman filter. Ambiguous returns reduce confidence. The radar's rotating-shuttle velocity convention does not by itself corrupt radial velocity, since a rotation term proportional to ω × r is perpendicular to the radar ray. The camera handles exist but the flight loop does not use depth images. Camera geometry, timing and accuracy would need calibration before fusion. [Radar source](../STU4-Sim/Systems/Radar.py) · [Tracker](Guidance.py#L171) · [Official competitor rules](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218)

The radar initializes all contacts at timestamp zero and starts its elapsed-time clock after that scan. The controller starts from the scenario clock epoch. Receipt minus positive measurement timestamp in the accepted records had median **0.99 s**, 95th percentile **1.63 s**, and range **0.19–2.22 s**. This includes publication batching, scheduling and any difference in clock origin. It does not identify a constant offset to subtract. The median interval between accepted batches was **0.82 s**. Median accepted throughput across flights was **58 returns per simulated second**. Dividing 500 by that rate gives roughly **8.6 s per equivalent sweep**, versus 4.17 s from the nominal 120 Hz producer. It is an aggregate estimate, not a measurement of every track's revisit. Median per-flight track counts after 20 s were 501 total and 496 confirmed. A fresh batch says little about the age of the specific approaching track. [Radar timestamps and publication](../STU4-Sim/Systems/Radar.py#L117) · [Observation age and history](Guidance.py#L185)

Tactical prediction uses sampled relative motion with a finite-burn swept corridor, 0.5 s spacing and margins for between-sample motion and uncertainty. Constant common gravity cancels in relative separation. Differential Phobos acceleration across 100 m near launch is at most about 0.00025 m/s² from the central-field gradient, or about 0.11 m in 30 s at that bound. Differential Coriolis at 5 m/s relative speed can reach about 0.0023 m/s², or about 1 m in 30 s and 16 m in 120 s at constant magnitude. These are order-of-magnitude bounds, not measured miss distances. Short-range radar position noise alone is 2 m per axis. At long range the common-gravity assumption, delayed debris gravity updates and uncertain transverse velocity matter more. The planner inflates rock radius by 15%, adds an estimated 2.5 m shuttle radius, a 3 m clearance margin and 2.5 times projected uncertainty. The collision mesh and radar sphere need not have identical geometry. [Tactical predictor](Guidance.py#L779) · [Strategic radar field](RoutePlanner.py#L181)

Strategic guidance already compares delays, advances, shifted flyby points and shallow turns against the direct path. It evaluates risk for the next 120 s in 3 s segments, prices time and fuel through entry up to 360 s, and ranks viable clear alternatives by score in the active policy. It performs two alternative evaluations per update and rechecks the selected route. Tactical launch protection starts immediately, can interrupt the route, and favors feasible short coasts or burns. This is a limited receding-horizon search. It does not jointly optimize an arbitrary sequence of actions through all 500 objects. The forecast approximates turn response and prices translation using learned fuel cost. It lacks a maneuver-specific simultaneous torque cost. More candidates also consume control-loop time. Across the recordings the sampled planning time was **73 ms median, 136 ms p95, 265 ms maximum** of wall time. That does not give the delay of every actuator command in simulation time. [Search and ranking](RoutePlanner.py#L270) · [Candidate generation](RoutePlanner.py#L424) · [Tactical handoff](Controller.py#L513)

| Flight | What the sensors and controller showed | What the evidence establishes |
|---|---|---|
| 040439 | At receipt 62.45 s a new observation had surface range 8.61 m and radial rate −2.71 m/s. A reconstruction associated it with a four-hit track. Logged avoidance reported limited clearance through 66.10 s. At 67.20 s the reported hull gap jumped to 54.3 m. First damage at 68.29 s coincided with spin jumping from 0.92 to 80.03°/s. | A risk was seen and the controller resumed acceleration before damage. Review threat identity and risk release across the intervening radar batches. The logs cannot identify the collider. |
| 084357 | At receipt 188.49 s a new observation had surface range 51.05 m and radial rate −8.38 m/s. The replay associated it with a 22-hit track and forecast a 23.43 m center miss in 4.55 s. The flight reported infinite hull gap on the samples preceding damage at 194.38 s. Spin jumped from 1.37 to 69.99°/s. | Fast closure alone would have suggested about 6 s to contact, but that assumes zero transverse motion. Infinite reported gap means the predictor retained no finite hazard for that assessment. The sampled telemetry and replay establish a missed warning before damage, without identifying which physical object hit. |

Replay uses only accepted sensor returns and interpolated ownship telemetry. It reconstructs tracks, not engine collision identities. Rejected raw returns, exact per-tick state history, true meshes, and actual minimum clearances are unavailable. The damage penalty alone in 084357 was **11.38 points**, or 0.95 point averaged over the 12 arrivals. Recovery propellant and the unfinished flight add exposure that cannot be priced from these logs. Neither impact provides evidence for blaming long-horizon gravity error or uniformly shrinking collision margins. [Replay](tests/audit_coasts.py) · [Damaged arrival recording](diagnostics/20261120T084357_673050Z) · [Incomplete flight recording](diagnostics/20261120T040439_044119Z)

**Follow-up on missed hazards.** I screened accepted radar samples within 60 m of the debris surface, closing at more than 2 m/s, with radial time to the estimated shuttle hull of at most eight seconds. That simple condition flagged a return about 5.9 s before each damage event. It also flagged 21 receipt batches across nine of the 11 zero-damage arrivals. In three of those batches the shuttle completed the next eight seconds with both commands off and fuel flat. In each damaged recording, the particular candidate reconstructed from the alert received no new observation before the first damage. A radial trigger is worth logging as a second opinion. The data support investigating how an aging track is cleared and how quickly the controller can respond. They do not support making the radial trigger command a maneuver on its own.

## Hypothesis assessment

| Hypothesis | Current implementation | Decision |
|---|---|---|
| Coasting ignores gravity | **Already implemented correctly** for local full-vector acceleration and ballistic entry. **Partially implemented** for long-horizon gradients and frame terms. | Keep the observer. Use the full-field model offline to audit disputed decisions. |
| The target requires rendezvous or braking | **Already implemented correctly.** Stationary in local axes, positional capture, inner coast hysteresis, no terminal speed match. | Preserve quiet coast and safety margin. |
| A fixed speed cap or a long-range cutoff limits the score | **Already implemented correctly** in active score mode. **Implemented but potentially inefficient** for local two-second pricing and the radial fallback. | Little measured clear-flight upside to replacing it with a fixed schedule. |
| Propulsion orientation is always fuel optimal | **Implemented but potentially inefficient.** The allocator minimizes activity for the requested wrench. Guidance chooses the demand and generally turns toward +X. | +Z deserves controlled research for selected geometries. Actual collision-free live gain is undetermined. |
| Repeated fine pointing or gravity trims account for the score deficit | **Little support.** Idle attitude is damped, and both actuators go quiet on verified coasts. Allocation of simultaneous torque/translation fuel is **unable to be determined** from the recorder. | Quantify burn-window fuel and useful outward impulse in a paired trial. |
| No predictive collision planning exists | **Already implemented** for finite-horizon tactical and strategic alternatives. **Partially implemented** for multiple encounters and delivered maneuver cost. | Prioritize the two observed missed or released hazards and score executable early alternatives. |
| Track fusion uses all available evidence | **Already implemented** for independent radar channels and ownship history. Camera depth fusion and explicit radar-clock calibration are **missing** in the active loop. | Validate timing and uncertainty using existing radar before adding a sensor. |
| A global 500-object plan would be superior | A globally optimized trajectory is **missing**. Its advantage at measured revisit, uncertainty and CPU cost is **unable to be determined**. | Use short alternatives and revalidation. A new global solver has no demonstrated score benefit. |

## Priorities before submission

| Rank | Action | Score evidence and confidence | Complexity and regression risk |
|---:|---|---|---|
| 1 | Keep the current submission. Reproduce the hazard-release and missed-warning cases, including the last two observations, forecasted closest approach, estimated hull, per-track age and time of release. | High confidence in the two observed events. The damaged arrival alone lost 11.38 points to damage. Prevention and average gain are unproven. | Low for replay, moderate for changing decisions. A longer hazard hold can delay launch and increase fuel. |
| 2 | Compare a low-turn +Z departure against +X in isolation, with the same geometry and a test of whether enough escape acceleration remains. Include the delayed-radar head-on case first. | Best measured control opportunity. +3.75 and +2.58 mean points in the two synthetic sets. The smaller head-on clearance makes a live score estimate premature. | Moderate implementation and high safety sensitivity. The planner, attitude transform, applied acceleration and escape forecast must agree on the active axis. |
| 3 | Price **both** coast and an executable early burn or shallow turn through the next credible crossing. Include delivered forward impulse, turn fuel and the return to a capturable coast. Keep the existing tactical veto. | High confidence that encounters carry a cost. The 11.37-point penalty includes unavoidable cost. Recoverable share is unknown. | Moderate. First replay alternatives from the same observations, then compare on unseen geometries. Avoid adding synchronous search until latency is measured. |
| 4 | Use full-field propagation in offline or shadow comparisons of borderline long intercepts. | High confidence in better 90–150 s position prediction. No demonstrated removable pre-trim burn or material live score gain. | Low offline, moderate in flight. An expensive propagator in the radar loop could worsen latency. |
| 5 | Retain current capture margin, adaptive cutoff and quiet-coast logic. Defer wholesale camera fusion, a new allocator or a global planner. | Fixed cutoff gained 0.18 on the best clear test. Alternative capture pricing was neutral in earlier debris tests. Larger changes lack deadline-quality evidence. | Low to retain. Major rewrites and uniformly smaller margins have high validation cost. |

**Space Teams validation.** The regular controller is ready to fly. With the clear baseline recorded, prioritize repeated final-scenario runs and capture the original radar/telemetry folder and a time-aligned video when damage occurs. Retain official times, fuel, damage and scores for every terminal outcome. Compare fuel, outward speed and nose/active-axis error at 20, 60 and 100 s, first sustained burn, fraction spent coasting, avoidance-mode fuel, and progress at the first trim. An alternate plan evaluated alongside the controller must use only measurements available at that instant. Assess changes against the current policy on the same initial conditions and radar schedules offline, then use repeated live trials to establish collision reliability and the mean across attempts. A reliable 50–60 average and the live benefit of alternative guidance are still unverified.

## Reproduction and changed files

Run from the team checkout with Space Teams Python 0.56.0. The coast replay uses the installed `spiceypy` and NAIF kernels (`naif0012.tls`, `pck00010.tpc`, `de440.bsp`, `mar097.bsp`). It reads local diagnostics and corresponding `STPro/Logs` folders. The installed force implementation is in `D:/Main/Compute_Server/packages/EnvironmentalModels/Gravity/src/Gravity_Spherical.cpp` and `Propagator/ApplyGravity.py`. The base scenario's Mars and Phobos entity templates specify their gravitational parameters and SPICE configuration.

```powershell
$py = "$env:LOCALAPPDATA/SimDynamX/STPro/Versions/v0.56.0/py/Scripts/python.exe"
$packages = 'D:/Main/Compute_Server/packages'
& $py -B -m unittest discover -s tests
& $py -B tests/audit_coasts.py $packages --official --verbose --decisions --encounters
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages --main-burn-until 68
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages --main-burn-until 78 --pause 40 50
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages --scene crossings --seed 7 --attitude top
& $py -B tests/check_launch_fuel.py --through-arrival --final-gravity $packages --scene head-on --seed 13 --delayed-radar --attitude top
```

Replace `top` with `flight` for the matched +X control. The original clear benchmark runs by omitting `--final-gravity`. The entire automated suite passed **166 of 166**. The work added this report and the [coast and encounter replay](tests/audit_coasts.py), extended the [offline flight bench](tests/check_launch_fuel.py), and let its [test fixture](tests/test_controller_loop.py) accept the final Phobos gravitational parameter. The regular flight sources retain their flown hashes and `FINAL_SUBMISSION` still selects the regular team controller. All simulations run for this investigation were offline.
