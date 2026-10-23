# Engine Simulation

A console engine simulator written in C#. You describe an engine by its bore, stroke and cylinder count, and the sim works out the rest (displacement, redline and torque) from real engine formulas. Then you drive it live from the keyboard.

The default engine matches a 2024 Nissan Frontier's 3.8 L V6 (VQ38DD): 95.5 mm bore, 88.4 mm stroke, 6 cylinders.

## Running it

Requires the [.NET 9 SDK](https://dotnet.microsoft.com/download).

```
dotnet run
```

Run it in a real terminal (the VS Code terminal, PowerShell or Command Prompt). The VS Code Debug Console doesn't pass key presses through.

| Key | Action |
| --- | --- |
| 1 | Battery on/off |
| Hold 3 | Start the engine (hold for half a second while it cranks) |
| 4 | Stop the engine |
| W / S | Throttle up / down 1% |
| Hold Space | Clutch pedal down. Let go and the clutch engages over 0.6 s, so it slips for a moment |
| Up / Down arrow | Shift up / down (clutch in only) |
| N | Neutral (clutch in only) |
| Hold B | Brake (builds to full over 0.3 s, off when you let go) |
| PageUp / PageDown | Hill grade +1% / −1% (up to ±15%) |
| Q or Esc | Quit |

The sim prints the engine's specs, then a live dash that updates 20 times a second:

```
throttle= 25%  rpm= 2410  torque= 118 Nm
fuel= 79.214 L  maf= 18.42 g/s  map= 6.67 psi
speed=  62 km/h ( 39 mph)  gear=3  clutch=locked    brake=  0%  grade= +2%
bat= 12.6 V  Engine running
```

Clutch shows `in` (pedal down), `slipping` or `locked`. Gear shows `N` or `1`–`9`. A refused shift shows "press clutch to shift" after the engine status.

### How to drive it

1. Press **1** to connect the battery, then hold **3** until the engine starts and settles at idle (~700 RPM).
2. Hold **Space** (clutch in) and press **Up** to select 1st.
3. Press **W** a few times for 15–20% throttle.
4. Let go of **Space**. The clutch slips for a moment while the truck gets moving, then locks.
5. To change gear: hold **Space**, press **Up**, let go of **Space**. Ease off the throttle first, or the engine will flare.

Let go of Space at idle with no throttle and the engine stalls, like a real car. Press **N** with the clutch in to coast.


## How the engine is calculated

Everything is derived from bore, stroke and cylinder count, so any engine can be built without changing constants.

**Displacement.** The volume one piston sweeps, times the cylinder count:

```
cylinder volume = π/4 × bore² × stroke
```

Bore is squared, so it matters far more than stroke: a 4% error in bore throws displacement off by about 8.5%.

**Redline.** Set by mean piston speed, which is how fast the piston travels up and down. The sim caps it at 20 m/s, a sensible limit for a street engine:

```
max RPM = 30 × piston speed limit / stroke (m)
```

The result is rounded down to the nearest 100 RPM. For the Frontier engine this gives 6,700 RPM.

**Torque.** Calculated from brake mean effective pressure (BMEP), the average pressure pushing on the pistons over a cycle:

```
torque = BMEP × displacement / 4π
```

With BMEP at 12.5 bar this gives about 378 Nm for the Frontier engine, against Nissan's rated 381 Nm.

**Friction.** The engine loses more to friction the faster it spins: bearings and piston rings, oil churning, the valvetrain. Friction is worked out as a friction mean effective pressure (FMEP) that rises with RPM, using Heywood's figures for spark-ignition engines (`rpmK` = RPM in thousands):

```
fmepBar          = 0.97 + 0.15 × rpmK + 0.05 × rpmK²
frictionTorqueNm = fmepBar × 100,000 × displacementM3 / (4 × Math.PI)
```

- `0.97`: base friction that's always there (seals, accessories)
- `0.15 × rpmK`: grows steadily with speed (bearings, rings)
- `0.05 × rpmK²`: grows fast at high RPM (oil churning, valvetrain)

| RPM | FMEP | Friction torque (3.8 L) |
| --- | --- | --- |
| 700 | 1.10 bar | 33 Nm |
| 3,000 | 1.87 bar | 57 Nm |
| 6,700 | 4.22 bar | 128 Nm |

Friction only applies while the crank is turning. Because it rises with RPM (and the throttle's air supply doesn't), an engine left at a fixed part throttle settles at a steady speed instead of racing to the limiter: about 3,150 RPM at 10% throttle and 5,350 RPM at 20%. The coefficients are constants at the top of `Engine.cs`.

**RPM.** RPM comes from net torque: engine torque minus friction and pumping loss, divided by the crank's inertia. Nothing sets RPM directly. It rises when the engine makes more torque than it loses, and falls when it makes less. Redline is a hard ceiling.

**Idle.** The throttle is drive-by-wire: the pedal is only a request, and the ECU decides where the throttle plate actually sits. With your foot off, a little air still leaks past the closed plate (10 mm²), but not enough to idle. The ECU holds the plate slightly open on top of the pedal, about 1.5% at sea level, and adjusts it every tick to keep about 700 RPM:

```
plateOpening        = pedalPosition + idleThrottleOpening
rpmErrorRpm         = targetIdleRpm − rpm
idleThrottleOpening = 0.015 + 0.000016 × rpmErrorRpm + 0.00002 × (rpmErrorRpm added up over time)
```

- The first term is a best first guess at the opening where torque balances friction and pumping loss.
- The second reacts straight away: idling 100 RPM low opens the plate a little more.
- The third slowly removes whatever error is left, for example in thin air at altitude, where it finds a bigger opening by itself.
- The opening stays between 0 and 6%. It holds still while the pedal is pressed, and on a lift-off from high RPM it won't close below its idle position (a dashpot), so the engine coasts down to idle instead of braking straight through it.
- The target RPM wobbles by ±2%, so idle still sounds a little uneven.
- **Stalling:** if the engine is running on its own below 250 RPM, the ECU stalls it. The check is skipped for the first second after starting, while the engine picks up from cranking speed (200 RPM).

**Time step.** Every update is scaled by `dt`, the seconds elapsed per tick, so the sim behaves the same at any update rate.

**Air density.** `AmbientConditions` calculates air density from pressure and temperature with the ideal gas law, plus a ratio against a standard sea-level day (1.225 kg/m³). Hot or thin air gives a ratio below 1.0.

## Project structure

| File | Contents |
| --- | --- |
| `Program.cs` | Builds the engine, prints its specs and runs the keyboard-driven update loop |
| `Engine.cs` | Engine specs, displacement, redline, torque, throttle and the RPM update |
| `AmbientConditions.cs` | Air pressure, temperature and density |
| `FuelSystem.cs` | Fuel tank capacity and level (placeholder) |

Tuning values such as rev rates, the piston speed limit, idle variation and BMEP are constants at the top of `Engine.cs`.

## Current limitations

- RPM follows the throttle directly. Torque is calculated but doesn't drive RPM yet.
- The engine accepts `AmbientConditions`, but air density doesn't affect it yet.
- The fuel system holds a tank size and level but doesn't drain.
- Cooling, lubrication, exhaust and the air filter are assumed to always work.

## Roadmap

The plan is a `Vehicle` class that owns every system and runs the update loop, passing values between them so no system reaches into another directly. Build order:

1. **RPM rework.** RPM comes from net torque (engine torque minus friction) divided by inertia, with an ECU idle floor.
2. **Battery.** Connected or not, with an ECU check before starting.
3. **Starter and start key.** A start command that brings the engine to idle.
4. **Fuel tank.** Capacity and level, an ECU fuel check, and a stall when the tank runs dry.
5. **Air.** Torque from real air math: air mass per cycle from volumetric efficiency, air density, displacement and throttle, then fuel burned, energy released and useful work. Fuel drain is connected here.
6. **Ignition.** Spark on or off, with an ECU rev limiter that cuts spark at redline.
