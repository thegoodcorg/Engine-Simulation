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
throttle= 25%  rpm= 2410  torque= 118 Nm  sim=  2.4x
fuel= 79.214 L  maf= 18.42 g/s  map= 6.67 psi
speed=  62 km/h ( 39 mph)  gear=3  clutch=locked    brake=  0%  grade= +2%
bat= 12.6 V  Engine running
```

`sim` is how fast the simulation ran compared with real time on the last tick: under `1.0x` means it's running slower than real life (most likely at high RPM).

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

**Torque.** Worked out crank degree by crank degree (the crank-angle model). Every cylinder is its own gas model: it breathes from the intake manifold through its valves, is fuelled for the air it actually traps, burns, and loses heat to the walls. Each half degree, every cylinder's push on its piston is turned into torque on the crank:

```
cylinderTorqueNm = (cylinderPressurePa − crankcasePressurePa) × pistonAreaM2 × leverArmM
netTorqueNm      = sum of every cylinder's torque − frictionTorqueNm
```

The cylinders fire evenly, every `720 / cylinderCount` crank degrees (180° for a four, 120° for a six, 90° for a V8), so any cylinder count works. The dash shows torque averaged over each 0.05 s tick.

At full throttle the model currently makes about 44% more torque than a real engine (about 517 Nm at 4,400 RPM, against roughly 360 Nm indicated), because the burned gas is treated as plain air. The dyno ticket (#30) calibrates it.

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

Friction only applies while the crank is turning. Because it rises with RPM (and the throttle's air supply doesn't), an engine left at a fixed part throttle should settle at a steady speed instead of racing to the limiter. Until #30 brings the torque down, it settles close to the limiter (about 6,200 RPM at 10% throttle). The coefficients are constants at the top of `Engine.cs`.

**RPM.** RPM comes from net torque: the cylinders' torque minus friction, divided by the crank's inertia, every half degree. Pumping loss is already in the cylinder pressures. The starter holds the crank at 200 RPM while it's engaged, so the cylinders can fill and fire before it lets go. Nothing sets RPM directly. It rises when the engine makes more torque than it loses, and falls when it makes less. Redline is a hard ceiling.

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
| `Engine.cs` | Engine specs, displacement, redline, the cylinders, throttle and the crank-angle update |
| `AmbientConditions.cs` | Air pressure, temperature and density |
| `FuelSystem.cs` | Fuel tank capacity and level (placeholder) |

Tuning values such as rev rates, the piston speed limit, idle variation and BMEP are constants at the top of `Engine.cs`.

## Current limitations

- Full-throttle torque is about 44% high until the dyno ticket (#30) calibrates it (burned gas is treated as air).
- At high RPM the crank-angle model can run slower than real time (the dash's `sim` figure).
- Below 150 RPM the crank counts as stopped, because a half-degree step gets too long for the valve flow to stay stable.
- Cooling, lubrication, exhaust and the air filter are assumed to always work.

## Roadmap

The plan is a `Vehicle` class that owns every system and runs the update loop, passing values between them so no system reaches into another directly. Build order:

1. **RPM rework.** RPM comes from net torque (engine torque minus friction) divided by inertia, with an ECU idle floor.
2. **Battery.** Connected or not, with an ECU check before starting.
3. **Starter and start key.** A start command that brings the engine to idle.
4. **Fuel tank.** Capacity and level, an ECU fuel check, and a stall when the tank runs dry.
5. **Air.** Torque from real air math: air mass per cycle from volumetric efficiency, air density, displacement and throttle, then fuel burned, energy released and useful work. Fuel drain is connected here.
6. **Ignition.** Spark on or off, with an ECU rev limiter that cuts spark at redline.
