using EngineSim.Enums;

namespace EngineSim.VehicleSystems.Driveline;

// The clutch (#38): how much torque can pass between the engine and the gearbox, and whether it's slipping.
public class Clutch
{
    public const double MaxClutchTorqueNm = 540;   // fully out it can hold 1.5 × the engine's peak torque
    public const double LockUpWindowRpm = 10;      // speeds this close count as matched

    // 0 = pedal down (nothing passes), 1 = pedal fully out
    private double _engagement;
    public double Engagement
    {
        get => _engagement;
        set => _engagement = Math.Clamp(value, 0, 1);
    }

    // The most torque it can pass right now: MaxClutchTorqueNm × Engagement
    public double CapacityNm => MaxClutchTorqueNm * Engagement;

    public ClutchState State { get; private set; } = ClutchState.Disengaged;

    // Torque passed this tick (Nm). Positive = the engine driving the wheels, negative = the wheels driving the engine.
    public double TorqueNm { get; private set; }

    // Decide the state and the torque for this tick.
    //   engineRpm:            crank speed
    //   gearboxInputRpm:      how fast the gearbox side is turning, from the vehicle's speed through the gearing
    //   torqueToStayLockedNm: the torque the clutch would have to pass to keep both sides turning together
    //   inNeutral:            no gear: nothing to connect to
    //
    // Disengaged: Engagement == 0, or inNeutral.                                     TorqueNm = 0
    // Locked:     |engineRpm − gearboxInputRpm| ≤ LockUpWindowRpm
    //             and |torqueToStayLockedNm| ≤ CapacityNm.                           TorqueNm = torqueToStayLockedNm
    // Slipping:   anything else. The faster side drags the slower one.               TorqueNm = CapacityNm × Math.Sign(engineRpm − gearboxInputRpm)
    public void Update(double engineRpm, double gearboxInputRpm, double torqueToStayLockedNm, double torqueToMeetThisTickNm, bool inNeutral)
    {
        if (Engagement == 0 || inNeutral)
        {
            TorqueNm = 0;
            State = ClutchState.Disengaged;
            return;
        }
        if (Math.Abs(engineRpm - gearboxInputRpm) <= LockUpWindowRpm && Math.Abs(torqueToStayLockedNm) <= CapacityNm)
        {
            TorqueNm = torqueToStayLockedNm;
            State = ClutchState.Locked;
            return;
        }
        // Slipping: pass up to the clutch's capacity, but never more than it takes to bring the two speeds together
        // by the end of this tick. Otherwise one tick of full slip overshoots, and the next tick flings it back.
        State = ClutchState.Slipping;
        TorqueNm = Math.Clamp(torqueToMeetThisTickNm, -CapacityNm, CapacityNm);
    }
}
