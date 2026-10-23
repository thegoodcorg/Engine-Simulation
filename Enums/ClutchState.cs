namespace EngineSim.Enums;

// Whether the clutch is passing torque, and how (#38)
public enum ClutchState
{
    Disengaged, // engine and wheels move independently
    Slipping,   // the plates rub: torque passes, but the two sides turn at different speeds
    Locked,     // the two sides turn together as one
}
