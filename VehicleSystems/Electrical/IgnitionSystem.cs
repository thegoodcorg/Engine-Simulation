namespace EngineSim.Electrical;

// Ignition (#6): whether the plugs are firing. It doesn't decide anything:
// the ECU decides each tick and the Vehicle passes the decision in here.
public class IgnitionSystem
{
    public bool SparkOn { get; private set; } = false;

    public void SetSpark(bool on)
    {
        SparkOn = on;
    }
}
