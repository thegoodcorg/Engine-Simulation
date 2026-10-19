namespace EngineSim.Electrical;

// Starter motor: spins the engine so it can start. It needs battery power to turn.
// No real cranking yet (#3): once it turns the engine, RPM goes straight to idle. Cranking comes later.
public class Starter
{
    public bool HasBatteryPower { get; set; }          // the Vehicle sets this from the battery
    public bool StarterEngaged { get; private set; }
    public double EngagedTimeSec { get; private set; }   // how long it has been turning without a break

    // Engage the starter. Returns false if there's no battery power to turn it.
    public bool TurnEngine()
    {
        StarterEngaged = HasBatteryPower;
        return StarterEngaged;
    }

    // Advance by dt seconds: count up while engaged
    public void Update(double dt)
    {
        EngagedTimeSec = StarterEngaged ? EngagedTimeSec + dt : 0;
    }

    // Disengage once the engine has started, or when the key is let go early. Resets the timer.
    public void Release()
    {
        StarterEngaged = false;
        EngagedTimeSec = 0;
    }
}
