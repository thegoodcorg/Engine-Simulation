using EngineSim;
using EngineSim.Constants;
using EngineSim.Enums;
using EngineSim.Helpers;
using EngineSim.VehicleSystems.Driveline;

AmbientConditions idealConditions = new();
Engine testEngine = new(95.5, 88.4, 144.2, 10.3, 6, idealConditions, FuelProperties.PumpGas); // bore, stroke, rod length (mm), compression ratio, cylinders
Vehicle testVehicle = new(testEngine);

const double dt = 0.05;           // time step in seconds (20 updates per second)
const double throttleStep = 0.01; // how much each W/S press changes the throttle

Console.WriteLine("Engine built!");
Console.WriteLine($"Cyl bore = {testVehicle.BoreMm}mm / Cyl stroke = {testVehicle.StrokeLengthMm}mm / Cyl count = {testVehicle.CylinderCount}");
Console.WriteLine($"Rod length = {testVehicle.RodLengthMm}mm / Compression ratio = {testVehicle.CompressionRatio}:1");
Console.WriteLine($"Displacement = {testVehicle.DisplacementLiters:F2} L");
Console.WriteLine($"Redline = {testVehicle.MaxRpm} RPM");

Console.WriteLine();
Console.WriteLine("1 = battery on/off, hold 3 = start engine, 4 = stop engine, W/S = throttle up/down, Q = quit");
Console.WriteLine("hold Space = clutch, Up/Down = shift, N = neutral, hold B = brake, PageUp/PageDown = grade");
Console.CursorVisible = false;

// Reserve four lines for the live display and remember where they start.
// Printing them first means the console has already scrolled, so the row number stays valid.
// Each line stays under 80 characters so it never wraps onto the next one.
const int displayLines = 4;
for (int i = 0; i < displayLines; i++)
    Console.WriteLine();
int displayTop = Console.CursorTop - displayLines;

bool running = true;
while (running)
{
    // Handle every key pressed since the last update
    while (Console.KeyAvailable)
    {
        switch (Console.ReadKey(intercept: true).Key)
        {
            case ConsoleKey.D1:
                testVehicle.SwitchBattery();
                break;
            // 3 (start) isn't handled here: it has to be held, which ReadKey can't tell. See below.
            case ConsoleKey.D4:
            case ConsoleKey.NumPad4:
                testVehicle.ShutdownEngine();
                break;
            case ConsoleKey.W:
                testVehicle.ThrottleControl(throttleStep);
                break;
            case ConsoleKey.S:
                testVehicle.ThrottleControl(-throttleStep);
                break;
            // Shifting needs the clutch pedal down (hold Space first); otherwise the dash says so
            case ConsoleKey.UpArrow:
                testVehicle.ShiftUp();
                break;
            case ConsoleKey.DownArrow:
                testVehicle.ShiftDown();
                break;
            case ConsoleKey.N:
                testVehicle.SelectNeutral();
                break;
            case ConsoleKey.PageUp:
                testVehicle.ChangeGrade(+1);
                break;
            case ConsoleKey.PageDown:
                testVehicle.ChangeGrade(-1);
                break;
            case ConsoleKey.Q:
            case ConsoleKey.Escape:
                running = false;
                break;
        }
    }

    // Held keys: ReadKey only sees presses, so ask whether they're down right now
    testVehicle.SetStartKeyHeld(KeyState.IsDown(ConsoleKey.D3) || KeyState.IsDown(ConsoleKey.NumPad3));
    testVehicle.SetClutchPedalHeld(KeyState.IsDown(ConsoleKey.Spacebar));
    testVehicle.SetBrakeHeld(KeyState.IsDown(ConsoleKey.B));
    testVehicle.Update(dt);

    // Jump back to the top of the display and redraw every line in place.
    // Fixed widths overwrite the previous values completely, so nothing is left behind.
    Console.SetCursorPosition(0, displayTop);
    // sim = how fast the crank-angle model ran compared with real time: under 1.0x means the sim is slower than real life
    Console.WriteLine($"throttle={testVehicle.ThrottlePosition * 100,3:F0}%  rpm={testVehicle.Rpm,5:F0}  torque={testVehicle.TorqueNm,4:F0} Nm  sim={testVehicle.RealTimeFactor,5:F1}x");
    Console.WriteLine($"fuel={testVehicle.FuelLevelL,7:F3} L  maf={testVehicle.MafGPerS,6:F2} g/s  map={GasHelper.ConvertKpaToPsi(testVehicle.ManifoldPressureKpa),5:F2} psi");
    Console.WriteLine($"speed={testVehicle.SpeedKmh,4:F0} km/h ({testVehicle.SpeedMph,3:F0} mph)  gear={GearLabel(testVehicle.Gear)}  " +
                      $"clutch={ClutchLabel(testVehicle.ClutchState),-8}  brake={testVehicle.BrakePedal * 100,3:F0}%  grade={testVehicle.GradePercent,3:+0;-0;0}%");
    string status = testVehicle.DriverMessage is null ? testVehicle.EngineStatus : $"{testVehicle.EngineStatus} | {testVehicle.DriverMessage}";
    Console.WriteLine($"bat={testVehicle.BatteryVoltage,5:F1} V  {status,-65}");
    // Empty tank ends the sim. A battery stall doesn't, so you can reconnect and restart.
    if (testVehicle.OutOfFuel)
    {
        Console.WriteLine();
        Console.WriteLine("Out of fuel. Sim over.");
        running = false;
    }

    Thread.Sleep((int)(dt * 1000));
}

Console.CursorVisible = true;
Console.WriteLine();

// Dash labels: N for neutral; "in" for the clutch pedal down
static string GearLabel(int gear) => gear == 0 ? "N" : gear.ToString();

static string ClutchLabel(ClutchState state) => state switch
{
    ClutchState.Disengaged => "in",
    ClutchState.Slipping => "slipping",
    _ => "locked",
};
