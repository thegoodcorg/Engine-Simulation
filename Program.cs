using EngineSim;
using EngineSim.Constants;
using EngineSim.Helpers;

AmbientConditions idealConditions = new();
Vehicle testVehicle = new(95.5, 88.4, 6, idealConditions);

const double dt = 0.05;           // time step in seconds (20 updates per second)
const double throttleStep = 0.01; // how much each W/S press changes the throttle

Console.WriteLine("Engine built!");
Console.WriteLine($"Cyl bore = {testVehicle.BoreMm}mm / Cyl stroke = {testVehicle.StrokeLengthMm}mm / Cyl count = {testVehicle.CylinderCount}");
Console.WriteLine($"Displacement = {testVehicle.DisplacementLiters:F2} L");
Console.WriteLine($"Redline = {testVehicle.MaxRpm} RPM");
Console.WriteLine($"Peak torque = {testVehicle.PeakTorqueNm:F0} Nm");

Console.WriteLine();
Console.WriteLine("1 = battery on/off, hold 3 = start engine, 4 = stop engine, W = throttle up, S = throttle down, Q = quit");
Console.CursorVisible = false;

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
            case ConsoleKey.Q:
            case ConsoleKey.Escape:
                running = false;
                break;
        }
    }

    testVehicle.SetStartKeyHeld(KeyState.IsDown(ConsoleKey.D3) || KeyState.IsDown(ConsoleKey.NumPad3));
    testVehicle.Update(dt);

    // \r returns to the start of the line so the display updates in place
    Console.Write($"\rthrottle={testVehicle.ThrottlePosition * 100,3:F0}%  rpm={testVehicle.Rpm,5:F0}  torque={testVehicle.EngineTorqueNm,3:F0}Nm  | Bat = {testVehicle.BatteryVoltage,-8} | {testVehicle.EngineStatus,-40}");

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
