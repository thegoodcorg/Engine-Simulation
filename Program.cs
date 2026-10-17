using EngineSim;
using EngineSim.Constants;

AmbientConditions idealConditions = new();
Engine testEngine = new(95.5, 88.4, 6, idealConditions);

const double dt = 0.05;           // time step in seconds (20 updates per second)
const double throttleStep = 0.01; // how much each W/S press changes the throttle

Console.WriteLine("Engine built!");
Console.WriteLine($"Cyl bore = {testEngine.BoreMm}mm / Cyl stroke = {testEngine.StrokeLengthMm}mm / Cyl count = {testEngine.CylinderCount}");
Console.WriteLine($"Displacement = {testEngine.DisplacementLiters:F2} L");
Console.WriteLine($"Redline = {testEngine.MaxRpm} RPM");
Console.WriteLine($"Torque = {testEngine.TorqueNm}Nm");

Console.WriteLine();
Console.WriteLine("W = throttle up, S = throttle down, Q = quit");
Console.CursorVisible = false;

bool running = true;
while (running)
{
    // Handle every key pressed since the last update
    while (Console.KeyAvailable)
    {
        switch (Console.ReadKey(intercept: true).Key)
        {
            case ConsoleKey.W:
                testEngine.ThrottleControl(throttleStep);
                break;
            case ConsoleKey.S:
                testEngine.ThrottleControl(-throttleStep);
                break;
            case ConsoleKey.Q:
            case ConsoleKey.Escape:
                running = false;
                break;
        }
    }

    testEngine.Update(dt);

    // \r returns to the start of the line so the display updates in place
    Console.Write($"\rthrottle={testEngine.ThrottlePosition * 100,3:F0}%  rpm={testEngine.EngineSpeed,5:F0}  torque={testEngine.TorqueNm * testEngine.ThrottlePosition,3:F0} Nm   ");

    Thread.Sleep((int)(dt * 1000));
}

Console.CursorVisible = true;
Console.WriteLine();
