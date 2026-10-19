using System.Runtime.InteropServices;

namespace EngineSim.Helpers;

// Console.ReadKey only reports presses, never releases. This asks Windows whether a key is down right now,
// so we can tell a held key from a tap. Windows only.
public static class KeyState
{
    [DllImport("user32.dll")]
    private static extern short GetAsyncKeyState(int virtualKey);

    // ConsoleKey values match Windows virtual key codes. High bit set = key is down.
    public static bool IsDown(ConsoleKey key) => (GetAsyncKeyState((int)key) & 0x8000) != 0;
}
