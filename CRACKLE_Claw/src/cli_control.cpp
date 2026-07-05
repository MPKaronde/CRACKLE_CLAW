#include "cli_control.h"

static void print_help()
{
    Serial.println("--- CRACKLE CLAW CLI ---");
    Serial.println("  grip                  close until contact, monitor slip");
    Serial.println("  release               open to max angle");
    Serial.println("  zero servos           move both servos to min angle");
    Serial.println("  zero loadcells        tare load cells (~3s, remove load first)");
    Serial.println("  read                  print load cell readings once");
    Serial.println("  monitor               toggle continuous load cell printing");
    Serial.println("  servo <l|r> <angle>   move one servo (0-180)");
    Serial.println("  help                  show this list");
    Serial.println("------------------------");
}

void CLI_Setup(Gripper* g)
{
    Serial.begin(115200);
    Serial.println("Initializing gripper...");
    g->zero_servos();
    Serial.println("Zeroing load cells — remove all load now.");
    g->zero_loadcells();
    Serial.println("Ready. Type 'help' for commands.");
}

void CLI(Gripper* g)
{
    static bool monitoring = false;

    if (monitoring) {
        Serial.printf("LC1: %.2f  LC2: %.2f\n",
                      g->get_loadcell_reading(LEFT),
                      g->get_loadcell_reading(RIGHT));
    }

    if (!Serial.available()) {
        delay(100);
        return;
    }

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    Serial.print("> "); Serial.println(line);

    if (line.equalsIgnoreCase("grip")) {
        Serial.println("Opening then gripping...");
        g->release();
        g->grip_object();
        Serial.println("Done.");

    } else if (line.equalsIgnoreCase("release")) {
        Serial.println("Releasing...");
        g->release();
        Serial.println("Done.");

    } else if (line.equalsIgnoreCase("zero servos")) {
        Serial.println("Zeroing servos...");
        g->zero_servos();
        Serial.println("Done.");

    } else if (line.equalsIgnoreCase("zero loadcells")) {
        Serial.println("Zeroing load cells — remove all load. Waiting 3s...");
        g->zero_loadcells();
        Serial.println("Done.");

    } else if (line.equalsIgnoreCase("read")) {
        Serial.printf("LC1: %.2f  LC2: %.2f\n",
                      g->get_loadcell_reading(LEFT),
                      g->get_loadcell_reading(RIGHT));

    } else if (line.equalsIgnoreCase("monitor")) {
        monitoring = !monitoring;
        Serial.println(monitoring ? "Monitoring ON." : "Monitoring OFF.");

    } else if (line.startsWith("servo")) {
        // "servo l 90" or "servo r 45"
        String args = line.substring(5);
        args.trim();

        if (args.length() < 3) {
            Serial.println("Usage: servo <l|r> <0-180>");
            return;
        }

        char side_char = tolower(args.charAt(0));
        String angle_str = args.substring(1);
        angle_str.trim();
        int angle = angle_str.toInt();

        if (side_char != 'l' && side_char != 'r') {
            Serial.println("Side must be 'l' or 'r'.");
            return;
        }

        Side side = (side_char == 'l') ? LEFT : RIGHT;
        g->move_servo(side, angle);
        Serial.printf("Servo %c -> %d\n", side_char, angle);

    } else if (line.equalsIgnoreCase("help")) {
        print_help();

    } else {
        Serial.print("Unknown command: ");
        Serial.println(line);
        Serial.println("Type 'help' for commands.");
    }
}
