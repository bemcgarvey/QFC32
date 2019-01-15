
#include "OffMode.h"
#include "Data.h"
#include "QFCConfig.h"
#include "Config.h"
#include "InputCapture.h"

void OffMode(void) {
    if (ailUpdate) {
        outputChannels.ailOut = inputChannels.ailIn;
        int ail2Out;
        ail2Out = config.ailServo.center - inputChannels.ailIn;
        ail2Out *= config.ail2Direction;
        ail2Out = config.ail2Servo.center + ail2Out;
        outputChannels.ail2Out = ail2Out;
        //TODO adjust ailOut and ail2Out according to direction and differential
        ailUpdate = false;
    }
    if (elevUpdate) {
        outputChannels.elevOut = inputChannels.elevIn;
        elevUpdate = false;
    }
    if (rudUpdate) {
        outputChannels.rudOut = inputChannels.rudIn;
        rudUpdate = false;
    }
}
