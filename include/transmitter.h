#pragma once

#include <Arduino.h>

bool transmitterInit();
bool transmitterSend(const String &payload);
void transmitterPoll();
bool transmitterReceiveDeployCommandWindow(uint32_t windowMs);
