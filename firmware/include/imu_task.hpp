#pragma once

#include "shared_state.hpp"

void imuTaskInit(SharedState &state);
void imuTask(void *sharedState);