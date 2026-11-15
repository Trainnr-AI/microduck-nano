// SPDX-License-Identifier: Apache-2.0
// The firmware's functions for the Linux side (docs/design/firmware.md, "Firmware functions").
#pragma once

// Registers every function this build has with the Bridge. Call once in setup(), after Bridge.begin().
void rpc_register();
