#pragma once
#include <cstdint>

//High-Level Emulation (HLE) of the Casio Loopy BIOS.
//
//The real console boots by running Casio's copyrighted BIOS, which configures
//the on-chip peripherals, clears VRAM/RAM and then jumps to the cartridge entry
//point. These routines reproduce the observable result of that BIOS directly,
//so the emulator can boot cartridges without running or distributing the BIOS
//file.
namespace HLE
{
//Reproduces the BIOS boot/init routine (FUN_400): sets up the pin controller,
//timers, serial/audio and VDP, clears VRAM and work RAM, and leaves the CPU
//ready to execute the cartridge at its entry point (0x0E000480).
//
//Must be called after every hardware subproject (Memory, SH2 + peripherals,
//Video, Sound, Expansion) has been initialized.
void fast_boot();

//Registers high-level replacements for the BIOS library functions that the
//games call at runtime (graphics helpers, math, print, etc.).
void install_services();

//Removes the runtime BIOS function replacements.
void remove_services();

//Notifies HLE that a natural VSYNC boundary has been reached; this arms
//the 6A48 frame barrier. Called by the video subsystem at VSYNC start.
void notify_frame_boundary();

//TEMP: probes that observe (without replacing) the real BIOS decoding a bitmap,
//used to capture a reference output for the HLE decoder. Only meaningful when
//the real BIOS is present.
void install_reference_probes();
}  // namespace HLE
