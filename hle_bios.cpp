#include "core/hle/hle_bios.h"

#include "core/sh2/sh2_bus.h"
#include "core/sh2/sh2_local.h"
#include "core/timing.h"
#include "video/video.h"
#include "log/log.h"

#include <cstdio>
#include <cstdarg>
#include <string>

using namespace SH2;

namespace HLE
{
//Base address of the on-chip peripheral registers; the real BIOS keeps this
//in the SH-2 GBR register and addresses everything relative to it.
constexpr static uint32_t OCPM_BASE = 0x05FFFF00;

//LoopyIO configuration registers touched by the BIOS io_setup routine.
constexpr static uint32_t LOOPYIO_30 = 0x0C05D030;
constexpr static uint32_t LOOPYIO_44 = 0x0C05D044;
constexpr static uint16_t IO_SETUP_CONSTANT = 0xA5A0;

static void ocpm_write8(uint32_t offset, uint8_t value)
{
	Bus::write8(OCPM_BASE + offset, value);
}

static void ocpm_write16(uint32_t offset, uint16_t value)
{
	Bus::write16(OCPM_BASE + offset, value);
}

//Reproduces a BIOS DMA fill: the DMA repeatedly writes the fixed source word
//0x0000 to an incrementing destination. A word count of 0 on real hardware
//means 65536 words; callers pass the already-expanded count.
static void dma_fill16(uint32_t dest, uint32_t word_count)
{
	for (uint32_t i = 0; i < word_count; i++)
	{
		Bus::write16(dest + i * 2, 0x0000);
	}
}

//TEMP diagnostic: append a line to geocalls.log
static void geo_log(const char* fmt, ...)
{
	FILE* f = fopen("C:\\Users\\karma\\AppData\\Local\\Temp\\geocalls.log", "a");
	if (f)
	{
		fprintf(f, "frame %llu ", Video::get_frame_count());
		va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
		fprintf(f, "\n");
		fclose(f);
	}
}
//TEMP: dump up to 32 words of a buffer to geobuf.log
static void geo_dump(const char* tag, uint32_t addr)
{
	FILE* f = fopen("C:\\Users\\karma\\AppData\\Local\\Temp\\geobuf.log", "a");
	if (f)
	{
		fprintf(f, "[%s] frame %llu addr=%08x:", tag, Video::get_frame_count(), addr);
		for (int i = 0; i < 32; i++) fprintf(f, " %04x", Bus::read16(addr + i*2));
		fprintf(f, "\n"); fclose(f);
	}
}
//High-level version of the BIOS io_setup routine (at BIOS offset 0x69C).
//It configures the LoopyIO sensor/direction register and briefly pulses a
//latch bit while doing so.
static void hle_io_setup(int mode_in)
{
	int mode = (mode_in & 1) * 4;
	mode |= Bus::read16(LOOPYIO_44) & 0xB;

	uint16_t saved = Bus::read16(LOOPYIO_30) & 0xFF;

	Bus::write16(LOOPYIO_30, saved | 0x0100);               //pulse the latch bit
	Bus::write16(LOOPYIO_44, IO_SETUP_CONSTANT | (uint16_t)mode);
	Bus::write16(LOOPYIO_30, saved);                        //restore the low byte
}
static void hle_fun_1b76_side_effects(uint16_t r4, uint8_t r5)
{
	Bus::write16(0x0900002A, r4);
	Bus::write8(0x09000030, static_cast<uint8_t>(r4 & 0xFF));
	Bus::write8(0x09000032, r5);
	ocpm_write16(0x2A, 0x3E80);
	ocpm_write8(0x25, 0xF8);
	ocpm_write8(0x24, 0xFA);
	uint8_t tstr = Bus::read8(OCPM_BASE + 0x00);
	Bus::write8(OCPM_BASE + 0x00, tstr | 0xE8);
	// Wait skipped
	tstr = Bus::read8(OCPM_BASE + 0x00);
	Bus::write8(OCPM_BASE + 0x00, tstr & 0xF7);
	ocpm_write8(0x24, 0xF8);
}
static void hle_printer_motor_setup()
{
	ocpm_write8(0x22, 0xC1);
	ocpm_write16(0x26, 0x0000);
	ocpm_write8(0x8A, 0x0F);

	// FUN_0000115C: write 0x0C058006 = 4, read 0x0C05D030, shift right 4, mask 7.
	Bus::write16(0x0C058006, 0x0004);
	uint16_t loop30 = Bus::read16(0x0C05D030);
	uint16_t ret = (loop30 >> 4) & 7;

	if (ret == 0)
	{
		hle_fun_1b76_side_effects(0x0064, 0x01);
	}
	else
	{
		uint16_t v = Bus::read16(0x0C05D030);
		Bus::write16(0x0C05D030, v | 0x0100);

		hle_fun_1b76_side_effects(0x021C, 0x01);
		hle_fun_1b76_side_effects(0x0212, 0xFF);

		v = Bus::read16(0x0C05D030);
		Bus::write16(0x0C05D030, v & 0xFEFF);
	}

	Bus::write16(0x0C05D042, 0x5A50);
	ocpm_write16(0x26, 0x0000);
	ocpm_write8(0x8A, 0x00);

	// Final SR: I3-I0 = 0xF
	sh2.sr = (sh2.sr & 0xFF0F) | 0x00F0;
}
void fast_boot()
{
	//The BIOS addresses on-chip registers through GBR; keep it identical.
	sh2.gbr = OCPM_BASE;

	//---- 1. Pin function controller (PFC) ----
	//The BIOS issues 32-bit writes that cover two adjacent 16-bit port control
	//registers; they are split here because the PFC model is 16-bit.
	ocpm_write16(0xC8, 0x0C02);   //PACR1
	ocpm_write16(0xCA, 0xBF99);   //PACR2
	ocpm_write16(0xCC, 0x0080);   //PBCR1
	ocpm_write16(0xCE, 0x1000);   //PBCR2
	ocpm_write16(0xC0, 0x0000);   //PADR
	ocpm_write16(0xC2, 0x0014);   //PBDR
	ocpm_write16(0xC4, 0x0400);   //PAIOR
	ocpm_write16(0xC6, 0x0015);   //PBIOR
	ocpm_write16(0xEE, 0xAFFF);   //CASCR

	//---- 2. ITU channels 2/3/4 (clamp the printer motor control pins) ----
	ocpm_write8(0x18, 0x20);      //TCR2
	ocpm_write8(0x22, 0x20);      //TCR3
	ocpm_write8(0x32, 0x20);      //TCR4
	ocpm_write8(0x19, 0x09);      //TIOR2
	ocpm_write8(0x23, 0x0A);      //TIOR3
	ocpm_write8(0x33, 0x0A);      //TIOR4
	ocpm_write16(0x1E, 0x0001);   //GRA2
	ocpm_write16(0x28, 0x0001);   //GRA3
	ocpm_write16(0x38, 0x0001);   //GRA4
	ocpm_write8(0x00, 0x7C);      //TSTR: start channels 2/3/4
	//The real BIOS polls TSR2 until the (instant in emulation) operation ends.
	ocpm_write8(0x00, 0x60);      //TSTR: stop
	ocpm_write16(0x1C, 0x0000);   //TCNT2
	ocpm_write16(0x26, 0x0000);   //TCNT3
	ocpm_write16(0x36, 0x0000);   //TCNT4
	ocpm_write8(0x1B, 0x78);      //TSR2
	ocpm_write8(0x25, 0x78);      //TSR3
	ocpm_write8(0x35, 0x78);      //TSR4
	ocpm_write16(0xCE, 0x1222);   //PBCR2 updated after the timer run

	//---- 3. Serial / audio configuration ----
	ocpm_write16(0xA0, 0x9000);
	ocpm_write16(0xA2, 0xB9FD);
	ocpm_write16(0xA4, 0xB9B9);
	ocpm_write16(0xA6, 0xA800);
	ocpm_write16(0xA8, 0x5D00);
	ocpm_write16(0xAA, 0x0000);
	ocpm_write16(0xAC, 0x5A80);
	ocpm_write16(0xAE, 0xA508);
	ocpm_write16(0xB0, 0x6900);
	ocpm_write16(0xB2, 0x967A);

	//---- 4. VDP / system registers ----
	Bus::write16(0x0C058006, 0x0004);
	uint16_t vdp_ctrl = 0x20;
	if ((Bus::read16(0x0C05D030) & 0x0001) == 0) vdp_ctrl = 0x21;
	if ((Bus::read16(0x05FFFFC0) & 0x0800) != 0) vdp_ctrl |= 0x04;
	Bus::write16(0x0C058000, vdp_ctrl);
	Bus::write16(0x0C05C000, 0xFFC0);
	ocpm_write16(0xC8, 0x1D02);   //PACR1 updated by the BIOS later
	Bus::write16(0x0C05D020, 0x0001);
	Bus::write16(0x0C060000, 0x0002);

	//---- 5. Memory clears (BIOS DMA-copies the fixed word 0x0000) ----
	//Enable the SH7021 DMAC master (DMAOR DME). The BIOS sets this in its
	//fill routine (FUN_5e4); the game later relies on it for its own DMAs.
	Bus::write16(0x05FFFF48, 0x0001);
	dma_fill16(0x0C040000, 0x8000);             //Tile VRAM
	Bus::write16(0x0C05E002, 0xFFFF);
	Bus::write16(0x0C05E004, 0x0000);
	dma_fill16(0x0C05F000, 0x0200);             //DMA work area
	Bus::write16(0x0C05E002, 0x0000);
	for (uint32_t a = 0x0C050000; a < 0x0C050200; a += 4)
	{
		Bus::write32(a, 0x00000200);           //OAM (CPU long stores)
	}
	dma_fill16(0x0C051000, 0x0100);             //Palette region
	dma_fill16(0x09000000, 0x10000);            //Work RAM bank 0
	dma_fill16(0x09020000, 0x10000);			    //Work RAM bank 1
	dma_fill16(0x09040000, 0x10000);            //Work RAM bank 2
	dma_fill16(0x09060000, 0x10000);            //Work RAM bank 3
	dma_fill16(0x0F000000, 0x0200);             //On-chip object/expansion RAM

	//---- 6. DMA shutdown ----
	ocpm_write16(0x48, 0x0000);                 //DMAOR
	ocpm_write16(0x4E, 0x0000);                 //CHCR0

	//---- 7. BIOS io_setup(1) ----
	hle_io_setup(1);

	//---- 8. Printer motor / paper positioning setup (FUN_00001C2C) ----
	hle_printer_motor_setup();

	//The remaining BIOS steps are mechanical/blocking and need no emulation:
	//  - paper-positioning via the printer motor (0x1C2C),
	//  - waiting for a hardware-ready sensor bit,
	//  - jumping to the cartridge entry point.
	//The CPU start PC is already set to the cartridge entry (0x0E000480).
	Log::info("[HLE] BIOS fast boot complete; cartridge starts at 0x0E000480");
}

//BIOS 0x668: configure the VDP display control register (vdp_ctrl @0x0C058000).
//The packed mode arguments select display fields; the routine keeps vdp_ctrl
//bits 0,2,5 and reprograms bits 1,3,4:
//  r4 (0..3) -> vdp_ctrl bits 3,4
//  r5 (0..1) -> vdp_ctrl bit 1
static bool hle_set_vdp_display_mode(uint32_t)
{
	uint32_t a = sh2.gpr[4] & 3;
	uint32_t b = sh2.gpr[5] & 1;
	//bits 1-3 = display params, bit 4 = pad_scan (always on for HLE)
	uint16_t fields = static_cast<uint16_t>(((a << 2) + b) << 1) | 0x0010;

	uint16_t vdp_ctrl = Bus::read16(0x0C058000);
	vdp_ctrl = (vdp_ctrl & 0x0025) | fields;
	Bus::write16(0x0C058000, vdp_ctrl);

	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x6AC0: reset/initialize the host audio synthesizer (uPD937). It writes a
//fixed command sequence to the audio command port, waiting several vertical
//blanks after each command. In emulation the commands take effect immediately,
//so the vblank waits (and their redundant pad sampling) are skipped.
//  r4 selects the sound program: 0/1/2 add a tail sequence; any other = none.
static bool hle_audio_reset(uint32_t)
{
	constexpr uint32_t AUDIO_CMD = 0x0C080000;

	//Fixed reset command sequence (each followed by a vblank wait on hardware).
	Bus::write16(AUDIO_CMD, 0x0000);
	Bus::write16(AUDIO_CMD, 0x0010);   //16
	Bus::write16(AUDIO_CMD, 0x0000);
	Bus::write16(AUDIO_CMD, 0x0020);   //32

	uint32_t select = sh2.gpr[4];
	switch (select)
	{
	case 0:
		Bus::write16(AUDIO_CMD, 0x0024);   //36
		Bus::write16(AUDIO_CMD, 0x0020);   //32
		break;
	case 1:
		Bus::write16(AUDIO_CMD, 0x0028);   //40
		Bus::write16(AUDIO_CMD, 0x0020);   //32
		Bus::write16(AUDIO_CMD, 0x0022);   //34
		Bus::write16(AUDIO_CMD, 0x0020);   //32
		break;
	case 2:
		Bus::write16(AUDIO_CMD, 0x0022);   //34
		Bus::write16(AUDIO_CMD, 0x0020);   //32
		break;
	default:
		//No tail commands; the real routine returns without the final wait.
		set_pc(sh2.pr);
		sh2.pipeline_valid = false;
		return true;
	}

	//Pure interception: return to where the jsr came from.
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x6B50: program an audio parameter expressed as a single-bit value
//(volume/note field). r4 selects the base range (coarse/fine), r5 the shift
//(level). On hardware each step waits out vertical blanks; these are skipped.
static bool hle_audio_set_param(uint32_t)
{
	constexpr uint32_t AUDIO_CMD = 0x0C080000;

	uint32_t base = (sh2.gpr[4] != 0) ? 0x0040u : 0x0200u;
	uint16_t value = static_cast<uint16_t>(base << sh2.gpr[5]);

	//Hardware sequence: wait, write 0, wait 4 frames, write value, wait 4 frames.
	Bus::write16(AUDIO_CMD, 0x0000);
	Bus::write16(AUDIO_CMD, value);

	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x613C: configure the SCI serial link (MIDI) together with ITU channel 1,
//which clocks it. The routine briefly uses a second register page (GBR base
//0x05FFFE00), then the main page (0x05FFFF00). The short busy-wait is skipped.
static bool hle_sci_timers_config(uint32_t)
{
	constexpr uint32_t PAGE2 = 0x05FFFE00;   //secondary serial-mode registers
	constexpr uint32_t MAIN  = 0x05FFFF00;   //main on-chip peripheral page

	//Secondary page: serial-mode setup bytes.
	Bus::write8(PAGE2 + 0xCA, 0x00);
	Bus::write8(PAGE2 + 0xC8, 0x00);
	Bus::write8(PAGE2 + 0xC9, 15);
	Bus::write8(PAGE2 + 0xCA, 0x00);

	//Main page: ITU channel 1 / SCI control.
	Bus::write8(MAIN + 0x01, Bus::read8(MAIN + 0x01) & 0xFE);
	Bus::write8(MAIN + 0x02, Bus::read8(MAIN + 0x02) & 0xFE);
	Bus::write8(MAIN + 0x05, 0x00);   //TIOR1
	Bus::write8(MAIN + 0x07, 0x00);   //TSR1
	Bus::write8(MAIN + 0x08, 0x00);   //TCNT1
	Bus::write8(MAIN + 0x06, 0x01);   //TIER1
	Bus::write8(MAIN + 0x04, 0x23);   //TCR1
	Bus::write16(MAIN + 0x94, 0x0000);
	Bus::write16(MAIN + 0xB0, 0x0000);

	//(100-iteration busy delay skipped)

	//Secondary page: final mode byte.
	Bus::write8(PAGE2 + 0xCA, 0xA0);

	//GBR is left at the main page, matching the real routine's restore.
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//--- Resource-loading pipeline (0x66D0 batch interpreter, 0x6A48 vblank+batch,
//0x7D96 byte-stream decompression) ---
//These are not yet implemented. The SH-2 HLE safety net (see sh2.cpp) catches
//calls to them and returns to the caller immediately, reporting each target
//once. They are replaced with faithful implementations as needed.

//Execute a BIOS 0x66D0 batch: an array of 16-byte transfer records, terminated
//by a record whose command word is 0xFFFF. On real hardware commands 1 and 4
//queue asynchronous DMA transfers (the descriptor is saved to a work area and
//the copy runs in the background). In emulation the bus is instant, so the
//copies are performed synchronously and the descriptor bookkeeping is skipped.
static void execute_transfer_batch(uint32_t rec)
{
	for (int i = 0; i < 256; i++)
	{
		uint16_t cmd = Bus::read16(rec);
		if (cmd == 0xFFFF)
		{
			break;
		}

		uint32_t src = Bus::read32(rec + 4);
		uint32_t dst = Bus::read32(rec + 8);
		uint16_t cnt = Bus::read16(rec + 12);
		{ FILE* tf=fopen("C:\\Users\\karma\\AppData\\Local\\Temp\\batches.log","a"); if(tf){fprintf(tf,"frame %llu rec%d cmd=%04x src=%08x dst=%08x cnt=%d\n",Video::get_frame_count(),i,cmd,Bus::read32(rec+4),Bus::read32(rec+8),cnt);fclose(tf);} }

		//Commands 0, 1 and 4 describe a word-for-word copy with an incrementing
		//source; they only differ in the real-hardware DMA path/triggering.
		if (cmd == 0 || cmd == 1 || cmd == 4)
		{
			for (uint16_t w = 0; w < cnt; w++)
			{
				Bus::write16(dst + w * 2, Bus::read16(src + w * 2));
			}
		}
		//Command 6 is a fixed-source DMA fill: the same source word is read once
		//and written to every destination word (the source addresses given by
		//the games, e.g. 0x0 or 0xFFFF0000, read back as 0x0000).
		else if (cmd == 6)
		{
			uint16_t fill = Bus::read16(src);
			for (uint16_t w = 0; w < cnt; w++)
			{
				Bus::write16(dst + w * 2, fill);
			}
		}
		else
		{
			Log::warn("[HLE] 0x66D0 record %d uses unsupported command %d (src=0x%08x dst=0x%08x cnt=%d)",
				i, cmd, src, dst, cnt);
		}

		rec += 16;
	}
}

static int s_dbg_c48 = 0, s_dbg_c5a = 0, s_dbg_d0 = 0;

//BIOS 0x66D0: run the transfer batch pointed to by r4.
static bool hle_transfer_batch(uint32_t)
{
	s_dbg_d0++;
	execute_transfer_batch(sh2.gpr[4]);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//Frame barrier: on real hardware the game's frame loop calls the
//"batch after vblank" service (0x6A48/0x6A0E) exactly once per display
//frame. The service samples the pad and immediately runs the transfer
//batch (0x66D0), so its VRAM writes appear in the frame being drawn.
//HLE services return their cycles instantly, so without a barrier the
//game runs several frame steps per display frame (too fast).
//s_barrier_ready becomes true after the first natural frame boundary;
//s_serviced_frame records the display frame of the last service.
static bool s_barrier_ready = false;
static int s_serviced_frame = -1;

//TEMP per-frame hook counters for diagnosing the post-menu black screen.
//Called by the video subsystem at each natural VSYNC start. Reaching the
//first frame boundary tells the 6A48 hook it may block extra same-frame calls.
void notify_frame_boundary()
{
	s_barrier_ready = true;
	FILE* f = fopen("C:\\Users\\karma\\AppData\\Local\\Temp\\hookframes.log", "a");
	if (f)
	{
		fprintf(f, "frame %d c48=%d c5a=%d d0=%d pc=%08x\n",
			Video::get_frame_count(), s_dbg_c48, s_dbg_c5a, s_dbg_d0, sh2.pc);
		fclose(f);
	}
	s_dbg_c48 = s_dbg_c5a = s_dbg_d0 = 0;
}

static bool hle_batch_after_vblank(uint32_t)
{
	s_dbg_c48++;
	int cur_frame = Video::get_frame_count();

	//Frame barrier: on real hardware one frame step occupies one display
	//frame. If a call was already serviced in this frame, fast-forward the
	//timing until the next vblank. This fires the scanline/vblank events (so
	//the frame is still rendered) without running game instructions, exactly
	//like a hardware frame wait.
	if (s_barrier_ready && s_serviced_frame == cur_frame)
	{
		const int blocked_frame = cur_frame;
		Timing::fast_forward_current_until([&]() {
			return Video::get_frame_count() > blocked_frame;
		});
		cur_frame = Video::get_frame_count();

		//fast_forward leaves the CPU slice at 0 cycles, but SH2::run is mid
		//iteration and applies one final cycles_left -= 1 after the hook,
		//which would make it -1 and loop forever. Leave exactly 1 cycle so
		//that final decrement lands on 0 and the slice exits at the boundary.
		sh2.cycles_left = 1;
	}

	//Sample pads now, as the real service does before the transfer.
	for (int i = 0; i < 3; i++)
	{
		uint16_t button_word = Bus::read16(0x0c05d010 + i * 2);
		Bus::write16(0x09000040 + i * 2, button_word);
	}
	{ FILE* pl=fopen("C:\\Users\\karma\\AppData\\Local\\Temp\\padsample.log","a"); if(pl){fprintf(pl,"frame %d via=6A48 w0=%04x w1=%04x w2=%04x\n",cur_frame,Bus::read16(0x09000040),Bus::read16(0x09000042),Bus::read16(0x09000044));fclose(pl);} }
	//Run the transfer batch immediately, just like the real 0x66D0 call:
	//the game places this service at the frame start, so these VRAM writes
	//must be present before the frame's scanlines render. Deferring them to
	//the next vblank added a frame of lag and scattered/missed content.
	execute_transfer_batch(sh2.gpr[4]);
	s_serviced_frame = cur_frame;

	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}
//BIOS 0x437C: decode a fixed-length-coded, RLE-compressed 4bpp bitmap.
//
//Descriptor (r4):
//  word +0  width in pixels
//  word +2  height in pixels
//  long +4  pointer to: byte symbolBits, byte runBits, then a colour lookup table
//  long +8  pointer to the MSB-first packed bitstream (a run of 16-bit words)
//
//Each symbol reads `symbolBits` bits, maps it through the lookup table to a
//colour nibble, and emits one pixel. A following flag bit is 1 for a single
//pixel; 0 introduces a run count (groups of `runBits` bits, terminated by a 1
//bit) and the colour is emitted that many more times. The output packs two
//4-bit pixels per byte: (first << 4) | second.
static void decode_bitmap_437c(uint32_t desc, uint32_t out)
{
	uint16_t width  = Bus::read16(desc + 0);
	uint16_t height = Bus::read16(desc + 2);
	uint32_t params = Bus::read32(desc + 4);
	uint32_t stream = Bus::read32(desc + 8);

	uint8_t b0 = Bus::read8(params);          //symbolBits parameter
	uint8_t run_bits = Bus::read8(params + 1);
	uint32_t lookup = params + 2;             //colour lookup table base

	//symbolBits = number of bits in (b0 - 1); do-while as on the CPU.
	unsigned symbol_bits = 0;
	uint32_t t = b0 - 1;
	do
	{
		t >>= 1;
		symbol_bits++;
	} while (t != 0);

	uint32_t total_pixels = static_cast<uint32_t>(width) * height;

	//MSB-first bit reader over consecutive 16-bit words.
	uint32_t acc = 0;
	unsigned bits_left = 0;
	uint32_t sp = stream;
	auto get_bits = [&](unsigned n) -> uint32_t
	{
		while (bits_left < n)
		{
			acc = (acc << 16) | Bus::read16(sp);
			sp += 2;
			bits_left += 16;
		}
		uint32_t value = (acc >> (bits_left - n)) & ((1u << n) - 1);
		bits_left -= n;
		return value;
	};

	uint32_t dst = out;
	uint32_t pixels = 0;
	uint8_t high_nibble = 0;
	bool have_high = false;
	auto emit_pixel = [&](uint8_t colour)
	{
		colour &= 0x0F;
		if (!have_high)
		{
			high_nibble = colour;
			have_high = true;
		}
		else
		{
			Bus::write8(dst++, static_cast<uint16_t>((high_nibble << 4) | colour));
			have_high = false;
		}
	};

	while (pixels < total_pixels)
	{
		uint32_t symbol = get_bits(symbol_bits);
		uint8_t colour = Bus::read8(lookup + symbol);
		emit_pixel(colour);
		pixels++;

		uint32_t flag = get_bits(1);
		if (flag == 1)
		{
			continue;  //single pixel; read the next symbol
		}

		//Run follows: gather the count (run_bits-bit groups, 1-bit terminator).
		uint32_t count = 0;
		for (;;)
		{
			for (unsigned i = 0; i < run_bits; i++)
			{
				count = (count << 1) | get_bits(1);
			}
			if (get_bits(1) == 1)
			{
				break;
			}
		}

		for (uint32_t k = 0; k < count && pixels < total_pixels; k++)
		{
			emit_pixel(colour);
			pixels++;
		}
	}
}

static bool hle_decode_bitmap(uint32_t)
{
	uint32_t desc = sh2.gpr[4];
	uint32_t out = sh2.gpr[5];
	decode_bitmap_437c(desc, out);	geo_log("decode_bitmap desc=%08x out=%08x", desc, out);

	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x7D96: LZ77-family byte decompressor that builds a working table.
//
//Source header:
//  word +0  total output bytes
//  word +2  size of the literal pool
//  word +4  size of the compressed flag stream
//  bytes +6 ... a small special-code table, then a descriptor stream (after a
//               16-byte skip), then the literal pool, then an LSB-first
//               flag bitstream
//
//Flag codes (read LSB-first):
//  0        literal: one byte copied from the literal pool
//  10       short match; next descriptor byte packs offset/length
//  110      long match; two descriptor bytes pack a 16-bit offset/length
//  111      special; a 4-bit field indexes the special table for either a
//           literal (field>=4) or a short-match descriptor (field<4)
static void decompress_7d96(uint32_t src, uint32_t out)
{
	uint32_t total = Bus::read16(src + 0);
	uint16_t literal_pool_size = Bus::read16(src + 2);
	uint16_t flag_stream_size  = Bus::read16(src + 4);

	uint32_t special_table = src + 6;
	uint32_t descriptors   = src + 6 + 16;
	uint32_t literals      = descriptors + literal_pool_size;
	uint32_t bitstream     = literals + flag_stream_size;

	//LSB-first bit reader over the flag bitstream bytes.
	uint32_t bit_ptr = bitstream;
	uint32_t bit_buf = 0;
	int bits_held = 0;
	auto get_bit = [&]() -> int
	{
		if (bits_held == 0)
		{
			bit_buf = Bus::read8(bit_ptr++);
			bits_held = 8;
		}
		int bit = bit_buf & 1;
		bit_buf >>= 1;
		bits_held--;
		return bit;
	};
	//4-bit field; bits assemble MSB-first (as the CPU's rotcl does).
	auto get_field4 = [&]() -> int
	{
		int value = 0;
		for (int i = 0; i < 4; i++) value = (value << 1) | get_bit();
		return value;
	};

	uint32_t dst = out;
	uint32_t remaining = total;

	//Copy a back-reference. Most are forward (incrementing); long matches may be
	//backward (decrementing) per the descriptor's direction bit.
	auto copy_forward = [&](uint32_t source, uint32_t length)
	{
		for (uint32_t i = 0; i < length; i++)
		{
			Bus::write8(dst++, Bus::read8(source++));
		}
	};
	auto copy_backward = [&](uint32_t source, uint32_t length)
	{
		for (uint32_t i = 0; i < length; i++)
		{
			Bus::write8(dst++, Bus::read8(source));
			source--;
		}
	};

	//Apply a short-match descriptor byte: offset = (d>>3)+1, length = (d&7)+2,
	//with extra 4-bit length groups when the low field is 7.
	auto apply_short_match = [&](uint8_t d) -> uint32_t
	{
		uint32_t length = (d & 7) + 2;
		if ((d & 7) == 7)
		{
			int extra;
			do
			{
				extra = get_field4();
				length += static_cast<uint32_t>(extra);
			} while (extra == 15);
		}
		uint32_t source = dst - ((d >> 3) + 1);
		copy_forward(source, length);
		return length;
	};

	while (remaining > 0)
	{
		if (get_bit() == 0)
		{
			//Literal byte.
			Bus::write8(dst++, Bus::read8(literals++));
			remaining--;
			continue;
		}

		if (get_bit() == 0)
		{
			//Short match.
			uint8_t d = Bus::read8(descriptors++);
			remaining -= apply_short_match(d);
			continue;
		}

		if (get_bit() == 0)
		{
			//Long match: 16-bit packed descriptor.
			uint8_t b1 = Bus::read8(descriptors++);
			uint8_t b2 = Bus::read8(descriptors++);
			uint32_t value = (static_cast<uint32_t>(b1) << 8) | b2;

			uint32_t offset = (value >> 5) + 1;
			bool backward = ((value >> 4) & 1) != 0;

			uint32_t length = (b2 & 0x0F) >> 1;
			if ((b2 & 1) != 0)
			{
				length = (length << 4) | static_cast<uint32_t>(get_field4());
			}
			length += 3;

			uint32_t source = dst - offset;
			if (backward) copy_backward(source, length);
			else copy_forward(source, length);
			remaining -= length;
			continue;
		}

		//Special (111): 4-bit selector into the special table.
		int selector = get_field4();
		uint8_t mapped = Bus::read8(special_table + selector);
		if (selector >= 4)
		{
			Bus::write8(dst++, mapped);   //literal
			remaining--;
		}
		else
		{
			remaining -= apply_short_match(mapped);
		}
	}
}

static bool hle_decompress_7d96(uint32_t)
{
	decompress_7d96(sh2.gpr[4], sh2.gpr[5]);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x61A0 (thunk) + FUNC51 (0x61D8): configure/queue one uPD937 MIDI command.
//
//The thunk resolves a parameter-block pointer from a table (base r7, index r6);
//FUNC51 then applies that 6-byte block while the SCI timer is stopped/restarted:
//  word +0 -> ITU GRA1 (SCI/MIDI timer period)
//  word +2 -> uPD937 command port 0x0C080000
//  byte +4,+5 -> the sound control block
static bool hle_audio_program_setup(uint32_t)
{
	uint32_t ctrl = sh2.gpr[4];
	uint8_t value = static_cast<uint8_t>(sh2.gpr[5]);
	uint32_t index = sh2.gpr[6];
	uint32_t table_base = sh2.gpr[7];

	//Thunk: resolve the parameter-block pointer from the table.
	uint32_t param = Bus::read32(table_base + index * 4);

	//Sound control block header.
	Bus::write32(ctrl + 24, ctrl);
	Bus::write8(ctrl + 11, value);
	Bus::write32(ctrl + 28, table_base);

	const uint32_t GBR = 0x05ffff00;   //TSTR @+0, TCNT1 @+8, GRA1 @+0x0a
	uint8_t tstr = Bus::read8(GBR);
	Bus::write8(GBR, tstr & 0xfe);     //stop SCI timer (TSTR bit0)

	Bus::write16(GBR + 0x0a, Bus::read16(param + 0));   //word0 -> GRA1
	Bus::write16(0x0c080000, Bus::read16(param + 2));   //word1 -> uPD937

	Bus::write32(ctrl + 12, param + 4);
	Bus::write8(ctrl + 8, Bus::read8(param + 4));
	Bus::write8(ctrl + 9, Bus::read8(param + 5));
	Bus::write32(ctrl, param + 6);

	Bus::write8(GBR + 0x08, 0);        //TCNT1 low = 0
	tstr = Bus::read8(GBR);
	Bus::write8(GBR, tstr | 0x01);     //restart SCI timer

	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x61B8 (thunk) + worker 0x6374: trigger a transient sound effect on a
//uPD937 channel without permanently changing its playing sequence.
//The thunk resolves a parameter-block pointer from a table (base r7, index r6);
//the worker backs up the channel's data pointer/word, points it at the block,
//lets the synth driver (0x62B2) consume it, then restores the channel.
//  r4 = channel control block, r5 = value/bound, r6 = table index, r7 = table base.
static bool hle_audio_play_transient(uint32_t)
{
	uint32_t ctrl = sh2.gpr[4];
	uint32_t index = sh2.gpr[6];
	uint32_t table_base = sh2.gpr[7];

	//Thunk (L61C2): r6 = *(table_base + index*4).
	uint32_t param = Bus::read32(table_base + index * 4);

	//Worker 0x6374: shadow the live data pointer (L6380) and 16-bit field (L6384).
	uint32_t data_ptr = Bus::read32(ctrl + 0);
	uint16_t field = Bus::read16(ctrl + 8);
	Bus::write32(ctrl + 16, data_ptr);
	Bus::write16(ctrl + 20, field);

	//Temporarily point the channel at the transient block (L6388-L6392).
	Bus::write8(ctrl + 8, Bus::read8(param + 0));
	Bus::write8(ctrl + 9, Bus::read8(param + 1));
	Bus::write32(ctrl + 0, param + 2);

	//L6390 -> 0x62B2 programs the uPD937 voice registers and consumes the
	//transient stream. No audio-synthesis backend exists yet, so those voice
	//register writes are not modelled; the channel bookkeeping keeps game RAM
	//consistent. Restore the live channel state (L6394-L639E).
	Bus::write16(ctrl + 8, Bus::read16(ctrl + 20));
	Bus::write32(ctrl + 0, Bus::read32(ctrl + 16));

	sh2.gpr[0] = 0;					//driver status: transient submitted
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x2E7C: unsigned integer division (SH compiler helper). r4 / r5 -> r0.
static bool hle_unsigned_divide(uint32_t)
{
	uint32_t dividend = sh2.gpr[4];
	uint32_t divisor  = sh2.gpr[5];
	sh2.gpr[0] = (divisor != 0) ? (dividend / divisor) : 0;
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x5B28: signed integer division (SH compiler helper).
//Takes absolute values, runs the unsigned divide, restores the sign.
//(int32)r4 / (int32)r5 -> r0
static bool hle_signed_divide(uint32_t)
{
	int32_t dividend = static_cast<int32_t>(sh2.gpr[4]);
	int32_t divisor  = static_cast<int32_t>(sh2.gpr[5]);
	int32_t result = (divisor != 0) ? (dividend / divisor) : 0;
	sh2.gpr[0] = static_cast<uint32_t>(result);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x5B52: signed fixed-point scale: (r4 * r5) / r6.
//  signed 16-bit multiply (0x2E68), then signed divide (0x5B28) by r6.
static bool hle_signed_scale(uint32_t)
{
	int32_t a = static_cast<int32_t>(static_cast<int16_t>(sh2.gpr[4]));
	int32_t b = static_cast<int32_t>(static_cast<int16_t>(sh2.gpr[5]));
	int32_t c = static_cast<int32_t>(static_cast<int16_t>(sh2.gpr[6]));
	int32_t product = a * b;
	int32_t result = (c != 0) ? (product / c) : 0;
	sh2.gpr[0] = static_cast<uint32_t>(result);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x6A5A: wait for the next vblank rising edge (VCOUNT bit8), then sample
//the three controller input words into the game's input RAM. 0x6A0E/0x6A48 call
//this internally before running their transfer batch; some scenes also call it
//directly as their once-per-frame sync.
static bool hle_sample_pad(uint32_t)
{
	s_dbg_c5a++;
	//Real BIOS busy-waits for VCOUNT bit8 (the vblank region). Advance emulated
	//time to that boundary: scanline/vblank events fire as needed so the frame
	//still renders, but no game instructions run - exactly like the hardware
	//wait. Advancing from the current timestamp spends only the *remaining*
	//part of this frame, regardless of how much (free) game work preceded us.
	const int start_frame = Video::get_frame_count();
	Timing::fast_forward_current_until([&]() {
		return Video::get_frame_count() > start_frame;
	});

	//fast_forward ends the current slice at 0 cycles mid-iteration; leave one
	//cycle so SH2::run's final cycles_left -= 1 lands on 0 instead of -1.
	sh2.cycles_left = 1;

	//Read 3 controller words from the serial pad port into the game's RAM.
	for (int i = 0; i < 3; i++)
	{
		uint16_t button_word = Bus::read16(0x0c05d010 + i * 2);
		Bus::write16(0x09000040 + i * 2, button_word);
	}
	{ FILE* pl=fopen("C:\\Users\\karma\\AppData\\Local\\Temp\\padsample.log","a"); if(pl){fprintf(pl,"frame %d via=6A5A w0=%04x w1=%04x w2=%04x\n",Video::get_frame_count(),Bus::read16(0x09000040),Bus::read16(0x09000042),Bus::read16(0x09000044));fclose(pl);} }
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}
//BIOS 0x3E9C: width-adaptive block copy. Real hardware picks byte/word/long
//based on the element width (r7) and copies r6 bytes from r4 to r5.
static bool hle_copy_block(uint32_t)
{
	uint32_t src = sh2.gpr[4];
	uint32_t dst = sh2.gpr[5];
	uint32_t bytes = sh2.gpr[6];
	geo_log("copy_block src=%08x dst=%08x bytes=%d width=%d", src,dst,bytes,sh2.gpr[7]&15);
	for (uint32_t i = 0; i < bytes; i++)
		Bus::write8(dst + i, Bus::read8(src + i));
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x3E64: uniform block fill (runs backward on hardware; result identical).
//Fills r5 bytes at r4 with the low byte of r6.
static bool hle_fill_block(uint32_t)
{
	uint32_t dst = sh2.gpr[4];
	uint32_t bytes = sh2.gpr[5];
	uint8_t value = static_cast<uint8_t>(sh2.gpr[6]);
	for (uint32_t i = 0; i < bytes; i++)
		Bus::write8(dst + i, value);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//Positive sine table, Q15: trunc(sin(angle)*32768), angle 0..90.
static const uint16_t POS_SIN[91] = {
	    0,  571, 1143, 1714, 2285, 2855, 3425, 3993, 4560,
	 5126, 5690, 6252, 6812, 7371, 7927, 8480, 9032, 9580,
	10125,10668,11207,11743,12275,12803,13327,13848,14364,
	14876,15383,15886,16383,16876,17364,17846,18323,18794,
	19260,19720,20173,20621,21062,21497,21926,22347,22762,
	23170,23571,23964,24351,24730,25101,25465,25821,26169,
	26509,26841,27165,27481,27788,28087,28377,28659,28932,
	29196,29451,29697,29935,30163,30381,30591,30791,30982,
	31164,31336,31498,31651,31794,31928,32051,32165,32270,
	32364,32449,32523,32588,32643,32688,32723,32748,32763,
	32768,
};

//BIOS 0x4E34: rotate a body of signed coordinate word pairs by angle r7 (degrees)
//and add the translation words at [r6]. Output coordinates are clamped to >=0.
//Returns r0 = number of coordinates clamped.
static bool hle_rotate_points(uint32_t)
{
	uint32_t src = sh2.gpr[4];
	uint32_t dst = sh2.gpr[5];
	uint32_t coeff_ptr = sh2.gpr[6];
	int32_t angle = static_cast<int32_t>(sh2.gpr[7]);	geo_log("rotate src=%08x dst=%08x angle=%d", src, dst, angle);

	//First two header words are copied verbatim; the second is the body word count.
	uint16_t h0 = Bus::read16(src); src += 2;
	Bus::write16(dst, h0); dst += 2;
	uint16_t body_words = Bus::read16(src); src += 2;
	Bus::write16(dst, body_words); dst += 2;
	int32_t remaining = static_cast<int16_t>(body_words);

	//Normalize angle to [0,360).
	while (angle < 0) angle += 360;
	while (angle >= 360) angle -= 360;

	//Select cos/sin coefficients (low-16, NOT-based negatives as on the CPU).
	auto neg = [](uint16_t v) -> uint16_t { return static_cast<uint16_t>(~v); };
	uint16_t cos_q, sin_q;
	if (angle < 90)
	{
		cos_q = POS_SIN[90 - angle];
		sin_q = POS_SIN[angle];
	}
	else if (angle < 180)
	{
		int t = angle - 90;
		cos_q = neg(POS_SIN[t]);
		sin_q = POS_SIN[90 - t];
	}
	else if (angle < 270)
	{
		int t = angle - 180;
		cos_q = neg(POS_SIN[90 - t]);
		sin_q = neg(POS_SIN[t]);
	}
	else
	{
		int t = angle - 270;
		cos_q = POS_SIN[t];
		sin_q = neg(POS_SIN[90 - t]);
	}

	//Translation words C1/C0.
	int16_t c1 = static_cast<int16_t>(Bus::read16(coeff_ptr));
	int16_t c0 = static_cast<int16_t>(Bus::read16(coeff_ptr + 2));

	int32_t clamped = 0;
	while (remaining > 0)
	{
		int16_t x = static_cast<int16_t>(Bus::read16(src)); src += 2;

		//Negative word is a marker copied through (single word, not a pair).
		if (x < 0)
		{
			Bus::write16(dst, static_cast<uint16_t>(x)); dst += 2;
			remaining -= 1;
			continue;
		}

		int32_t r6 = static_cast<int32_t>(x) - (static_cast<int32_t>(c1) << 16);
		int16_t y = static_cast<int16_t>(Bus::read16(src)); src += 2;
		int32_t r7 = static_cast<int32_t>(y) - (static_cast<int32_t>(c0) << 16);

		int32_t out_x = static_cast<int32_t>(static_cast<int16_t>(r6)) * static_cast<int16_t>(cos_q);
		out_x <<= 1;
		out_x += c1;
		int32_t t = static_cast<int16_t>(sin_q) * static_cast<int16_t>(r7);
		t <<= 1;
		out_x -= t;

		int32_t out_y = static_cast<int16_t>(r6) * static_cast<int16_t>(sin_q);
		out_y <<= 1;
		out_y += c0;
		t = static_cast<int16_t>(cos_q) * static_cast<int16_t>(r7);
		t <<= 1;
		out_y += t;

		if (out_x < 0) { out_x = 0; clamped++; }
		Bus::write16(dst, static_cast<uint16_t>(out_x >> 16)); dst += 2;
		if (out_y < 0) { out_y = 0; clamped++; }
		Bus::write16(dst, static_cast<uint16_t>(out_y >> 16)); dst += 2;

		remaining -= 2;
	}

	sh2.gpr[0] = static_cast<uint32_t>(clamped);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x5474: translate a body of signed coordinate word pairs by offsets
//(r6,r7), clamping every result to >=0. Negative words are markers copied
//verbatim. Returns r0 = number of coordinates clamped.
static bool hle_translate_points(uint32_t)
{
	uint32_t src = sh2.gpr[4];
	uint32_t dst = sh2.gpr[5];
	int16_t off_x = static_cast<int16_t>(sh2.gpr[6]);
	int16_t off_y = static_cast<int16_t>(sh2.gpr[7]);	geo_log("translate src=%08x dst=%08x off=(%d,%d)", src, dst, off_x, off_y);

	//First two header words copied verbatim; word1 is the body word count.
	uint16_t h0 = Bus::read16(src); src += 2;
	Bus::write16(dst, h0); dst += 2;
	uint16_t body_words = Bus::read16(src); src += 2;
	Bus::write16(dst, body_words); dst += 2;
	int32_t remaining = static_cast<int16_t>(body_words);

	int32_t clamped = 0;
	while (remaining > 0)
	{
		int16_t a = static_cast<int16_t>(Bus::read16(src)); src += 2;
		remaining -= 1;

		int32_t out_a;
		if (a < 0)
		{
			out_a = a;   //marker copied verbatim
		}
		else
		{
			out_a = static_cast<int32_t>(a) + off_x;
			if (out_a < 0) { out_a = 0; clamped++; }
		}
		Bus::write16(dst, static_cast<uint16_t>(out_a)); dst += 2;

		//A marker is a single word; pairs have a second word.
		if (a < 0) continue;

		int16_t b = static_cast<int16_t>(Bus::read16(src)); src += 2;
		remaining -= 1;
		int32_t out_b = static_cast<int32_t>(b) + off_y;
		if (out_b < 0) { out_b = 0; clamped++; }
		Bus::write16(dst, static_cast<uint16_t>(out_b)); dst += 2;
	}

	if(src==0x0907f3a8 && Video::get_frame_count()>=894 && Video::get_frame_count()<=896){geo_dump("trans-OUT",dst);}
	sh2.gpr[0] = static_cast<uint32_t>(clamped);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//Convert the BIOS packed positive float (7-bit exponent, 24-bit mantissa,
//exponent bias 127, implicit leading 1) to a signed 16-bit fixed scale factor.
static int16_t convert_scale_factor(uint32_t in)
{
	uint32_t shifted = in << 1;
	uint32_t mantissa = (shifted & 0x00ffffff) | 0x01000000;
	int exponent = static_cast<int>(shifted >> 24) - 127;
	while (exponent > 0) { mantissa <<= 1; exponent--; }
	while (exponent < 0) { mantissa >>= 1; exponent++; }
	return static_cast<int16_t>(mantissa >> 16);
}

//BIOS 0x4FE4: scale a body of signed coordinate word pairs around reference
//points (base_x,base_y): out = (coord-base)*scale + base, clamped to >=0.
//Negative words are markers copied verbatim. Returns r0 = clamp count.
static bool hle_scale_points(uint32_t)
{
	uint32_t src = sh2.gpr[4];
	uint32_t dst = sh2.gpr[5];
	int16_t scale_x = convert_scale_factor(sh2.gpr[6]);
	int16_t scale_y = convert_scale_factor(sh2.gpr[7]);	geo_log("scale src=%08x dst=%08x sx_raw=%08x sy_raw=%08x", src, dst, sh2.gpr[6], sh2.gpr[7]);
	static bool s_dumped_in=false;
	if(!s_dumped_in && src==0x0907f3a8 && Video::get_frame_count()>=894 && Video::get_frame_count()<=896){geo_dump("scale-IN",src);s_dumped_in=true;}

	//The reference/base values come from a structure passed as the 5th (stack)
	//argument: at entry its pointer sits on top of the stack.
	uint32_t base_ptr = Bus::read32(sh2.gpr[15]);
	int16_t base_x = static_cast<int16_t>(Bus::read16(base_ptr));
	int16_t base_y = static_cast<int16_t>(Bus::read16(base_ptr + 2));

	//First two header words copied verbatim; word1 = body word count.
	uint16_t h0 = Bus::read16(src); src += 2;
	Bus::write16(dst, h0); dst += 2;
	uint16_t body_words = Bus::read16(src); src += 2;
	Bus::write16(dst, body_words); dst += 2;
	int32_t remaining = static_cast<int16_t>(body_words);

	int32_t clamped = 0;
	while (remaining > 0)
	{
		int16_t x = static_cast<int16_t>(Bus::read16(src)); src += 2;
		remaining -= 1;

		int32_t out_x;
		if (x < 0)
		{
			out_x = x;   //marker
		}
		else
		{
			int32_t dx = static_cast<int32_t>(x) - base_x;
			out_x = static_cast<int16_t>((dx * scale_x) >> 8) + base_x;
			if (out_x < 0) { out_x = 0; clamped++; }
		}
		Bus::write16(dst, static_cast<uint16_t>(out_x)); dst += 2;
		if (x < 0) continue;

		int16_t y = static_cast<int16_t>(Bus::read16(src)); src += 2;
		remaining -= 1;
		int32_t dy = static_cast<int32_t>(y) - base_y;
		int32_t out_y = static_cast<int16_t>((dy * scale_y) >> 8) + base_y;
		if (out_y < 0) { out_y = 0; clamped++; }
		Bus::write16(dst, static_cast<uint16_t>(out_y)); dst += 2;
	}

	sh2.gpr[0] = static_cast<uint32_t>(clamped);
	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x54B8: compile a packed byte coordinate list into the word structure the
//VDP object/seal engine uses.
//
//Stream:  byte0 header; 16-bit section length; then (x,y) coordinate bytes with
//interspersed command bytes (0xFF -> marker 0x8001, 0xFE -> marker 0x8002 then
//three coordinate pairs, 0xFD -> continue). Offsets r7 (x) and r6 (y) are added.
//
//Output:  word0 = decoded header, word1 = entry count (backpatched), then each
//entry as [y][x] with an optional marker word in its third slot.
static bool hle_compile_seal(uint32_t)
{
	//Casio Loopy BIOS 0x54B8: compile a seal polygon command stream.
	//  r4 = source command bytes, r5 = destination word buffer,
	//  r6 = signed y offset, r7 = signed x offset.
	//Returns r0 = source pointer just past the section.
	uint32_t sp = sh2.gpr[4];
	uint32_t dp = sh2.gpr[5];
	const int32_t off_y = static_cast<int32_t>(static_cast<int16_t>(sh2.gpr[6]));
	const int32_t off_x = static_cast<int32_t>(static_cast<int16_t>(sh2.gpr[7]));

	uint8_t b0 = Bus::read8(sp); sp += 1;				//L54B8
	if (b0 == 0)
	{
		sh2.gpr[0] = 0;							//L54BE
		set_pc(sh2.pr);
		sh2.pipeline_valid = false;
		return true;
	}

	const uint16_t word0 = static_cast<uint16_t>(((b0 >> 4) << 8) | (b0 & 15));
	Bus::write16(dp, word0); dp += 2;				//L54CE
	const uint32_t count_slot = dp;					//L54D2
	dp += 2;
	const uint16_t length = static_cast<uint16_t>(
		(Bus::read8(sp) << 8) | Bus::read8(sp + 1));		//L54D6
	sp += 2;
	const uint32_t end = sp + length;				//L54E2

	//Emit one coordinate pair. y is committed immediately; x is written but the
	//destination advances only when advance_after_x is set (the 3rd pair of an FE
	//group defers its x advance to the L5564 step), exactly like the asm.
	auto emit_pair = [&](uint8_t xb, uint8_t yb, bool advance_after_x)
	{
		Bus::write16(dp, static_cast<uint16_t>(static_cast<int32_t>(yb) + off_y)); dp += 2;
		Bus::write16(dp, static_cast<uint16_t>(static_cast<int32_t>(xb) + off_x));
		if (advance_after_x) dp += 2;
	};
	auto read_pair = [&]() -> std::pair<uint8_t, uint8_t>
	{
		uint8_t x = Bus::read8(sp); sp += 1;
		uint8_t y = Bus::read8(sp); sp += 1;
		return {x, y};
	};
	auto finish = [&]()
	{
		//L54FA: dp points at the just-written x slot; count words from count_slot.
		const uint16_t count = static_cast<uint16_t>((dp - count_slot) / 2);
		Bus::write16(count_slot, count);
		sh2.gpr[0] = sp;
	};

	//A command byte that is actually a coordinate is carried here as the next x.
	int pending_x = -1;
	bool done = false;

	while (!done)
	{
		//L54E4 / L54E6: obtain one pair (x may be a carried command byte).
		uint8_t x;
		if (pending_x >= 0) { x = static_cast<uint8_t>(pending_x); pending_x = -1; }
		else { x = Bus::read8(sp); sp += 1; }
		uint8_t y = Bus::read8(sp); sp += 1;
		emit_pair(x, y, false);					//x advance deferred to L5504

		if (!(end > sp)) { finish(); break; }			//L54F6
		dp += 2;							//L5504

		uint8_t cmd = Bus::read8(sp); sp += 1;			//L5506
		if (cmd == 0xFF)
		{
			Bus::write16(dp, 0x8001); dp += 2;		//L5510 (-1 marker)
			continue;
		}
		if (cmd == 0xFE)
		{
			//L551C: -2 marker, three pairs, then a follow-up command (c2).
			for (;;)
			{
				Bus::write16(dp, 0x8002); dp += 2;
				for (int i = 0; i < 3; i++)
				{
					auto [px, py] = read_pair();
					emit_pair(px, py, i < 2);		//3rd pair defers x advance
				}
				if (!(end > sp)) { finish(); done = true; break; }	//L5560
				dp += 2;					//L5564
				uint8_t c2 = Bus::read8(sp); sp += 1;	//L5566
				if (c2 == 0xFF)
				{
					Bus::write16(dp, 0x8001); dp += 2;	//then L54E4
					pending_x = -1;
					break;
				}
				if (c2 == 0xFD) { pending_x = -1; break; }	//straight to L54E4
				sp -= 1;					//c2 starts a new FE group
			}
			continue;
		}
		//Not a marker command: this byte is the x of the next pair (L54E6).
		pending_x = cmd;
	}

	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//BIOS 0x5892: fill a run of 4bpp pixels (nibbles) with one colour.
//  r4 = bitmap byte pointer, r5 = pixel count, r6 = colour nibble,
//  r7&1 = first pixel lands in the current byte's low nibble.
static bool hle_fill_nibbles(uint32_t)
{
	uint32_t p = sh2.gpr[4];
	int32_t count = static_cast<int32_t>(sh2.gpr[5]);
	uint8_t color = static_cast<uint8_t>(sh2.gpr[6]) & 0x0F;

	if (count > 0 && (sh2.gpr[7] & 1))
	{
		uint8_t b = Bus::read8(p);
		Bus::write8(p, static_cast<uint8_t>((b & 0xF0) | color));  //low nibble
		p += 1;
		count -= 1;
	}

	//Aligned to a high nibble: fill whole bytes (two same-colour pixels each).
	uint8_t pair_byte = static_cast<uint8_t>((color << 4) | color);
	int32_t pairs = count / 2;
	for (int32_t i = 0; i < pairs; i++)
		Bus::write8(p + i, pair_byte);
	p += pairs;
	count -= pairs * 2;

	if (count > 0)
	{
		uint8_t b = Bus::read8(p);
		Bus::write8(p, static_cast<uint8_t>((b & 0x0F) | (color << 4)));  //high nibble
	}

	set_pc(sh2.pr);
	sh2.pipeline_valid = false;
	return true;
}

//TEMP diagnostic: dump line-draw arguments.
static bool probe_line(uint32_t addr)
{
	static int n = 0;
	if (n++ < 6)
	{
		uint32_t sp = sh2.gpr[15];
		Log::info("[PROBE line #%d @0x%x] r4=0x%x r5=0x%x r6=0x%x r7=0x%x | [sp]=0x%x [sp+4]=0x%x SR=0x%x",
			n, addr, sh2.gpr[4], sh2.gpr[5], sh2.gpr[6], sh2.gpr[7],
			Bus::read32(sp), Bus::read32(sp + 4), sh2.sr);
	}
	return false;  //let the safety net return immediately
}

static bool probe_d308(uint32_t addr);  //forward
static bool probe_poll(uint32_t addr);  //forward

void install_services()
{
	add_hook(0x00000668, hle_set_vdp_display_mode);
	Log::info("[HLE] installed high-level BIOS service: set_vdp_display_mode @0x668");

	add_hook(0x0000613C, hle_sci_timers_config);
	Log::info("[HLE] installed high-level BIOS service: sci_timers_config @0x613C");

	add_hook(0x00006AC0, hle_audio_reset);
	Log::info("[HLE] installed high-level BIOS service: audio_reset @0x6AC0");

	add_hook(0x00006B50, hle_audio_set_param);
	Log::info("[HLE] installed high-level BIOS service: audio_set_param @0x6B50");

	add_hook(0x000066D0, hle_transfer_batch);
	Log::info("[HLE] installed high-level BIOS service: transfer_batch @0x66D0");

	add_hook(0x00006A48, hle_batch_after_vblank);
	add_hook(0x00006A0E, hle_batch_after_vblank);
	Log::info("[HLE] installed high-level BIOS service: batch_after_vblank @0x6A48/0x6A0E");

	add_hook(0x0000437C, hle_decode_bitmap);
	Log::info("[HLE] installed high-level BIOS service: decode_bitmap @0x437C");

	add_hook(0x00007D96, hle_decompress_7d96);
	Log::info("[HLE] installed high-level BIOS service: decompress @0x7D96");

	add_hook(0x000061A0, hle_audio_program_setup);
	add_hook(0x000061B8, hle_audio_play_transient);
	Log::info("[HLE] installed high-level BIOS service: audio_play_transient @0x61B8");
	Log::info("[HLE] installed high-level BIOS service: audio_program_setup @0x61A0");

	add_hook(0x00002E7C, hle_unsigned_divide);
	Log::info("[HLE] installed high-level BIOS service: unsigned_divide @0x2E7C");

	add_hook(0x00005B28, hle_signed_divide);
	Log::info("[HLE] installed high-level BIOS service: signed_divide @0x5B28");

	add_hook(0x00005B52, hle_signed_scale);
	Log::info("[HLE] installed high-level BIOS service: signed_scale @0x5B52");

	add_hook(0x00006A5A, hle_sample_pad);
	Log::info("[HLE] installed high-level BIOS service: sample_pad @0x6A5A");

	add_hook(0x00003E9C, hle_copy_block);
	Log::info("[HLE] installed high-level BIOS service: copy_block @0x3E9C");

	add_hook(0x00003E64, hle_fill_block);
	Log::info("[HLE] installed high-level BIOS service: fill_block @0x3E64");

	add_hook(0x00004E34, hle_rotate_points);
	Log::info("[HLE] installed high-level BIOS service: rotate_points @0x4E34");

	add_hook(0x00005474, hle_translate_points);
	Log::info("[HLE] installed high-level BIOS service: translate_points @0x5474");

	add_hook(0x00004FE4, hle_scale_points);
	Log::info("[HLE] installed high-level BIOS service: scale_points @0x4FE4");

	add_hook(0x000054B8, hle_compile_seal);
	Log::info("[HLE] installed high-level BIOS service: compile_seal @0x54B8");

	add_hook(0x00005892, hle_fill_nibbles);
	Log::info("[HLE] installed high-level BIOS service: fill_nibbles @0x5892");
}

void remove_services()
{
}

//--- TEMP reference probes (real BIOS) ---
static uint32_t s_ref_dest = 0;
static bool s_ref_captured = false;

static bool probe_437c_entry(uint32_t)
{
	if (!s_ref_captured)
	{
		s_ref_dest = sh2.gpr[5];
		Log::info("[REF] 0x437c entry r4=0x%08x r5=0x%08x", sh2.gpr[4], s_ref_dest);
	}
	return false;  //let the real BIOS execute
}

static bool probe_437c_return(uint32_t)
{
	if (!s_ref_captured)
	{
		s_ref_captured = true;
		uint32_t end = sh2.gpr[5];
		uint32_t n = end - s_ref_dest;
		Log::info("[REF] 0x437c return r5=0x%08x produced %d bytes", end, n);

		std::string line;
		char buf[8];
		for (uint32_t i = 0; i < n && i < 4096; i++)
		{
			snprintf(buf, sizeof buf, "%02x ", Bus::read8(s_ref_dest + i));
			line += buf;
			if ((i & 15) == 15)
			{
				Log::info("[REF] %04x: %s", i - 15, line.c_str());
				line.clear();
			}
		}
		if (!line.empty()) Log::info("[REF] tail: %s", line.c_str());
	}
	return false;
}

static int s_emit_log = 0;
static bool probe_4444_entry(uint32_t)
{
	if (s_emit_log < 12)
	{
		Log::info("[EMIT] #%d T=%d r11=0x%08x r0=0x%02x r5=0x%08x",
			s_emit_log, sh2.sr & 1, sh2.gpr[11], sh2.gpr[0], sh2.gpr[5]);
	}
	s_emit_log++;
	return false;
}

//--- 0x7D96 reference ---
static uint32_t s7_dest = 0;
static bool s7_captured = false;

static bool probe_7d96_entry(uint32_t)
{
	if (!s7_captured)
	{
		s7_dest = sh2.gpr[5];
		Log::info("[REF7] 0x7d96 entry r4=0x%08x r5=0x%08x", sh2.gpr[4], s7_dest);
	}
	return false;
}

static bool probe_7d96_return(uint32_t)
{
	if (!s7_captured)
	{
		s7_captured = true;
		uint32_t end = sh2.gpr[5];
		uint32_t n = end - s7_dest;
		Log::info("[REF7] 0x7d96 return r5=0x%08x produced %d bytes", end, n);

		std::string line;
		char buf[8];
		for (uint32_t i = 0; i < n && i < 4096; i++)
		{
			snprintf(buf, sizeof buf, "%02x ", Bus::read8(s7_dest + i));
			line += buf;
			if ((i & 15) == 15)
			{
				Log::info("[REF7] %04x: %s", i - 15, line.c_str());
				line.clear();
			}
		}
		if (!line.empty()) Log::info("[REF7] tail: %s", line.c_str());
	}
	return false;
}

//TEMP: probe the ctrl006 poll loop.
static bool probe_poll(uint32_t addr)
{
	static int n = 0;
	n++;
	if ((n % 40) == 1 || n == 1)
	{
		Log::info("[PROBE poll #%d @0x%x] status@r11=0x%x want r13=0x%x",
			n, addr, Bus::read16(sh2.gpr[11]), sh2.gpr[13]);
	}
	if (n == 224)
	{
		const char* fn = sh2.bios_present ? "ref_vram_aftercap.bin" : "hle_vram_aftercap.bin";
		FILE* f = fopen(fn, "wb");
		for (uint32_t a = 0x0C000000; a < 0x0C020000; a += 2)
		{
			uint16_t v = Bus::read16(a);
			fputc((v >> 8) & 0xff, f); fputc(v & 0xff, f);
		}
		fclose(f);
		Log::info("[PROBE] dumped post-capture VRAM -> %s", fn);
	}
	return false;
}

//TEMP diagnostic: branch point at cart 0xd308.
static bool probe_d308(uint32_t addr)
{
	static int n = 0;
	++n;
	if (n <= 4)
	{
		uint32_t sp = sh2.gpr[15];
		Log::info("[PROBE d308 #%d] r0(idx)=0x%x r7=0x%x | [sp]=0x%x r4=0x%x r5=0x%x r6=0x%x",
			n, sh2.gpr[0], sh2.gpr[7], Bus::read16(sp),
			sh2.gpr[4], sh2.gpr[5], sh2.gpr[6]);
	}
	if (n == 300)
	{
		FILE* f = fopen("hle_bitmap_vram.bin", "wb");
		uint32_t nz = 0;
		for (uint32_t a = 0x0C000000; a < 0x0C020000; a += 2)
		{
			uint16_t v = Bus::read16(a);
			if (v) ++nz;
			unsigned char lo = v & 0xff, hi = (v >> 8) & 0xff;
			fputc(hi, f); fputc(lo, f);
		}
		fclose(f);
		Log::info("[PROBE] dumped bitmap VRAM, nonzero words=%d / 65536", nz);
		//also dump display/control registers
		Log::info("[PROBE] MODE=%04x ctrl000=%04x BM0BASE=%08x tile=%04x",
			Bus::read16(0x0C058012), Bus::read16(0x0C058000),
			(uint32_t)Bus::read32(0x0C059000), Bus::read16(0x0C05A000));
	}
	return false;
}

static bool probe_log_service(uint32_t a)
{
	Log::info("[SVCCALL] @%05x from_pr=%08x r4=%08x r5=%08x r6=%08x frame=%llu",
		a, sh2.pr, sh2.gpr[4], sh2.gpr[5], sh2.gpr[6], (unsigned long long)Video::get_frame_count());
	return false;  //let real BIOS execute
}

void install_reference_probes()
{
	static const uint32_t svc[] = {
		0x668,0x613c,0x6ac0,0x6b50,0x66d0,0x6a48,0x6a0e,0x437c,0x7d96,
		0x61a0,0x2e7c,0x5b28,0x5b52,0x6a5a,0x3e9c,0x3e64,0x4e34,0x5474,
		0x4fe4,0x54b8,0x5892,0x57f4 };
	for (uint32_t a : svc) add_hook(a, probe_log_service);
	add_hook(0x0E00D308, probe_d308);
	add_hook(0x0E009E42, probe_poll);
	Log::info("[HLE] installed reference probes (%d services + cart probes)", (int)(sizeof(svc)/4));
}

}  // namespace HLE
