# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
"""
Comprehensive cocotb probe suite for tt_um_triple_t_devEclipse2.

Covers:
  1. Overlord / Core Commander mode (ui_in[2:0] == 3'b001):
     - Command ack handshake, register dump/write (all 24 regs), PC read/write,
       command buffer dump/clear, microcode fence & single-step, error detection,
       auto-fence, and error counter telemetry.
  2. Thread Bus mode (ui_in[2:0] == 3'b010):
     - FPGA->ASIC RX (direct execution, macro-ops, 24-bit parameters),
     - ASIC->FPGA TX (TX_SHOW_REG 3'b001, TX_SET_PC 3'b110, TX_MEM_WRITE 3'b101),
     - Bidirectional TX_MEM_READ (3'b100) with High-Z turnaround & trit sanitization.
  3. Microcode ROM Fetch mode (ui_in[2:0] == 3'b101):
     - Sectorized action/param ROM addressing, odd/even micro-op caching,
       early-exit vs 0xFF timeout, hold-until-register-change sync, dual-slot
       pipelining, and conditional micro-branching.
  4. ALU, Masking, and Aliasing:
     - Balanced ternary Add/Sub/Add3, MAC multiply/pump, Trit-Shift (local + ovf),
       single-use & persistent masks, and sub-alias stall auto-resolution.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly

# =============================================================================
# Mode & Protocol Constants
# =============================================================================
MODE_TESTER     = 0b000
MODE_OVERLORD   = 0b001
MODE_THREAD     = 0b010
MODE_UCODE      = 0b101
MODE_STANDALONE = 0b111

# Overlord (Core Commander) opcodes: ui_in[6:3] when ui_in[2:0] == 3'b001
OVL_NOP            = 0b0000
OVL_TOGGLE_DIRECT  = 0b0001
OVL_READ_PC        = 0b0010
OVL_INC_THREAD     = 0b0011  # Deprecated / NOP
OVL_FENCE_OR_STEP  = 0b0100
OVL_WRITE_PC       = 0b0101
OVL_DUMP_CMD_BUF   = 0b0110
OVL_DUMP_REGS      = 0b0111
OVL_UNFENCE        = 0b1000
OVL_CLR_CMD_BUF    = 0b1001
OVL_INC_ERR_ID     = 0b1010
OVL_WRITE_REGS     = 0b1011
OVL_CLR_ERR_FLAG   = 0b1100
OVL_TOGGLE_ERR_UI  = 0b1101
OVL_CLR_ALL_ERRS   = 0b1110

# Thread mode request types: uo_out[6:4] when ui_in[2:0] == 3'b010
REQ_IDLE      = 0b000
REQ_SHOW_REG  = 0b001
REQ_UCODE     = 0b010
REQ_MEM_READ  = 0b100
REQ_MEM_WRITE = 0b101
REQ_SET_PC    = 0b110

# 2-bit Override Balanced Ternary constants (00 = -1, 01 = +1, 10 = 0)
TRIT_NEG = 0b00
TRIT_POS = 0b01
TRIT_ZRO = 0b10

ZERO_1T     = 0b10
ZERO_4T     = 0xAA
ZERO_8T     = 0xAAAA
ZERO_12T    = 0xAAAAAA
POS1_MASK_4T  = 0x55
POS1_MASK_12T = 0x555555
NEG1_MASK_4T  = 0x00
NEG1_MASK_12T = 0x000000


# =============================================================================
# Balanced Ternary & Microcode ROM Helpers
# =============================================================================
def int_to_trits(val: int, width: int) -> int:
    """Encode signed integer into 2-bit override balanced ternary (LSB = trit 0)."""
    max_val = (3 ** width - 1) // 2
    if not (-max_val <= val <= max_val):
        raise ValueError(f"Value {val} out of range for {width}-trit balanced ternary")
    bits = 0
    v = val
    for i in range(width):
        rem = v % 3
        v //= 3
        if rem == 2:
            trit = TRIT_NEG  # -1
            v += 1
        elif rem == 1:
            trit = TRIT_POS  # +1
        else:
            trit = TRIT_ZRO  # 0
        bits |= (trit << (2 * i))
    return bits


def trits_to_int(bits: int, width: int) -> int:
    """Decode 2-bit override balanced ternary into signed integer."""
    val = 0
    power = 1
    for i in range(width):
        t = (bits >> (2 * i)) & 0b11
        if t == TRIT_NEG:
            val -= power
        elif t == TRIT_POS:
            val += power
        # 10 and 11 both decode as 0
        power *= 3
    return val


def sanitize_trits_24b(raw: int) -> int:
    """Apply ASIC's canonical zero sanitization (2'b11 -> 2'b10) across 12 trits."""
    out = 0
    for i in range(12):
        t = (raw >> (2 * i)) & 0b11
        if t == 0b11:
            t = 0b10
        out |= (t << (2 * i))
    return out


def calc_sector_base(mop: int) -> int:
    """Mirror thread_command.calc_sector_base(mop)."""
    if mop <= 31:
        return mop * 2
    elif mop <= 111:
        return 64 + (mop - 32) * 4
    elif mop <= 191:
        return 384 + (mop - 112) * 8
    return 0


def build_ucode_rom(programs: dict) -> dict:
    """
    Pack macro-op microcode programs into the 12-bit byte-addressed ROM format.

    programs: {macro_op_int: [(cmd, p0, p1, p2), ...]}
      - cmd == 0: uses 1 param nibble  (p0)
      - cmd == 1: uses 2 param nibbles (p0, p1)
      - cmd >= 2: uses 3 param nibbles (p0, p1, p2)
    """
    rom = {}
    for mop, uops in programs.items():
        base = calc_sector_base(mop)
        act_off = 0
        prm_off = 0
        # Pad odd-length program with harmless NOP (0000:0000)
        padded = list(uops)
        if len(padded) % 2 == 1:
            padded.append((0x0, 0x0, 0x0, 0x0))

        for i in range(0, len(padded), 2):
            op0 = padded[i]
            op1 = padded[i + 1]
            c0, p0_0, p0_1, p0_2 = op0
            c1, p1_0, p1_1, p1_2 = op1

            # Action byte: high nibble = even op (op0), low nibble = odd op (op1)
            rom[base + (act_off >> 1)] = ((c0 & 0xF) << 4) | (c1 & 0xF)
            act_off += 2

            # Collect parameter nibbles for op0 then op1
            nibs = []
            for c, a, b, d in (op0, op1):
                if c == 0:
                    nibs.append(a & 0xF)
                elif c == 1:
                    nibs.extend([a & 0xF, b & 0xF])
                else:
                    nibs.extend([a & 0xF, b & 0xF, d & 0xF])

            if len(nibs) % 2 == 1:
                nibs.append(0)

            prm_base = 1024 + 3 * base
            for j in range(0, len(nibs), 2):
                rom[min(3999, prm_base + prm_off)] = (nibs[j] << 4) | nibs[j + 1]
                prm_off += 1
    return rom


def safe_int(sig, ctx: str = "") -> int:
    """Convert cocotb LogicArray to int, reporting raw bitstring & context if X/Z."""
    val = sig.value
    try:
        return int(val)
    except ValueError as exc:
        name = getattr(sig, "_name", "sig")
        info = f" ({ctx})" if ctx else ""
        raise ValueError(f"{name}={val.binstr!r}{info}: {exc}") from exc


# =============================================================================
# Chip Protocol Driver (Deadlock-Guarded)
# =============================================================================
class TripleTDriver:
    def __init__(self, dut, ucode_rom=None, memory=None, timeout=200):
        self.dut = dut
        self.ucode_rom = dict(ucode_rom) if ucode_rom else {}
        self.memory = dict(memory) if memory else {}
        self.timeout = timeout
        self.shown_regs = []
        self.pc_updates = []
        self._hs = 0

    async def reset(self):
        self.dut.ena.value = 1
        self.dut.ui_in.value = 0
        self.dut.uio_in.value = 0
        self.dut.rst_n.value = 0
        self._hs = 0
        await ClockCycles(self.dut.clk, 10)
        self.dut.rst_n.value = 1
        await ClockCycles(self.dut.clk, 2)

    # -------------------------------------------------------------------------
    # Overlord / Core Commander Protocol (ui_in[2:0] == 3'b001)
    # -------------------------------------------------------------------------
    def _ovl_ui(self, opcode: int, ack: int = 0) -> int:
        return ((ack & 1) << 7) | ((opcode & 0xF) << 3) | MODE_OVERLORD

    async def ovl_idle(self):
        self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 0)
        await ClockCycles(self.dut.clk, 1)

    async def ovl_cmd(self, opcode: int):
        """Issue a 4-bit Overlord opcode, wait for command_op_ack, then drop opcode to 0."""
        self.dut.ui_in.value = self._ovl_ui(opcode, 0)
        for _ in range(self.timeout):
            await ClockCycles(self.dut.clk, 1)
            if (safe_int(self.dut.uo_out) >> 5) & 1:
                break
        else:
            raise AssertionError(f"Timeout waiting for command_op_ack on opcode {opcode:#06b}")

        # Return opcode to 0000 so FSM can advance and command_op_ack clears
        self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 0)
        for _ in range(self.timeout):
            await ClockCycles(self.dut.clk, 1)
            if ((safe_int(self.dut.uo_out) >> 5) & 1) == 0:
                return
        raise AssertionError(f"Timeout waiting for command_op_ack to clear on opcode {opcode:#06b}")

    async def _ovl_read_bytes(self, count: int) -> list:
        """
        4-phase bus_ready / fpga_ack byte read from Overlord.
        Waits 1 extra cycle after bus_ready rises before sampling uio_out so
        thread_command's 2-stage debug_reg_cache -> dbg_dout pipeline settles
        on Stage 0 of each register.
        """
        out = []
        for idx in range(count):
            # 1. Wait for bus_ready == 1 with fpga_ack == 0
            self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 0)
            for _ in range(self.timeout):
                await ReadOnly()
                if (safe_int(self.dut.uo_out) >> 7) & 1:
                    break
                await ClockCycles(self.dut.clk, 1)
            else:
                raise AssertionError(f"Timeout waiting for bus_ready on Overlord read byte {idx}")

            # 2. Wait 1 extra cycle while keeping fpga_ack=0 for debug_reg_cache -> dbg_dout settle
            await ClockCycles(self.dut.clk, 1)
            await ReadOnly()
            uo_dbg = safe_int(self.dut.uo_out, f"ovl_read idx={idx}")
            reg_id = uo_dbg & 0x1F
            assert safe_int(self.dut.uio_oe, f"ovl_read idx={idx} reg={reg_id}") == 0xFF, (
                f"Expected ASIC to drive uio_oe=0xFF on Overlord read (idx={idx}, reg={reg_id})"
            )
            out.append(safe_int(self.dut.uio_out, f"ovl_read idx={idx} reg={reg_id} byte={idx % 3}"))

            # 3. Assert fpga_ack = 1 and wait for bus_ready == 0
            await ClockCycles(self.dut.clk, 1)
            self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 1)
            for _ in range(self.timeout):
                await ClockCycles(self.dut.clk, 1)
                if ((safe_int(self.dut.uo_out) >> 7) & 1) == 0:
                    break
            else:
                raise AssertionError(f"Timeout waiting for bus_ready drop on Overlord read byte {idx}")

            # 4. Deassert fpga_ack = 0
            self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 0)
        await ClockCycles(self.dut.clk, 1)
        return out

    async def _ovl_write_bytes(self, data: list):
        """
        4-phase bus_ready / fpga_ack byte write to Overlord.
        CRITICAL: uio_in MUST remain stable across the cycle where bus_ready drops
        (when prev_reg_stage != register_read_stage in thread_command) so dbg_din
        is latched cleanly.
        """
        for idx, b in enumerate(data):
            # 1. Wait for bus_ready == 1
            self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 0)
            for _ in range(self.timeout):
                await ReadOnly()
                if (safe_int(self.dut.uo_out) >> 7) & 1:
                    break
                await ClockCycles(self.dut.clk, 1)
            else:
                raise AssertionError(f"Timeout waiting for bus_ready on Overlord write byte {idx}")

            assert safe_int(self.dut.uio_oe) == 0x00, "Expected uio_oe=0x00 during Overlord write"

            # 2. Drive data byte and assert fpga_ack = 1 until bus_ready drops
            await ClockCycles(self.dut.clk, 1)
            self.dut.uio_in.value = b & 0xFF
            self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 1)
            for _ in range(self.timeout):
                await ClockCycles(self.dut.clk, 1)
                if ((safe_int(self.dut.uo_out) >> 7) & 1) == 0:
                    break
            else:
                raise AssertionError(f"Timeout waiting for bus_ready drop on Overlord write byte {idx}")

            # 3. Lower fpga_ack = 0 while HOLDING uio_in = b for 1 clock so thread_command latches it
            self.dut.ui_in.value = self._ovl_ui(OVL_NOP, 0)
            await ClockCycles(self.dut.clk, 1)

        # Extra cycle for final handover completion
        await ClockCycles(self.dut.clk, 1)

    async def ovl_read_pc(self) -> int:
        await self.ovl_cmd(OVL_READ_PC)
        b = await self._ovl_read_bytes(3)
        return b[0] | (b[1] << 8) | (b[2] << 16)

    async def ovl_write_pc(self, val24: int):
        """Write 24-bit PC via Overlord. Note: sets microcode_fence=1 and pc_tx_pending=1."""
        await self.ovl_cmd(OVL_WRITE_PC)
        await self._ovl_write_bytes([val24 & 0xFF, (val24 >> 8) & 0xFF, (val24 >> 16) & 0xFF])

    async def ovl_dump_regs(self) -> dict:
        """Dump all 24 Overlord-visible registers (addresses 1..24)."""
        await self.ovl_cmd(OVL_DUMP_REGS)
        raw = await self._ovl_read_bytes(24 * 3)
        regs = {}
        for addr in range(1, 25):
            off = (addr - 1) * 3
            regs[addr] = raw[off] | (raw[off + 1] << 8) | (raw[off + 2] << 16)
        return regs

    async def ovl_write_regs(self, reg_map: dict):
        """Write all 24 registers (addresses 1..24). Missing entries default to ZERO_12T / safe defaults."""
        defaults = {a: ZERO_12T for a in range(1, 21)}
        defaults[21] = POS1_MASK_12T
        defaults[22] = (0 << 16) | (POS1_MASK_4T << 8) | (TRIT_POS << 4)
        defaults[23] = (ZERO_1T << 20)
        defaults[24] = 0x000000
        defaults.update(reg_map)

        payload = []
        for addr in range(1, 25):
            v = defaults[addr]
            payload.extend([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])

        await self.ovl_cmd(OVL_WRITE_REGS)
        await self._ovl_write_bytes(payload)

    async def ovl_dump_cmd_buf(self) -> tuple:
        """Returns (active_macro_op, next_macro_op)."""
        await self.ovl_cmd(OVL_DUMP_CMD_BUF)
        b = await self._ovl_read_bytes(2)
        return (b[0], b[1])

    async def ovl_clear_cmd_buf(self):
        await self.ovl_cmd(OVL_CLR_CMD_BUF)
        # Wait until controller_state returns to state_idle (uo_out[2:0] == 0)
        for _ in range(self.timeout):
            await ReadOnly()
            if (safe_int(self.dut.uo_out) & 0x07) == 0:
                await ClockCycles(self.dut.clk, 1)
                return
            await ClockCycles(self.dut.clk, 1)
        raise AssertionError("Timeout waiting for OVL_CLR_CMD_BUF to finish")

    async def ovl_read_error_counters(self) -> dict:
        """Toggle showing_errors ON, step IDs 0..5, return dict for IDs 2..5, then turn OFF."""
        await self.ovl_cmd(OVL_TOGGLE_ERR_UI)
        counts = {}
        for err_id in range(6):
            await ReadOnly()
            assert safe_int(self.dut.uio_oe) == 0xFF, "Expected uio_oe=0xFF while showing_errors"
            if err_id in (2, 3, 4, 5):
                counts[err_id] = safe_int(self.dut.uio_out)
            await ClockCycles(self.dut.clk, 1)
            if err_id < 5:
                await self.ovl_cmd(OVL_INC_ERR_ID)
        await self.ovl_cmd(OVL_TOGGLE_ERR_UI)
        return counts

    # -------------------------------------------------------------------------
    # Microcode ROM Fetch Protocol (ui_in[2:0] == 3'b101)
    # -------------------------------------------------------------------------
    async def serve_microcode_rom(self):
        """
        Switch to Mode 3'b101 and serve ROM bytes on dbus = {uio_in[2:0], ui_in[7:3]}
        until cycle_threads (uo_out[7]) goes HIGH.
        CRITICAL: Never leave Mode 3'b101 before cycle_threads == 1 (protects ST_WAIT_PRM2).
        """
        self.dut.ui_in.value = MODE_UCODE
        self.dut.uio_in.value = 0

        for _ in range(self.timeout * 4):
            await ReadOnly()
            uo = safe_int(self.dut.uo_out)
            uio_o = safe_int(self.dut.uio_out)
            oe = safe_int(self.dut.uio_oe)
            assert oe == 0xF8, f"Expected uio_oe=0xF8 in ucode mode, got {oe:#04x}"

            if (uo >> 7) & 1:  # cycle_threads == 1 -> fetch complete!
                await ClockCycles(self.dut.clk, 1)
                return

            addr = ((uo & 0x7F) << 5) | ((uio_o >> 3) & 0x1F)
            rom_byte = self.ucode_rom.get(addr, 0x00)
            await ClockCycles(self.dut.clk, 1)
            self.dut.ui_in.value = ((rom_byte & 0x1F) << 3) | MODE_UCODE
            self.dut.uio_in.value = (rom_byte >> 5) & 0x07

        raise AssertionError("Timeout serving Microcode ROM (cycle_threads never asserted)")

    # -------------------------------------------------------------------------
    # Thread Bus Protocol (ui_in[2:0] == 3'b010)
    # -------------------------------------------------------------------------
    def _thr_ui(self, phase_3b: int = 0, is_cmd: int = 0, hs: int = 0) -> int:
        self._hs = hs & 1
        return ((phase_3b & 0x7) << 5) | ((is_cmd & 1) << 4) | (self._hs << 3) | MODE_THREAD

    async def enter_thread_idle(self):
        self.dut.ui_in.value = self._thr_ui(0, 0, 0)
        self.dut.uio_in.value = 0
        await ClockCycles(self.dut.clk, 1)

    async def _service_asic_tx_3b(self, enter_tx: bool = True) -> int:
        """Handle TX_SHOW_REG (001) or TX_SET_PC (110): 3 bytes ASIC -> FPGA."""
        if enter_tx:
            # Enter BUS_ASIC_TX via hs_rise (0 -> 1)
            self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=1)
            await ClockCycles(self.dut.clk, 1)

        vals = []
        for idx in range(3):
            self.dut.ui_in.value = self._thr_ui(phase_3b=idx, is_cmd=0, hs=1)
            await ReadOnly()
            assert safe_int(self.dut.uio_oe) == 0xFF, "Expected ASIC driving uio_oe=0xFF in BUS_ASIC_TX"
            vals.append(safe_int(self.dut.uio_out))
            await ClockCycles(self.dut.clk, 1)

        # Complete via phase=111 + hs_toggle (1 -> 0)
        self.dut.ui_in.value = self._thr_ui(phase_3b=0b111, is_cmd=0, hs=0)
        await ClockCycles(self.dut.clk, 1)
        self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=0)
        await ClockCycles(self.dut.clk, 1)
        return vals[0] | (vals[1] << 8) | (vals[2] << 16)

    async def _service_mem_write(self, enter_tx: bool = True):
        """Handle TX_MEM_WRITE (101): 5-byte addr then 3-byte data ASIC -> FPGA."""
        if enter_tx:
            # Enter BUS_ASIC_TX (mem_data_phase=0)
            self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=1)
            await ClockCycles(self.dut.clk, 1)

        addr_bytes = []
        for idx in range(5):
            self.dut.ui_in.value = self._thr_ui(phase_3b=idx, is_cmd=0, hs=1)
            await ReadOnly()
            assert safe_int(self.dut.uio_oe) == 0xFF
            addr_bytes.append(safe_int(self.dut.uio_out))
            await ClockCycles(self.dut.clk, 1)

        addr24 = addr_bytes[0] | (addr_bytes[1] << 8) | (addr_bytes[2] << 16)
        ext16 = addr_bytes[3] | (addr_bytes[4] << 8)

        # Advance to mem_data_phase=1 via phase=111 + hs_toggle (1 -> 0)
        self.dut.ui_in.value = self._thr_ui(phase_3b=0b111, is_cmd=0, hs=0)
        await ClockCycles(self.dut.clk, 1)

        data_bytes = []
        for idx in range(3):
            self.dut.ui_in.value = self._thr_ui(phase_3b=idx, is_cmd=0, hs=0)
            await ReadOnly()
            assert safe_int(self.dut.uio_oe) == 0xFF
            data_bytes.append(safe_int(self.dut.uio_out))
            await ClockCycles(self.dut.clk, 1)

        data24 = data_bytes[0] | (data_bytes[1] << 8) | (data_bytes[2] << 16)
        self.memory[(ext16, addr24)] = data24

        # Finish via phase=111 + hs_toggle (0 -> 1), then return hs=0
        self.dut.ui_in.value = self._thr_ui(phase_3b=0b111, is_cmd=0, hs=1)
        await ClockCycles(self.dut.clk, 1)
        self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=0)
        await ClockCycles(self.dut.clk, 1)

    async def _service_mem_read(self, enter_tx: bool = True):
        """Handle TX_MEM_READ (100): 5-byte addr -> High-Z Turnaround -> 3-byte data FPGA -> ASIC."""
        if enter_tx:
            # Enter BUS_ASIC_TX
            self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=1)
            await ClockCycles(self.dut.clk, 1)

        addr_bytes = []
        for idx in range(5):
            self.dut.ui_in.value = self._thr_ui(phase_3b=idx, is_cmd=0, hs=1)
            await ReadOnly()
            assert safe_int(self.dut.uio_oe) == 0xFF
            addr_bytes.append(safe_int(self.dut.uio_out))
            await ClockCycles(self.dut.clk, 1)

        addr24 = addr_bytes[0] | (addr_bytes[1] << 8) | (addr_bytes[2] << 16)
        ext16 = addr_bytes[3] | (addr_bytes[4] << 8)
        data24 = self.memory.get((ext16, addr24), ZERO_12T)

        # Enter BUS_MEM_TURN via phase=111 + hs_toggle (1 -> 0)
        self.dut.ui_in.value = self._thr_ui(phase_3b=0b111, is_cmd=0, hs=0)
        await ClockCycles(self.dut.clk, 1)

        # Wait 1 cycle for High-Z dead time -> data_phase becomes 2'b00
        for _ in range(self.timeout):
            await ReadOnly()
            uo = safe_int(self.dut.uo_out)
            assert safe_int(self.dut.uio_oe) == 0x00, "Expected High-Z (uio_oe=0) in BUS_MEM_TURN"
            if ((uo >> 1) & 0x3) == 0b00:
                break
            await ClockCycles(self.dut.clk, 1)
        else:
            raise AssertionError("Timeout waiting for data_phase==00 in BUS_MEM_TURN")

        # Acknowledge turnaround via hs_toggle (0 -> 1) -> enters BUS_MEM_RD
        await ClockCycles(self.dut.clk, 1)
        self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=1)
        await ClockCycles(self.dut.clk, 1)

        # Stream 3 bytes FPGA -> ASIC on hs_toggle
        hs = 1
        for idx in range(3):
            hs ^= 1
            self.dut.uio_in.value = (data24 >> (8 * idx)) & 0xFF
            self.dut.ui_in.value = self._thr_ui(phase_3b=idx, is_cmd=0, hs=hs)
            await ClockCycles(self.dut.clk, 1)

        # Finalize via phase=111 + hs_toggle
        hs ^= 1
        self.dut.ui_in.value = self._thr_ui(phase_3b=0b111, is_cmd=0, hs=hs)
        await ClockCycles(self.dut.clk, 1)

        # Ensure hs returns to 0 in BUS_IDLE
        self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=0)
        await ClockCycles(self.dut.clk, 1)

    async def service_once(self) -> int:
        """Check req_type in Thread Mode and service 1 request if pending. Returns req_type handled."""
        self.dut.ui_in.value = self._thr_ui(0, 0, 0)
        await ReadOnly()
        req = (safe_int(self.dut.uo_out) >> 4) & 0x7
        await ClockCycles(self.dut.clk, 1)

        if req == REQ_UCODE:
            await self.serve_microcode_rom()
            await self.enter_thread_idle()
        elif req == REQ_SHOW_REG:
            val = await self._service_asic_tx_3b()
            self.shown_regs.append(val)
        elif req == REQ_SET_PC:
            val = await self._service_asic_tx_3b()
            self.pc_updates.append(val)
        elif req == REQ_MEM_WRITE:
            await self._service_mem_write()
        elif req == REQ_MEM_READ:
            await self._service_mem_read()
        return req

    async def run_until_quiet(self, quiet_cycles: int = 8, max_steps: int = 400):
        """Service all bus/microcode requests until chip stays REQ_IDLE & !cur_full for quiet_cycles."""
        await self.enter_thread_idle()
        idle_streak = 0
        for _ in range(max_steps):
            self.dut.ui_in.value = self._thr_ui(0, 0, 0)
            await ReadOnly()
            uo = safe_int(self.dut.uo_out)
            req = (uo >> 4) & 0x7
            full = (uo >> 3) & 1

            if req != REQ_IDLE:
                idle_streak = 0
                await ClockCycles(self.dut.clk, 1)
                await self.service_once()
            elif full:
                idle_streak = 0
                await ClockCycles(self.dut.clk, 1)
            else:
                idle_streak += 1
                await ClockCycles(self.dut.clk, 1)
                if idle_streak >= quiet_cycles:
                    return
        uo = safe_int(self.dut.uo_out)
        raise AssertionError(
            f"Timeout in run_until_quiet (req={(uo >> 4) & 7:#05b}, full={(uo >> 3) & 1}, dp={(uo >> 1) & 3:#04b})"
        )

    async def send_rx_frame(self, is_cmd: int, param_idx: int, payload: list):
        """
        Execute an FPGA -> ASIC RX transfer (1-byte cmd, 2-byte direct payload, or 3-byte param).
        Automatically services any pending ASIC requests before starting, and retries cleanly
        if the ASIC asserts a request (e.g., REQ_UCODE) on the same clock edge as hs_rise.
        """
        await self.enter_thread_idle()
        for _ in range(self.timeout):
            await ReadOnly()
            uo = safe_int(self.dut.uo_out)
            req = (uo >> 4) & 0x7
            full = (uo >> 3) & 1
            if req != REQ_IDLE:
                await ClockCycles(self.dut.clk, 1)
                await self.service_once()
                continue
            if full:
                await ClockCycles(self.dut.clk, 1)
                continue

            # Step 1: Attempt BUS_IDLE -> BUS_RX_WAIT via hs_rise (0 -> 1)
            await ClockCycles(self.dut.clk, 1)
            self.dut.ui_in.value = self._thr_ui(phase_3b=param_idx, is_cmd=is_cmd, hs=1)
            await ClockCycles(self.dut.clk, 1)
            await ReadOnly()
            uo2 = safe_int(self.dut.uo_out)
            dp = (uo2 >> 1) & 0x3
            req2 = (uo2 >> 4) & 0x7
            if dp == 0b00:
                break

            # Collision: ASIC asserted a request on the exact hs_rise cycle!
            if req2 in (REQ_SHOW_REG, REQ_SET_PC, REQ_MEM_READ, REQ_MEM_WRITE):
                # hs_rise already transitioned logic_container into BUS_ASIC_TX
                await ClockCycles(self.dut.clk, 1)
                if req2 == REQ_SHOW_REG:
                    self.shown_regs.append(await self._service_asic_tx_3b(enter_tx=False))
                elif req2 == REQ_SET_PC:
                    self.pc_updates.append(await self._service_asic_tx_3b(enter_tx=False))
                elif req2 == REQ_MEM_WRITE:
                    await self._service_mem_write(enter_tx=False)
                elif req2 == REQ_MEM_READ:
                    await self._service_mem_read(enter_tx=False)
            else:
                # REQ_UCODE (or cur_full) kept logic_container in BUS_IDLE; lower hs=0 and retry
                await ClockCycles(self.dut.clk, 1)
                await self.enter_thread_idle()
        else:
            uo = safe_int(self.dut.uo_out)
            raise AssertionError(
                f"Timeout waiting for RX ready (req={(uo >> 4) & 7:#05b}, full={(uo >> 3) & 1}, dp={(uo >> 1) & 3:#04b})"
            )

        # Step 2: BUS_RX_WAIT -> BUS_RX_STRM via hs_fall (1 -> 0)
        await ClockCycles(self.dut.clk, 1)
        self.dut.ui_in.value = self._thr_ui(phase_3b=param_idx, is_cmd=is_cmd, hs=0)
        await ClockCycles(self.dut.clk, 1)

        # Step 3: Stream bytes on hs_toggle
        hs = 0
        for b in payload:
            hs ^= 1
            self.dut.uio_in.value = b & 0xFF
            self.dut.ui_in.value = self._thr_ui(phase_3b=param_idx, is_cmd=is_cmd, hs=hs)
            await ClockCycles(self.dut.clk, 1)

        await ReadOnly()
        dp = (safe_int(self.dut.uo_out) >> 1) & 0x3
        assert dp == 0b11, f"Expected data_phase=11 after RX frame completion, got {dp:#04b}"

        # Step 4: Return hs to 0 in BUS_IDLE so next transfer can generate hs_rise
        await ClockCycles(self.dut.clk, 1)
        self.dut.ui_in.value = self._thr_ui(phase_3b=0, is_cmd=0, hs=0)
        await ClockCycles(self.dut.clk, 1)

    async def send_direct_uop(self, cmd: int, p0: int, p1: int = 0, p2: int = 0, wait_quiet: bool = True):
        """
        Send a direct-execution micro-op:
          Header (is_cmd=1): 11aaaa00
          Payload (is_cmd=0, 2 bytes): [bbbbcccc, dddd0000]
        """
        hdr = 0xC0 | ((cmd & 0xF) << 2)
        b0 = ((p0 & 0xF) << 4) | (p1 & 0xF)
        b1 = (p2 & 0xF) << 4
        await self.send_rx_frame(is_cmd=1, param_idx=0, payload=[hdr])
        await self.send_rx_frame(is_cmd=0, param_idx=0, payload=[b0, b1])
        if wait_quiet:
            await self.run_until_quiet(quiet_cycles=6)

    async def send_macro_op(self, opcode: int, params: list = None):
        """
        Send an 8-bit macro-op (< 192) followed by its 24-bit parameters counted down
        from N..1 (where param index k writes cmd_reg_{k-1}).
        """
        params = list(params) if params else []
        assert 0 <= opcode < 192, f"Illegal macro-op {opcode}"
        assert len(params) <= 6, "At most 6 macro-op parameters"

        await self.send_rx_frame(is_cmd=1, param_idx=len(params), payload=[opcode])
        # Protocol counts down N..1 so idx=1 marks 2nd slot full
        for idx in range(len(params), 0, -1):
            v = params[idx - 1]
            await self.send_rx_frame(
                is_cmd=0,
                param_idx=idx,
                payload=[v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF],
            )


# =============================================================================
# Probe Tests
# =============================================================================
@cocotb.test()
async def test_01_reset_and_overlord_registers(dut):
    """Probe reset, unused modes, Overlord 24-register R/W, and PC R/W + TX_SET_PC sync."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()

    # Check placeholder modes (3'b000, 3'b111) stay silent & High-Z
    for m in (MODE_TESTER, MODE_STANDALONE):
        dut.ui_in.value = m
        await ClockCycles(dut.clk, 2)
        assert safe_int(dut.uo_out) == 0
        assert safe_int(dut.uio_oe) == 0

    # Clear command buffer via Overlord
    await drv.ovl_clear_cmd_buf()

    # Write distinct ternary-valid patterns across all 24 Overlord registers
    expected = {
        1: int_to_trits(1234, 12),      # sp0
        2: int_to_trits(-5678, 12),     # sp1
        3: 0xAA5500,                    # spf ({spf_trib2, spf_trib1, spf_trib0})
        4: 0x55AA00,                    # mac ({mac_d, mac_b, mac_a})
        5: int_to_trits(42, 12),        # PC
        6: int_to_trits(100, 12),       # gp_reg_6
        7: int_to_trits(-200, 12),      # gp_reg_7
        8: int_to_trits(300, 12),       # gp_reg_8
        9: int_to_trits(11, 12),        # active cmd_reg_0
        10: int_to_trits(22, 12),       # active cmd_reg_1
        11: int_to_trits(33, 12),       # active cmd_reg_2
        12: int_to_trits(44, 12),       # active cmd_reg_3
        13: int_to_trits(55, 12),       # active cmd_reg_4
        14: int_to_trits(66, 12),       # active cmd_reg_5
        15: int_to_trits(-11, 12),      # inactive cmd_reg_0
        16: int_to_trits(-22, 12),      # inactive cmd_reg_1
        17: int_to_trits(-33, 12),      # inactive cmd_reg_2
        18: int_to_trits(-44, 12),      # inactive cmd_reg_3
        19: int_to_trits(-55, 12),      # inactive cmd_reg_4
        20: int_to_trits(-66, 12),      # inactive cmd_reg_5
        21: POS1_MASK_12T,              # am_buff_12t
        22: (0 << 16) | (POS1_MASK_4T << 8) | (TRIT_POS << 4),
        23: (ZERO_1T << 20) | (0 << 8) | 0,
        24: 0x000000,                   # macroop_register
    }
    await drv.ovl_write_regs(expected)
    readback = await drv.ovl_dump_regs()

    for addr, exp_val in expected.items():
        assert readback[addr] == exp_val, (
            f"Reg {addr} mismatch: got {readback[addr]:#08x}, expected {exp_val:#08x}"
        )

    # Probe Set PC (0101) -> Read PC (0010) -> Unfence -> TX_SET_PC (req_type 110)
    new_pc = int_to_trits(777, 12)
    await drv.ovl_write_pc(new_pc)
    assert await drv.ovl_read_pc() == new_pc, "PC readback mismatch after OVL_WRITE_PC"

    # Unfence so thread_command emits TX_SET_PC (3'b110), then service it cleanly
    await drv.ovl_cmd(OVL_UNFENCE)
    await drv.run_until_quiet()
    assert new_pc in drv.pc_updates, f"Expected TX_SET_PC update {new_pc:#08x}, got {drv.pc_updates}"
    dut._log.info("test_01_reset_and_overlord_registers PASSED")


@cocotb.test()
async def test_02_direct_exec_alu_and_show_reg(dut):
    """Probe Direct Execution: 4t imm load, Add, Sub, Add3, Trit-Shift, MAC, and TX_SHOW_REG."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    # Initialize scratchpads, GP regs, MAC ladder, carry, and masks via direct exec
    await drv.send_direct_uop(0x0, 0x7)  # Clear scratchpads (sp0, sp1)
    await drv.send_direct_uop(0x0, 0x8)  # Clear GP regs
    await drv.send_direct_uop(0x0, 0x1)  # Flush MAC ladder
    await drv.send_direct_uop(0x0, 0x2)  # Clear carry
    await drv.send_direct_uop(0x0, 0x3)  # Reset masks/aliases

    # Load 4-trit immediates into sp0_trib0 (addr 0) = +10, sp0_trib1 (addr 1) = +5, sp0_trib2 (addr 2) = -3
    for addr4, val in [(0, 10), (1, 5), (2, -3)]:
        t4 = int_to_trits(val, 4)
        await drv.send_direct_uop(0xB, (t4 >> 4) & 0xF, t4 & 0xF, (addr4 << 1) | 0)

    # 1. Add: sp1_trib0 (addr 3) = sp0_trib0 (10) + sp0_trib1 (5) = +15
    await drv.send_direct_uop(0x3, (0 << 1) | 0, (1 << 1) | 0, (3 << 1) | 0)

    # 2. Sub: clear carry, sp1_trib1 (addr 4) = sp0_trib0 (10) - sp0_trib1 (5) = +5
    await drv.send_direct_uop(0x0, 0x2)
    await drv.send_direct_uop(0x3, (0 << 1) | 1, (1 << 1) | 0, (4 << 1) | 0)

    # 3. Add3: clear carry, sp1_trib2 (addr 5) = sp0_trib0 (10) + sp0_trib1 (5) + sp0_trib2 (-3) = +12
    # Packed aaab bbcc cddd: a=0 (000), b=1 (001), c=2 (010), d=5 (101)
    await drv.send_direct_uop(0x0, 0x2)
    p0 = (0 << 1) | 0          # 0000
    p1 = (0b01 << 2) | 0b01    # 0101
    p2 = (0 << 3) | 5          # 0101
    await drv.send_direct_uop(0x5, p0, p1, p2)

    # Verify sp1 via TX_SHOW_REG (0001:1101, addr=2 -> sp1)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    assert len(drv.shown_regs) == 1, "Expected 1 TX_SHOW_REG packet"
    sp1_val = drv.shown_regs[0]
    assert trits_to_int(sp1_val & 0xFF, 4) == 15, "Add result mismatch"
    assert trits_to_int((sp1_val >> 8) & 0xFF, 4) == 5, "Sub result mismatch"
    assert trits_to_int((sp1_val >> 16) & 0xFF, 4) == 12, "Add3 result mismatch"

    # 4. Trit-Shift: load flag0 (1t addr 0) = -1 (shift amt = 1 trit)
    # 0010:0001 -> p1 = {trit[1:0], 2'b00}, p2 = {addr_1t[2:0], mask=0}
    await drv.send_direct_uop(0x2, 0x1, (TRIT_NEG << 2) | 0b00, (0 << 1) | 0)
    # Shift sp0_trib0 (+10 = [+1, 0, +1, 0] -> wait: 10 = 1*9 + 0*3 + 1*1 = [0, +1, 0, +1])
    # Left shift by 1 (dir_bit=0, local window=0) -> multiplies by 3 -> +30!
    await drv.send_direct_uop(0x6, (0 << 1) | 0, (0 << 1) | 0, (3 << 1) | 0)
    # Right shift sp0_trib0 (+10) by 1 (dir_bit=1, local window=0) -> floor_div_trit -> +3!
    await drv.send_direct_uop(0x6, (0 << 1) | 0, (0 << 1) | 0, (4 << 1) | 1)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1_shift = drv.shown_regs[0]
    assert trits_to_int(sp1_shift & 0xFF, 4) == 30, f"Left shift mismatch: {trits_to_int(sp1_shift & 0xFF, 4)}"
    assert trits_to_int((sp1_shift >> 8) & 0xFF, 4) == 3, f"Right shift mismatch: {trits_to_int((sp1_shift >> 8) & 0xFF, 4)}"

    # 5. MAC Seed Multiply (0010:1111, p1[3]=1 -> B,A = sp0_trib0 * sp0_trib1 = 10 * 5 = +50)
    await drv.send_direct_uop(0x2, 0xF, (1 << 3) | 0, 1)
    # Read mac_a (00) -> sp1_trib0 (idx 0 -> 4t addr 3), mac_b (01) -> sp1_trib1 (idx 1 -> 4t addr 4)
    await drv.send_direct_uop(0x1, 0x1, (0 << 2) | 0b00)
    await drv.send_direct_uop(0x1, 0x1, (1 << 2) | 0b01)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    mac_prod = drv.shown_regs[0] & 0xFFFF
    assert trits_to_int(mac_prod, 8) == 50, f"MAC multiply 10*5 mismatch: got {trits_to_int(mac_prod, 8)}"
    dut._log.info("test_02_direct_exec_alu_and_show_reg PASSED")


@cocotb.test()
async def test_03_masking_and_aliasing_hazard(dut):
    """Probe single-use 4t mask auto-reset and 4t/12t swap aliasing + sub_alias_stall resolution."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()
    await drv.send_direct_uop(0x0, 0x7)  # Clear sp0, sp1
    await drv.send_direct_uop(0x0, 0x3)  # Reset masks/aliases

    # Load sp0_trib0 = +7, sp0_trib1 = -13, sp0_trib2 = all -1s (negation mask 0x00)
    for addr4, bits in [(0, int_to_trits(7, 4)), (1, int_to_trits(-13, 4)), (2, NEG1_MASK_4T)]:
        await drv.send_direct_uop(0xB, (bits >> 4) & 0xF, bits & 0xF, (addr4 << 1) | 0)

    # Load single-use 4t mask from sp0_trib2 (0001:0111, p1 = {addr=2, 0=singleuse})
    await drv.send_direct_uop(0x1, 0x7, (2 << 1) | 0)

    # Move sp0_trib0 (+7) -> sp1_trib0 WITH mask (0010:0100, p1={0, mask=1}, p2={3, 0}) -> expects -7
    await drv.send_direct_uop(0x2, 0x4, (0 << 1) | 1, (3 << 1) | 0)
    # Move sp0_trib0 (+7) -> sp1_trib1 WITH mask again -> single-use mask auto-cleared to +1s -> expects +7!
    await drv.send_direct_uop(0x2, 0x4, (0 << 1) | 1, (4 << 1) | 0)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == -7, "Single-use negation mask did not apply"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == 7, "Single-use mask did not auto-reset to 0x55"

    # 4t Swap sp0_trib0 (+7) <-> sp0_trib1 (-13): sets alias_active_4t on sp0_trib1
    await drv.send_direct_uop(0x2, 0x6, (0 << 1) | 0, (1 << 1) | 0)
    # Immediately read 12t sp0 (addr 1) via TX_SHOW_REG -> triggers sub_alias_stall & auto-housekeeping!
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    sp0_after = drv.shown_regs[0]
    assert trits_to_int(sp0_after & 0xFF, 4) == -13, "4t swap trib0 mismatch"
    assert trits_to_int((sp0_after >> 8) & 0xFF, 4) == 7, "4t swap trib1 mismatch (alias commit failed)"

    # 12t Swap sp0 (addr 1) <-> sp1 (addr 2)
    await drv.send_direct_uop(0x2, 0x5, 0x1, 0x2)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    assert drv.shown_regs[0] == sp1, "12t swap sp0 mismatch"
    assert drv.shown_regs[1] == sp0_after, "12t swap sp1 mismatch"
    dut._log.info("test_03_masking_and_aliasing_hazard PASSED")


@cocotb.test()
async def test_04_memory_read_write_and_ext_addr(dut):
    """Probe ext_addr_out (0001:1100), TX_MEM_WRITE (101), TX_MEM_READ (100) turnaround & sanitization."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    ext_val = 0x5A01
    write_payload = int_to_trits(123456, 12)
    ptr_addr = int_to_trits(9876, 12)
    raw_unsanitized = 0xFF5500  # Contains 2'b11 trits that ASIC must sanitize to 2'b10 (0xAA5500)

    await drv.ovl_write_regs({
        6: (ZERO_4T << 16) | ext_val,  # gp_reg_6 holds ext_addr_out in [15:0]
        7: write_payload,              # gp_reg_7 holds data to write
        8: ptr_addr,                   # gp_reg_8 holds pointer address for 0001:1010
    })
    # Absorb TX_SET_PC triggered by writing Reg 5 inside ovl_write_regs
    await drv.run_until_quiet()

    # Set ext_addr_out from gp_reg_6 (0001:1100, p1=6)
    await drv.send_direct_uop(0x1, 0xC, 0x6)

    # Write gp_reg_7 to extended register slot 0x3 (0010:1001, p1=7, p2=3)
    # Address emitted on bus: addr24 = {16'h0000, 4'h3, 4'hA} = 0x00003A, ext16 = ext_val
    await drv.send_direct_uop(0x2, 0x9, 0x7, 0x3)
    assert drv.memory.get((ext_val, 0x00003A)) == write_payload, (
        f"TX_MEM_WRITE failed: memory={drv.memory}"
    )

    # Read back extended register slot 0x3 into sp0 (addr 1) via 0010:1000
    await drv.send_direct_uop(0x2, 0x8, 0x1, 0x3)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    assert drv.shown_regs[0] == write_payload, "TX_MEM_READ (0010:1000) mismatch"

    # Pre-seed memory at (ext_val, ptr_addr) with unsanitized 2'b11 trits and test 0001:1010
    drv.memory[(ext_val, ptr_addr)] = raw_unsanitized
    await drv.send_direct_uop(0x1, 0xA, 0x8)  # Read mem[gp_reg_8] -> overwrites gp_reg_8
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x8)
    assert drv.shown_regs[0] == sanitize_trits_24b(raw_unsanitized), (
        f"TX_MEM_READ sanitization mismatch: got {drv.shown_regs[0]:#08x}"
    )
    dut._log.info("test_04_memory_read_write_and_ext_addr PASSED")


@cocotb.test()
async def test_05_microcode_rom_macro_op_and_pipeline(dut):
    """
    Probe Microcode ROM Mode (3'b101), sector addressing, odd/even caching,
    hold-until-register-change (0001:0100), dual-slot pipelining, cmd buffer dump,
    and PC increment on ret_micro.
    """
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Macro-op 32 (sector 32..111: 4 action bytes = up to 8 micro-ops):
    #   0: Hold until cmd_reg_0 (addr 9) arrives          (0001:0100, p1=9)
    #   1: Clear carry                                     (0000:0010)
    #   2: Extract cmd_reg_0[7:0] -> sp0_trib0 (4t addr 0) (1100: p0=9, p1=0, p2=0)
    #   3: Extract cmd_reg_1[7:0] -> sp0_trib1 (4t addr 1) (1100: p0=10, p1=2, p2=0)
    #   4: Add sp0_trib0 + sp0_trib1 -> sp0_trib2 (addr 2) (0011: p0=0, p1=2, p2=4)
    #   5: Show sp0 (addr 1)                               (0001:1101, p1=1)
    #   6: Return (increments PC, switches to 2nd slot)    (0000:0100)
    mop32_prog = [
        (0x1, 0x4, 0x9, 0x0),
        (0x0, 0x2, 0x0, 0x0),
        (0xC, 0x9, (0 << 1) | 0, 0x0),
        (0xC, 0xA, (1 << 1) | 0, 0x0),
        (0x3, (0 << 1) | 0, (1 << 1) | 0, (2 << 1) | 0),
        (0x1, 0xD, 0x1, 0x0),
        (0x0, 0x4, 0x0, 0x0),
    ]

    # Macro-op 1 (sector 0..31: 2 action bytes = up to 4 micro-ops):
    #   0: Clear scratchpads                               (0000:0111)
    #   1: Load 4t imm (+9) into sp1_trib0 (4t addr 3)     (1011)
    #   2: Show sp1 (addr 2)                               (0001:1101, p1=2)
    #   3: Return                                          (0000:0100)
    imm9 = int_to_trits(9, 4)
    mop1_prog = [
        (0x0, 0x7, 0x0, 0x0),
        (0xB, (imm9 >> 4) & 0xF, imm9 & 0xF, (3 << 1) | 0),
        (0x1, 0xD, 0x2, 0x0),
        (0x0, 0x4, 0x0, 0x0),
    ]

    rom = build_ucode_rom({1: mop1_prog, 32: mop32_prog})
    drv = TripleTDriver(dut, ucode_rom=rom)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    # Initialize PC = 0 via Overlord and absorb initial TX_SET_PC
    await drv.ovl_write_regs({1: ZERO_12T, 2: ZERO_12T, 5: ZERO_12T})
    await drv.run_until_quiet()
    drv.shown_regs.clear()
    drv.pc_updates.clear()

    # Stream Macro-op 32 with live params: param0 = +11, param1 = +8 -> sum = +19
    p0_val = (ZERO_8T << 8) | int_to_trits(11, 4)
    p1_val = (ZERO_8T << 8) | int_to_trits(8, 4)
    await drv.send_macro_op(32, [p0_val, p1_val])

    # Immediately queue 0-param Macro-op 1 into 2nd slot (tests dual-slot pipeline!)
    await drv.send_macro_op(1, [])

    # Let both macro-ops execute to completion, serving all ROM fetches & TX_SHOW_REG
    await drv.run_until_quiet(quiet_cycles=10)

    assert len(drv.shown_regs) == 2, f"Expected 2 TX_SHOW_REG outputs, got {len(drv.shown_regs)}"
    # First output: sp0 = {sum(+19), +8, +11}
    sp0_out = drv.shown_regs[0]
    assert trits_to_int(sp0_out & 0xFF, 4) == 11
    assert trits_to_int((sp0_out >> 8) & 0xFF, 4) == 8
    assert trits_to_int((sp0_out >> 16) & 0xFF, 4) == 19, (
        f"Macro-op 32 sum mismatch: got {trits_to_int((sp0_out >> 16) & 0xFF, 4)}"
    )
    # Second output: sp1 = {0, 0, +9}
    sp1_out = drv.shown_regs[1]
    assert trits_to_int(sp1_out & 0xFF, 4) == 9

    # Verify PC incremented by 2 (one per returned macro-op)
    final_pc = await drv.ovl_read_pc()
    assert trits_to_int(final_pc, 12) == 2, f"Expected PC=+2, got {trits_to_int(final_pc, 12)}"

    # Verify Overlord Command Buffer Dump (0110) while fenced with 2 queued ops
    await drv.ovl_cmd(OVL_FENCE_OR_STEP)  # Engage fence
    await drv.send_macro_op(32, [p0_val, p1_val])
    await drv.send_macro_op(1, [])
    active_op, next_op = await drv.ovl_dump_cmd_buf()
    assert (active_op, next_op) == (32, 1), f"Cmd buf dump mismatch: got {(active_op, next_op)}"
    await drv.ovl_clear_cmd_buf()
    await drv.ovl_cmd(OVL_UNFENCE)
    dut._log.info("test_05_microcode_rom_macro_op_and_pipeline PASSED")


@cocotb.test()
async def test_06_fence_single_step_and_error_autofence(dut):
    """Probe Fence/Single-Step execution and reg_overflow -> Auto-Fence -> Error Telemetry."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()
    await drv.ovl_write_regs({1: ZERO_12T})
    # Absorb TX_SET_PC triggered by writing Reg 5 BEFORE engaging microcode_fence
    await drv.run_until_quiet()

    # 1. Engage microcode fence and load a direct micro-op (4t imm +14 -> sp0_trib0)
    await drv.ovl_cmd(OVL_FENCE_OR_STEP)
    imm14 = int_to_trits(14, 4)
    await drv.send_direct_uop(0xB, (imm14 >> 4) & 0xF, imm14 & 0xF, (0 << 1) | 0, wait_quiet=False)

    # Wait 10 cycles in Thread Mode: while fenced, cur_full (direct_exec_ready) MUST remain 1!
    await drv.enter_thread_idle()
    await ClockCycles(dut.clk, 10)
    await ReadOnly()
    assert ((safe_int(dut.uo_out) >> 3) & 1) == 1, "Direct op executed despite active microcode_fence!"
    await ClockCycles(dut.clk, 1)

    # Single-step 3 times: Step 1 = Stage 0, Step 2 = Stage 1 commit, Step 3 = execute_done clears direct_exec_ready
    for _ in range(3):
        await drv.ovl_cmd(OVL_FENCE_OR_STEP)

    # Verify single-step cleanly consumed direct_exec_ready while still fenced
    await drv.enter_thread_idle()
    await ReadOnly()
    assert ((safe_int(dut.uo_out) >> 3) & 1) == 0, "Single-step failed to clear direct_exec_ready (execute_done wiped while fenced?)"
    await ClockCycles(dut.clk, 1)

    # Verify sp0_trib0 now holds +14
    regs = await drv.ovl_dump_regs()
    assert trits_to_int(regs[1] & 0xFF, 4) == 14, "Single-step did not commit 4t imm load"

    # 2. Unfence and trigger deterministic reg_overflow via Generate Mask (0010:1100) with p1 = 0
    await drv.ovl_cmd(OVL_UNFENCE)
    await drv.send_direct_uop(0x2, 0xC, 0x0, 0x0, wait_quiet=False)
    await ClockCycles(dut.clk, 6)

    # Inspect Overlord status: uo_out[6] = new_error_flag, uo_out[2:0] = offending_error_id (5)
    await drv.ovl_idle()
    await ReadOnly()
    uo = safe_int(dut.uo_out)
    assert ((uo >> 6) & 1) == 1, f"Expected new_error_flag=1, got uo_out={uo:#04x}"
    assert (uo & 0x7) == 5, f"Expected offending_error_id=5 (reg_overflow), got {uo & 0x7}"
    await ClockCycles(dut.clk, 1)

    # Read error counters via showing_errors telemetry (1101 + 1010)
    counts = await drv.ovl_read_error_counters()
    assert counts[5] == 1, f"Expected err_reg_ovf_t == 1, got {counts}"
    assert counts[2] == 0 and counts[3] == 0 and counts[4] == 0, f"Unexpected other errors: {counts}"

    # Clear error counters & alert flag (1110), unfence (1000), and verify recovery!
    await drv.ovl_cmd(OVL_CLR_ALL_ERRS)
    await drv.ovl_cmd(OVL_UNFENCE)

    # Confirm chip executes normally after recovery
    imm_neg5 = int_to_trits(-5, 4)
    await drv.send_direct_uop(0xB, (imm_neg5 >> 4) & 0xF, imm_neg5 & 0xF, (0 << 1) | 0)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == -5, "Chip did not recover after error unfence"
    dut._log.info("test_06_fence_single_step_and_error_autofence PASSED")
