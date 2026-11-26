# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
"""
Exhaustive GLS-safe cocotb probe suite for tt_um_triple_t_devEclipse2.

Test ordering (Logic -> Microcode -> Overlord Injection & Fencing):
  Phase 1 — Logic & Datapath (Direct Execution only, zero Overlord injections):
    test_01_alu_arith_and_shift       : Clear ops, Gen Value, 4t imm, Add/Sub/Add3,
                                        carry, invert-store, Trit-Shift (1/2/3, L/R, local/ovf/mask)
    test_02_mac_ladder_and_tritwise   : MAC seed/mul/read/add/pump, reworked MAC (0100),
                                        4t scratch/MAC clear, Tritwise 1/2/3 (1000..1010)
    test_03_transfers_slices_and_wallow: 12t/4t/1t copy/move/imm, carry imm, all 4 slice modes (1100),
                                        write_allow disable/enable (0000:0101 / 0110)
    test_04_masks_aliases_and_housekeep: 12t/4t/1t single-use & persistent masks, Gen Mask (0010:1100),
                                        12t/4t/1t swaps, sub_alias_stall hazards, housekeep (0000:1010)
    test_05_memory_bus_and_ext_addr   : ext_addr_out, TX_MEM_WRITE, TX_MEM_READ (ext + ptr),
                                        High-Z turnaround & 2'b11 -> 2'b10 sanitization

  Phase 2 — Microcode ROM, Branching, Hops & Pipelining:
    test_06_microcode_sectors_and_flow: All 3 ROM sectors (0..31, 32..111, 112..191),
                                        0xFF 16-cycle timeout, cond branch (0111 eq/neq),
                                        hold_reg (0001:0100), stall-for-next (0000:1001),
                                        next-cmd-space write (0010:1110), ternary hops (0001:1001, 0010:1010..1011)

  Phase 3 — Overlord Register Injections, Fencing & All 4 Error Counters:
    test_07_overlord_injection_and_pc : Placeholder modes, 24-reg write/dump + live ALU check,
                                        PC R/W, 0000:1011 PC inc, direct_instr toggle, cmd buf dump/clr
    test_08_fencing_step_and_errors   : Microcode fence & single-step, plus all 4 error counters
                                        (ID 2 pc_ovf, ID 3 a_ctr_ovf, ID 4 p_ctr_ovf, ID 5 reg_ovf),
                                        auto-fence, alert clear, telemetry, and full recovery
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
# Phase 1: Logic & Datapath Checks (No Overlord Register Injections)
# =============================================================================
@cocotb.test()
async def test_01_alu_arith_and_shift(dut):
    """Exhaustive ALU arithmetic (Add, Sub, Invert, Mask, Add3) & Trit-Shift (1/2/3, L/R, local/ovf)."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    # Initialize datapath purely via direct-exec clear/generate ops (GLS X-safe)
    await drv.send_direct_uop(0x0, 0x7)  # Clear sp0, sp1
    await drv.send_direct_uop(0x0, 0x8)  # Clear gp_reg_6..8
    await drv.send_direct_uop(0x0, 0x1)  # Flush MAC
    await drv.send_direct_uop(0x0, 0x2)  # Clear carry
    await drv.send_direct_uop(0x0, 0x3)  # Reset masks/aliases
    await drv.send_direct_uop(0x2, 0xD, (TRIT_ZRO << 2) | 0b00, 0x3)  # Gen Value 0 -> spf (12t)

    # 1. Verify Generate Value (0010:1101) for 12t (+1s), 4t (-1s), 1t (+1)
    await drv.send_direct_uop(0x2, 0xD, (TRIT_POS << 2) | 0b00, 0x6)       # gp_reg_6 = all +1s (0x555555)
    await drv.send_direct_uop(0x2, 0xD, (TRIT_NEG << 2) | 0b01, (0 << 1))  # sp0_trib0 = all -1s (0x00)
    await drv.send_direct_uop(0x2, 0xD, (TRIT_POS << 2) | 0b10, (0 << 1))  # spf_flag0 = +1 (0b01)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x6)
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    await drv.send_direct_uop(0x1, 0xD, 0x3)
    assert drv.shown_regs[0] == POS1_MASK_12T, "Gen Value 12t mismatch"
    assert (drv.shown_regs[1] & 0xFF) == NEG1_MASK_4T, "Gen Value 4t mismatch"
    assert (drv.shown_regs[2] & 0x03) == TRIT_POS, "Gen Value 1t mismatch"

    # 2. Load sp0_trib0=+30, sp0_trib1=+20, sp0_trib2=-7
    for addr4, val in [(0, 30), (1, 20), (2, -7)]:
        t4 = int_to_trits(val, 4)
        await drv.send_direct_uop(0xB, (t4 >> 4) & 0xF, t4 & 0xF, (addr4 << 1) | 0)

    # Add with overflow: 30 + 20 = +50 -> in 4t balanced ternary: +1*81 + (-31), so sum=-31, carry=+1
    await drv.send_direct_uop(0x0, 0x2)
    await drv.send_direct_uop(0x3, (0 << 1) | 0, (1 << 1) | 0, (3 << 1) | 0)  # sp1_trib0 = -31, carry = +1
    # Next Add uses carry=+1: 30 + (-7) + 1 = +24
    await drv.send_direct_uop(0x3, (0 << 1) | 0, (2 << 1) | 0, (4 << 1) | 0)  # sp1_trib1 = +24, carry = 0
    # Sub with invert-store: -(30 - 20) = -10 (p0[0]=1 sub, p1[0]=1 invert store)
    await drv.send_direct_uop(0x0, 0x2)
    await drv.send_direct_uop(0x3, (0 << 1) | 1, (1 << 1) | 1, (5 << 1) | 0)  # sp1_trib2 = -10

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == -31, "Add overflow sum mismatch"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == 24, "Add carry-in propagation mismatch"
    assert trits_to_int((sp1 >> 16) & 0xFF, 4) == -10, "Sub + invert-store mismatch"

    # 3. Add3 (0101): 30 + 20 + (-7) + carry(+1) = +44 -> 1*81 + (-37), sum=-37, carry=+1
    # First set carry_reg = +1 via 0010:0001 (p1={TRIT_POS, 2'b10})
    await drv.send_direct_uop(0x2, 0x1, (TRIT_POS << 2) | 0b10, 0x0)
    # Packed aaab bbcc cddd: a=0, b=1, c=2, d=3 -> p0=0000, p1=0101, p2=0011
    await drv.send_direct_uop(0x5, 0x0, 0x5, 0x3)
    # Next Add3 with a=2(-7), b=2(-7), c=2(-7) + carry(+1) = -20 -> dest=4
    # a=2(010), b=2(010), c=2(010), d=4(100) -> p0=0100(4), p1=1001(9), p2=0100(4)
    await drv.send_direct_uop(0x5, 0x4, 0x9, 0x4)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == -37, "Add3 overflow sum mismatch"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == -20, "Add3 carry-out propagation mismatch"

    # 4. Trit-Shift (0110): test amt=1 (TRIT_NEG), amt=2 (TRIT_ZRO), amt=3 (TRIT_POS),
    #    both Left (p2[0]=0) & Right (p2[0]=1), local (p1[0]=0) & ovf/udf (p1[0]=1)
    # Load sp0_trib0 = +31 = [1*27 + 0*9 + 1*3 + 1*1] -> trits (t3..t0) = (+1, 0, +1, +1)
    t31 = int_to_trits(31, 4)
    await drv.send_direct_uop(0xB, (t31 >> 4) & 0xF, t31 & 0xF, (0 << 1) | 0)
    # flag0 = -1 (amt=1), flag1 = 0 (amt=2), flag2 = +1 (amt=3)
    await drv.send_direct_uop(0x2, 0x1, (TRIT_NEG << 2) | 0b00, (0 << 1) | 0)
    await drv.send_direct_uop(0x2, 0x1, (TRIT_ZRO << 2) | 0b00, (1 << 1) | 0)
    await drv.send_direct_uop(0x2, 0x1, (TRIT_POS << 2) | 0b00, (2 << 1) | 0)

    # Left shift by 1: local -> (0, +1, +1, 0) = +12; ovf window -> (0, 0, 0, +1) = +1
    await drv.send_direct_uop(0x6, (0 << 1) | 0, (0 << 1) | 0, (3 << 1) | 0)  # sp1_trib0 = +12
    await drv.send_direct_uop(0x6, (0 << 1) | 0, (0 << 1) | 1, (4 << 1) | 0)  # sp1_trib1 = +1
    # Right shift by 2: local -> (0, 0, +1, 0) = +3
    await drv.send_direct_uop(0x6, (0 << 1) | 0, (1 << 1) | 0, (5 << 1) | 1)  # sp1_trib2 = +3

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == 12, "Trit-shift L1 local mismatch"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == 1, "Trit-shift L1 ovf window mismatch"
    assert trits_to_int((sp1 >> 16) & 0xFF, 4) == 3, "Trit-shift R2 local mismatch"

    # Right shift by 3: local -> (0, 0, 0, +1) = +1; udf window -> (0, +1, +1, 0) = +12
    await drv.send_direct_uop(0x6, (0 << 1) | 0, (2 << 1) | 0, (3 << 1) | 1)  # sp1_trib0 = +1
    await drv.send_direct_uop(0x6, (0 << 1) | 0, (2 << 1) | 1, (4 << 1) | 1)  # sp1_trib1 = +12
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == 1, "Trit-shift R3 local mismatch"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == 12, "Trit-shift R3 udf window mismatch"
    dut._log.info("test_01_alu_arith_and_shift PASSED")


@cocotb.test()
async def test_02_mac_ladder_and_tritwise(dut):
    """Exhaustive MAC ladder (seed, mul, add, pump, reworked 0100, clear) & Tritwise 1/2/3 (1000..1010)."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    await drv.send_direct_uop(0x0, 0x7)  # Clear sp0, sp1
    await drv.send_direct_uop(0x0, 0x1)  # Flush MAC
    await drv.send_direct_uop(0x0, 0x3)  # Reset masks/aliases
    await drv.send_direct_uop(0x2, 0xD, (TRIT_ZRO << 2) | 0b00, 0x3)  # Clear spf

    # Load sp0_trib0 = +10, sp0_trib1 = +5, sp0_trib2 = +4
    for addr4, val in [(0, 10), (1, 5), (2, 4)]:
        t4 = int_to_trits(val, 4)
        await drv.send_direct_uop(0xB, (t4 >> 4) & 0xF, t4 & 0xF, (addr4 << 1) | 0)

    # 1. MAC seed (0010:1111, p1[3]=1): 10 * 5 = 50 -> mac_b = +1, mac_a = -31
    await drv.send_direct_uop(0x2, 0xF, (1 << 3) | 0, 1)

    # 2. MAC mul (0010:1111, p1[3]=0): 4 * 5 = 20 -> mac_d = 0, mac_c = +20
    await drv.send_direct_uop(0x2, 0xF, (0 << 3) | 2, 1)

    # 3. MAC add (0000:1101): mac_b <= mac_b(+1) + mac_c(+20) = +21; mac_c <= 0
    await drv.send_direct_uop(0x0, 0xD)
    # Read mac_a(00)->sp1_t0, mac_b(01)->sp1_t1, mac_c(10)->sp1_t2 via 0001:0001
    await drv.send_direct_uop(0x1, 0x1, (0 << 2) | 0b00)
    await drv.send_direct_uop(0x1, 0x1, (1 << 2) | 0b01)
    await drv.send_direct_uop(0x1, 0x1, (2 << 2) | 0b10)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == -31, "MAC seed mac_a mismatch"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == 21, "MAC add mac_b mismatch"
    assert trits_to_int((sp1 >> 16) & 0xFF, 4) == 0, "MAC add did not clear mac_c"

    # 4. Reworked MAC (0100): mul sp0_t2(4) * sp0_t2(4) = 16 (D=0, C=+16),
    #    kicks old mac_a(-31) -> sp1_t0 (addr 3), mac_a <= mac_b(+21)+16 = +37, mac_b <= D(0)
    await drv.send_direct_uop(0x4, (2 << 1) | 0, (2 << 1) | 0, (3 << 1) | 0)
    await drv.send_direct_uop(0x1, 0x1, (1 << 2) | 0b00)  # mac_a -> sp1_t1 (addr 4)
    await drv.send_direct_uop(0x1, 0x1, (2 << 2) | 0b01)  # mac_b -> sp1_t2 (addr 5)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == -31, "Reworked MAC kicked-out mac_a mismatch"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == 37, "Reworked MAC new mac_a mismatch"
    assert trits_to_int((sp1 >> 16) & 0xFF, 4) == 0, "Reworked MAC new mac_b mismatch"

    # 5. MAC pump with dest (0001:0010): kicks mac_a(+37) -> sp1_t0, mac_a<=mac_c(+16), mac_b<=mac_d(0)
    await drv.send_direct_uop(0x1, 0x2, (3 << 1) | 0)
    # MAC pump destinationless (0000:1100): mac_a<=mac_c(+16), mac_b<=mac_d(0)
    await drv.send_direct_uop(0x0, 0xC)
    # Clear 4t MAC slot mac_a (0001:0011, p1={0, 1}) & clear 4t scratch sp1_t1 (0001:0011, p1={4, 0})
    await drv.send_direct_uop(0x1, 0x3, (0 << 1) | 1)
    await drv.send_direct_uop(0x1, 0x3, (4 << 1) | 0)
    await drv.send_direct_uop(0x1, 0x1, (2 << 2) | 0b00)  # cleared mac_a -> sp1_t2
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    sp1 = drv.shown_regs[0]
    assert trits_to_int(sp1 & 0xFF, 4) == 37, "MAC pump dest mismatch"
    assert trits_to_int((sp1 >> 8) & 0xFF, 4) == 0, "Clear 4t scratch mismatch"
    assert trits_to_int((sp1 >> 16) & 0xFF, 4) == 0, "Clear 4t MAC slot mismatch"

    # 6. Tritwise 1, 2, 3 (1000, 1001, 1010): sweep all 8 control sub-ops across (-1, 0, +1)
    #    Keep am_buff_1t = +1 (identity) since p2[0] shares tritwise_command[0] & mask bit
    await drv.send_direct_uop(0x0, 0x3)
    await drv.send_direct_uop(0x2, 0x1, (TRIT_NEG << 2) | 0b00, (0 << 1) | 0)  # flag0 = -1
    await drv.send_direct_uop(0x2, 0x1, (TRIT_ZRO << 2) | 0b00, (1 << 1) | 0)  # flag1 = 0
    await drv.send_direct_uop(0x2, 0x1, (TRIT_POS << 2) | 0b00, (2 << 1) | 0)  # flag2 = +1

    for tw_cmd in (0x8, 0x9, 0xA):
        for subop in range(8):
            b0 = (subop >> 2) & 1
            b1 = (subop >> 1) & 1
            b2 = subop & 1
            # Test (flag0=-1, flag2=+1) -> flag3, and (flag1=0, flag2=+1) -> flag4
            await drv.send_direct_uop(tw_cmd, (0 << 1) | b0, (2 << 1) | b1, (3 << 1) | b2)
            await drv.send_direct_uop(tw_cmd, (1 << 1) | b0, (2 << 1) | b1, (4 << 1) | b2)
        drv.shown_regs.clear()
        await drv.send_direct_uop(0x1, 0xD, 0x3)  # Show spf (checks no X/Z on flag3/flag4!)
        spf_val = drv.shown_regs[0]
        f3 = (spf_val >> 6) & 0x3
        f4 = (spf_val >> 8) & 0x3
        assert f3 in (TRIT_NEG, TRIT_POS, TRIT_ZRO), f"Tritwise {tw_cmd:#x} invalid trit f3={f3:#04b}"
        assert f4 in (TRIT_NEG, TRIT_POS, TRIT_ZRO), f"Tritwise {tw_cmd:#x} invalid trit f4={f4:#04b}"

    dut._log.info("test_02_mac_ladder_and_tritwise PASSED")


@cocotb.test()
async def test_03_transfers_slices_and_write_allow(dut):
    """Exhaustive 12t/4t/1t copy/move, all 4 slice modes (1100), and write_allow disable/enable."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    await drv.send_direct_uop(0x0, 0x7)
    await drv.send_direct_uop(0x0, 0x8)
    await drv.send_direct_uop(0x0, 0x3)
    await drv.send_direct_uop(0x2, 0xD, (TRIT_ZRO << 2) | 0b00, 0x3)

    # Seed sp0 = {+19, -8, +5}
    for addr4, val in [(0, 5), (1, -8), (2, 19)]:
        t4 = int_to_trits(val, 4)
        await drv.send_direct_uop(0xB, (t4 >> 4) & 0xF, t4 & 0xF, (addr4 << 1) | 0)

    # 1. 12t Copy unmasked (0010:0000): sp0(1) -> gp_reg_6(6)
    await drv.send_direct_uop(0x2, 0x0, 0x1, 0x6)
    # 2. Slice 12t -> 4t extract (1100, p1[0]=0, p2[0]=0): extract trib2 (+19) of gp_reg_6 -> sp1_t0(3)
    await drv.send_direct_uop(0xC, 0x6, (3 << 1) | 0, (2 << 2) | 0b00)
    # 3. Slice 4t -> 12t insert (1100, p1[0]=0, p2[0]=1): insert sp0_t1 (-8) into trib0 of gp_reg_6
    await drv.send_direct_uop(0xC, 0x6, (1 << 1) | 0, (0 << 2) | 0b01)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    await drv.send_direct_uop(0x1, 0xD, 0x6)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == 19, "12t->4t slice extract mismatch"
    assert trits_to_int(drv.shown_regs[1] & 0xFF, 4) == -8, "4t->12t slice insert mismatch"

    # 4. Slice 4t -> 1t extract & 1t -> 4t insert (1100, p1[0]=1)
    # sp0_t0 = +5 = (0, +1, -1, -1) -> trit0 = -1 (00), trit1 = -1 (00), trit2 = +1 (01), trit3 = 0 (10)
    await drv.send_direct_uop(0xC, (0 << 1) | 0, (0 << 1) | 1, (2 << 2) | 0b00)  # extract trit2 (+1) -> flag0
    # 1t Move (0010:0011): flag0 -> flag1
    await drv.send_direct_uop(0x2, 0x3, (0 << 1) | 0, (1 << 1) | 0)
    # Insert flag1 (+1) into trit3 of sp1_t1 (currently 0 -> becomes +27)
    await drv.send_direct_uop(0xC, (4 << 1) | 0, (1 << 1) | 1, (3 << 2) | 0b01)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    assert trits_to_int((drv.shown_regs[0] >> 8) & 0xFF, 4) == 27, "4t<->1t slice transfer mismatch"

    # 5. Disable write (0000:0101): attempt to overwrite sp0_t0 and gp_reg_6 -> MUST NOT CHANGE
    await drv.send_direct_uop(0x0, 0x5)
    t_bad = int_to_trits(-40, 4)
    await drv.send_direct_uop(0xB, (t_bad >> 4) & 0xF, t_bad & 0xF, (0 << 1) | 0)
    await drv.send_direct_uop(0x2, 0x0, 0x2, 0x6)
    await drv.send_direct_uop(0x0, 0x7)  # Clear scratchpads while write_allow=0 -> ignored

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == 5, "write_allow=0 failed to block writes!"

    # 6. Re-enable write (0000:0110) and verify writes resume
    await drv.send_direct_uop(0x0, 0x6)
    t_ok = int_to_trits(17, 4)
    await drv.send_direct_uop(0xB, (t_ok >> 4) & 0xF, t_ok & 0xF, (0 << 1) | 0)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == 17, "write_allow=1 failed to re-enable writes!"
    dut._log.info("test_03_transfers_slices_and_write_allow PASSED")


@cocotb.test()
async def test_04_masks_aliases_and_housekeep(dut):
    """Exhaustive 12t/4t/1t masks (single-use & persistent), Gen Mask (0010:1100), swaps & sub_alias_stall."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()
    await drv.send_direct_uop(0x0, 0x7)
    await drv.send_direct_uop(0x0, 0x8)
    await drv.send_direct_uop(0x0, 0x3)
    await drv.send_direct_uop(0x2, 0xD, (TRIT_ZRO << 2) | 0b00, 0x3)

    # 1. Generate 12t Mask (0010:1100): trib 0 = all -1s (negate), single-use (p2[1]=0)
    #    sp0 = {+6, +6, +6}
    for addr4 in (0, 1, 2):
        t6 = int_to_trits(6, 4)
        await drv.send_direct_uop(0xB, (t6 >> 4) & 0xF, t6 & 0xF, (addr4 << 1) | 0)

    await drv.send_direct_uop(0x2, 0xC, 0x1, (TRIT_NEG << 2) | 0b00)  # trib 0 negate, single-use
    # Masked 12t Copy (0010:0010): sp0(1) -> gp_reg_6(6) -> expects {+6, +6, -6}
    await drv.send_direct_uop(0x2, 0x2, 0x1, 0x6)
    # Second masked 12t Copy -> single-use reset to +1s -> expects {+6, +6, +6}
    await drv.send_direct_uop(0x2, 0x2, 0x1, 0x7)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x6)
    await drv.send_direct_uop(0x1, 0xD, 0x7)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == -6, "Gen 12t mask trib0 negate failed"
    assert trits_to_int((drv.shown_regs[0] >> 8) & 0xFF, 4) == 6, "Gen 12t mask altered trib1"
    assert trits_to_int(drv.shown_regs[1] & 0xFF, 4) == 6, "Gen 12t single-use mask did not reset"

    # 2. Persistent 12t mask via 0001:0110 (copy gp_reg_8 = all -1s into am_buff_12t)
    await drv.send_direct_uop(0x2, 0xD, (TRIT_NEG << 2) | 0b00, 0x8)
    await drv.send_direct_uop(0x1, 0x6, 0x8)  # Persistent 12t mask
    await drv.send_direct_uop(0x2, 0x2, 0x1, 0x6)
    await drv.send_direct_uop(0x2, 0x2, 0x1, 0x7)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x6)
    await drv.send_direct_uop(0x1, 0xD, 0x7)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == -6
    assert trits_to_int(drv.shown_regs[1] & 0xFF, 4) == -6, "Persistent 12t mask did not persist!"

    # 3. 1t Swap (0010:0111) + 1t persistent mask (0001:1000) + sub_alias_stall (12t vs 1t & 4t vs 1t)
    await drv.send_direct_uop(0x0, 0x3)  # Reset masks
    await drv.send_direct_uop(0x2, 0x1, (TRIT_POS << 2) | 0b00, (0 << 1) | 0)  # flag0 = +1
    await drv.send_direct_uop(0x2, 0x1, (TRIT_NEG << 2) | 0b00, (1 << 1) | 0)  # flag1 = -1
    # Swap flag0 <-> flag1 -> sets alias_active_1t on flag1
    await drv.send_direct_uop(0x2, 0x7, (0 << 1) | 0, (1 << 1) | 0)
    # Immediately load 1t mask from flag0 (forces Stage 1 housekeeping if aliased!)
    await drv.send_direct_uop(0x1, 0x8, (0 << 1) | 1)  # persistent 1t mask = flag0 (now -1)
    # Move flag1 (+1) -> flag2 WITH mask (-1) -> expects -1
    await drv.send_direct_uop(0x2, 0x3, (1 << 1) | 1, (2 << 1) | 0)
    # Explicit housekeep (0000:1010)
    await drv.send_direct_uop(0x0, 0xA)

    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x3)
    spf = drv.shown_regs[0]
    assert (spf & 0x3) == TRIT_NEG, "1t swap flag0 mismatch"
    assert ((spf >> 2) & 0x3) == TRIT_POS, "1t swap flag1 mismatch"
    assert ((spf >> 4) & 0x3) == TRIT_NEG, "1t masked move mismatch"

    # 4. 4t Swap (0010:0110) + 12t Swap (0010:0101)
    t_a, t_b = int_to_trits(11, 4), int_to_trits(-9, 4)
    await drv.send_direct_uop(0xB, (t_a >> 4) & 0xF, t_a & 0xF, (0 << 1) | 0)
    await drv.send_direct_uop(0xB, (t_b >> 4) & 0xF, t_b & 0xF, (1 << 1) | 0)
    await drv.send_direct_uop(0x2, 0x6, (0 << 1) | 0, (1 << 1) | 0)  # 4t swap sp0_t0 <-> sp0_t1
    await drv.send_direct_uop(0x2, 0x5, 0x1, 0x2)                    # 12t swap sp0 <-> sp1 (triggers sub_alias_stall!)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x2)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == -9
    assert trits_to_int((drv.shown_regs[0] >> 8) & 0xFF, 4) == 11
    dut._log.info("test_04_masks_aliases_and_housekeep PASSED")


@cocotb.test()
async def test_05_memory_bus_and_ext_addr(dut):
    """Memory R/W (0001:1100, 0010:1001, 0010:1000, 0001:1010) initialized purely via direct exec."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    await drv.send_direct_uop(0x0, 0x7)
    await drv.send_direct_uop(0x0, 0x8)

    # Build ext_val = 0x5A01 in sp0_t0/t1 -> copy to gp_reg_6 -> ext_addr_out
    await drv.send_direct_uop(0xB, 0x0, 0x1, (0 << 1) | 0)  # sp0_t0 = 0x01
    await drv.send_direct_uop(0xB, 0x5, 0xA, (1 << 1) | 0)  # sp0_t1 = 0x5A
    await drv.send_direct_uop(0x1, 0xC, 0x1)                # ext_addr_out <= sp0[15:0] = 0x5A01

    # Build write_payload = int_to_trits(123456, 12) in sp1
    write_payload = int_to_trits(123456, 12)
    for idx in range(3):
        b = (write_payload >> (8 * idx)) & 0xFF
        await drv.send_direct_uop(0xB, (b >> 4) & 0xF, b & 0xF, ((3 + idx) << 1) | 0)

    # TX_MEM_WRITE (0010:1001): write sp1(2) to ext reg slot 0x3 (addr 0x00003A, ext 0x5A01)
    await drv.send_direct_uop(0x2, 0x9, 0x2, 0x3)
    assert drv.memory.get((0x5A01, 0x00003A)) == write_payload, "TX_MEM_WRITE mismatch"

    # TX_MEM_READ (0010:1000): read back slot 0x3 -> gp_reg_7(7)
    await drv.send_direct_uop(0x2, 0x8, 0x7, 0x3)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x7)
    assert drv.shown_regs[0] == write_payload, "TX_MEM_READ (0010:1000) mismatch"

    # Pointer-based TX_MEM_READ (0001:1010) with unsanitized 2'b11 trits -> must sanitize to 2'b10
    raw_unsanitized = 0xFF5500
    drv.memory[(0x5A01, write_payload)] = raw_unsanitized
    await drv.send_direct_uop(0x1, 0xA, 0x7)  # Read mem[gp_reg_7] -> overwrites gp_reg_7
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x7)
    assert drv.shown_regs[0] == sanitize_trits_24b(raw_unsanitized), "TX_MEM_READ sanitization failed"
    dut._log.info("test_05_memory_bus_and_ext_addr PASSED")


# =============================================================================
# Phase 2: Microcode ROM, All 3 Sectors, Branching, Hops & Pipelining
# =============================================================================
@cocotb.test()
async def test_06_microcode_sectors_and_flow(dut):
    """
    Tests ROM sectors 0..31 (mop 1), 32..111 (mop 32), 112..191 (mop 112),
    0xFF 16-cycle timeout, cond branch (0111 eq/neq), hold_reg (0001:0100),
    wait-for-next-cmd (0000:1001), next-cmd-space write (0010:1110), and ternary hops.
    """
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Macro-op 112 (sector 112..191): exercises cond branch 0111 (taken + not-taken) & 0xFF timeout
    #   0: 1t imm +1 -> flag0                    (0010:0001, p1=0100, p2=0000) [3 nibs]
    #   1: if flag0 == +1 jump act=4, prm=5      (0111, p0=0001, p1=1001, p2=0101) [3 nibs] -> prm 0..2
    #   2: (SKIPPED) 4t imm -1 -> sp0_t0         (1011, p0=A, p1=8, p2=0) [3 nibs]
    #   3: (SKIPPED) Return                      (0000:0100) [1 nib] -> prm 3..4
    #   4: (TARGET) 4t imm 0xFF -> sp0_t0 (tests 0xFF ROM timeout!) (1011, p0=F, p1=F, p2=0) [3 nibs]
    #   5: if flag0 != +1 jump act=0, prm=0 (NOT TAKEN!) (0111, p0=0000, p1=0000, p2=0001) [3 nibs] -> prm 5..7
    #   6: Show sp0 (addr 1)                     (0001:1101, p1=1) [2 nibs]
    #   7: Return                                (0000:0100) [1 nib] -> prm 8..9
    mop112_prog = [
        (0x2, 0x1, (TRIT_POS << 2) | 0b00, 0x0),
        (0x7, (0 << 1) | 1, (4 << 1) | 1, (0b01 << 2) | TRIT_POS),
        (0xB, 0xA, 0x8, 0x0),
        (0x0, 0x4, 0x0, 0x0),
        (0xB, 0xF, 0xF, 0x0),
        (0x7, (0 << 1) | 0, 0x0, TRIT_POS),
        (0x1, 0xD, 0x1, 0x0),
        (0x0, 0x4, 0x0, 0x0),
    ]

    # Macro-op 32 (sector 32..111): hold_reg + stall-for-next (0000:1001) + write-next-cmd-space (0010:1110)
    #   0: Hold until cmd_reg_0 (addr 9) arrives          (0001:0100, p1=9)
    #   1: Wait until 2nd macro-op is queued              (0000:1001)
    #   2: Copy cmd_reg_0 (9) -> NEXT cmd_reg_0 (9)       (0010:1110, p1=9, p2=9)
    #   3: Return (hands off to 2nd macro-op!)            (0000:0100)
    mop32_prog = [
        (0x1, 0x4, 0x9, 0x0),
        (0x0, 0x9, 0x0, 0x0),
        (0x2, 0xE, 0x9, 0x9),
        (0x0, 0x4, 0x0, 0x0),
    ]

    # Macro-op 1 (sector 0..31): receives carried-over cmd_reg_0 from mop32 and shows it!
    #   0: Show cmd_reg_0 (addr 9)                        (0001:1101, p1=9)
    #   1: Return                                         (0000:0100)
    mop1_prog = [
        (0x1, 0xD, 0x9, 0x0),
        (0x0, 0x4, 0x0, 0x0),
    ]

    rom = build_ucode_rom({1: mop1_prog, 32: mop32_prog, 112: mop112_prog})
    drv = TripleTDriver(dut, ucode_rom=rom)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()

    await drv.send_direct_uop(0x0, 0x7)
    await drv.send_direct_uop(0x2, 0xD, (TRIT_ZRO << 2) | 0b00, 0x3)

    # 1. Run Macro-op 112 (sector 112..191 + cond branch + 0xFF timeout)
    drv.shown_regs.clear()
    await drv.send_macro_op(112, [])
    await drv.run_until_quiet(quiet_cycles=10)
    assert len(drv.shown_regs) == 1, "Expected 1 TX_SHOW_REG from mop112"
    assert (drv.shown_regs[0] & 0xFF) == 0xFF, (
        f"mop112 branch/0xFF timeout failed: sp0_t0={drv.shown_regs[0] & 0xFF:#04x}"
    )

    # 2. Run Macro-op 32 (stalls on 0000:1001 until mop 1 arrives, then passes param via 0010:1110!)
    drv.shown_regs.clear()
    carry_val = int_to_trits(54321, 12)
    await drv.send_macro_op(32, [carry_val])
    await drv.send_macro_op(1, [])
    await drv.run_until_quiet(quiet_cycles=10)
    assert len(drv.shown_regs) == 1, "Expected 1 TX_SHOW_REG from mop1"
    assert drv.shown_regs[0] == carry_val, (
        f"0010:1110 next-cmd-space handoff mismatch: got {drv.shown_regs[0]:#08x}, exp {carry_val:#08x}"
    )

    # 3. Probe ternary counter hops (0010:1011 act imm, 0010:1010 prm imm, 0001:1001 reg hop)
    #    Verify each updates action_ctr/param_ctr cleanly into [0..80] with no X/deadlock
    await drv.send_direct_uop(0x2, 0xB, 0xA, 0x5)  # imm hop action_ctr
    await drv.send_direct_uop(0x2, 0xA, 0xA, 0x5)  # imm hop param_ctr
    await drv.send_direct_uop(0xB, 0xA, 0x9, (0 << 1) | 0)  # sp0_t0 = 0xA9
    await drv.send_direct_uop(0x1, 0x9, (0 << 1) | 0)       # reg hop action_ctr
    await drv.send_direct_uop(0x1, 0x9, (0 << 1) | 1)       # reg hop param_ctr
    regs = await drv.ovl_dump_regs()
    act_c = (regs[23] >> 8) & 0x7F
    prm_c = regs[23] & 0x7F
    assert 0 <= act_c <= 80 and 0 <= prm_c <= 80, f"Hop counter out of range: act={act_c}, prm={prm_c}"
    await drv.ovl_clear_cmd_buf()
    dut._log.info("test_06_microcode_sectors_and_flow PASSED")


# =============================================================================
# Phase 3: Overlord Register Injections, Fencing, Single-Step & All 4 Errors
# =============================================================================
@cocotb.test()
async def test_07_overlord_injection_and_pc(dut):
    """Overlord 24-reg injection -> live ALU execution, PC R/W, 0000:1011 PC inc, direct toggle & cmd dump."""
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

    await drv.ovl_clear_cmd_buf()

    # 1. Inject all 24 registers via Overlord (1011) and verify readback (0111)
    expected = {
        1: (int_to_trits(0, 4) << 16) | (int_to_trits(13, 4) << 8) | int_to_trits(7, 4),  # sp0 = {0, 13, 7}
        2: int_to_trits(-5678, 12),
        3: 0xAA5500,
        4: 0x55AA00,
        5: int_to_trits(42, 12),
        6: int_to_trits(100, 12),
        7: int_to_trits(-200, 12),
        8: int_to_trits(300, 12),
        9: int_to_trits(11, 12),
        10: int_to_trits(22, 12),
        11: int_to_trits(33, 12),
        12: int_to_trits(44, 12),
        13: int_to_trits(55, 12),
        14: int_to_trits(66, 12),
        15: int_to_trits(-11, 12),
        16: int_to_trits(-22, 12),
        17: int_to_trits(-33, 12),
        18: int_to_trits(-44, 12),
        19: int_to_trits(-55, 12),
        20: int_to_trits(-66, 12),
        21: POS1_MASK_12T,
        22: (0 << 16) | (POS1_MASK_4T << 8) | (TRIT_POS << 4),
        23: (ZERO_1T << 20),
        24: 0x000000,
    }
    await drv.ovl_write_regs(expected)
    readback = await drv.ovl_dump_regs()
    for addr, exp_val in expected.items():
        assert readback[addr] == exp_val, (
            f"Overlord Reg {addr} mismatch: got {readback[addr]:#08x}, exp {exp_val:#08x}"
        )

    # Drain TX_SET_PC from writing Reg 5, then verify injected sp0 (7 + 13 = 20) drives live ALU!
    await drv.run_until_quiet()
    await drv.send_direct_uop(0x3, (0 << 1) | 0, (1 << 1) | 0, (2 << 1) | 0)  # sp0_t2 = 7 + 13 = 20
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    assert trits_to_int((drv.shown_regs[0] >> 16) & 0xFF, 4) == 20, "Injected sp0 failed in live ALU"

    # 2. PC Write (0101) -> PC Read (0010) -> Unfence -> TX_SET_PC -> 0000:1011 PC Inc -> TX_SET_PC(+1)
    new_pc = int_to_trits(777, 12)
    await drv.ovl_write_pc(new_pc)
    assert await drv.ovl_read_pc() == new_pc, "OVL_READ_PC mismatch"
    drv.pc_updates.clear()
    await drv.ovl_cmd(OVL_UNFENCE)
    await drv.run_until_quiet()
    assert new_pc in drv.pc_updates, "Missing TX_SET_PC after OVL_WRITE_PC"

    drv.pc_updates.clear()
    await drv.send_direct_uop(0x0, 0xB)  # 0000:1011 PC increment -> 778
    assert await drv.ovl_read_pc() == int_to_trits(778, 12), "0000:1011 PC increment mismatch"
    assert int_to_trits(778, 12) in drv.pc_updates, "Missing TX_SET_PC after 0000:1011"

    # 3. Direct instruction toggle (0001) + Command Buffer Dump (0110) while fenced
    await drv.ovl_cmd(OVL_TOGGLE_DIRECT)
    await drv.ovl_cmd(OVL_TOGGLE_DIRECT)  # Toggle back OFF
    await drv.ovl_cmd(OVL_FENCE_OR_STEP)  # Engage fence
    await drv.send_macro_op(10, [])
    await drv.send_macro_op(20, [])
    active_op, next_op = await drv.ovl_dump_cmd_buf()
    assert (active_op, next_op) == (10, 20), f"Cmd buf dump mismatch: {(active_op, next_op)}"
    await drv.ovl_clear_cmd_buf()
    await drv.ovl_cmd(OVL_UNFENCE)
    dut._log.info("test_07_overlord_injection_and_pc PASSED")


@cocotb.test()
async def test_08_fencing_step_and_all_errors(dut):
    """Fencing, single-stepping, and all 4 error IDs (2:PC, 3:Act, 4:Prm, 5:Reg) + auto-fence + recovery."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    drv = TripleTDriver(dut)
    await drv.reset()
    await drv.ovl_clear_cmd_buf()
    await drv.ovl_write_regs({1: ZERO_12T, 5: ZERO_12T})
    await drv.run_until_quiet()

    # 1. Fence + Single-Step execution
    await drv.ovl_cmd(OVL_FENCE_OR_STEP)
    imm14 = int_to_trits(14, 4)
    await drv.send_direct_uop(0xB, (imm14 >> 4) & 0xF, imm14 & 0xF, (0 << 1) | 0, wait_quiet=False)

    await drv.enter_thread_idle()
    await ClockCycles(dut.clk, 8)
    await ReadOnly()
    assert ((safe_int(dut.uo_out) >> 3) & 1) == 1, "Op ran despite microcode_fence!"
    await ClockCycles(dut.clk, 1)

    for _ in range(3):
        await drv.ovl_cmd(OVL_FENCE_OR_STEP)

    await drv.enter_thread_idle()
    await ReadOnly()
    assert ((safe_int(dut.uo_out) >> 3) & 1) == 0, "Single-step failed to clear direct_exec_ready"
    await ClockCycles(dut.clk, 1)

    regs = await drv.ovl_dump_regs()
    assert trits_to_int(regs[1] & 0xFF, 4) == 14, "Single-step did not commit sp0_t0"
    await drv.ovl_cmd(OVL_UNFENCE)

    # 2. Error ID 5: reg_overflow via 0010:1100 with p1=0
    await drv.send_direct_uop(0x2, 0xC, 0x0, 0x0, wait_quiet=False)
    await ClockCycles(dut.clk, 6)
    await drv.ovl_idle()
    await ReadOnly()
    uo = safe_int(dut.uo_out)
    assert ((uo >> 6) & 1) == 1 and (uo & 0x7) == 5, f"Expected ID 5 reg_ovf, got uo={uo:#04x}"
    await ClockCycles(dut.clk, 1)
    # Test OVL_CLR_ERR_FLAG (1100) clears alert bit without losing counter
    await drv.ovl_cmd(OVL_CLR_ERR_FLAG)
    await drv.ovl_idle()
    await ReadOnly()
    assert ((safe_int(dut.uo_out) >> 6) & 1) == 0, "OVL_CLR_ERR_FLAG did not clear uo_out[6]"
    await ClockCycles(dut.clk, 1)

    # 3. Error ID 3: a_ctr_overflow via Overlord Reg 23 injection (action_ctr = 81)
    await drv.ovl_write_regs({23: (ZERO_1T << 20) | (81 << 8) | 0})
    await ClockCycles(dut.clk, 2)
    await drv.ovl_idle()
    await ReadOnly()
    uo = safe_int(dut.uo_out)
    assert ((uo >> 6) & 1) == 1 and (uo & 0x7) == 3, f"Expected ID 3 a_ctr_ovf, got uo={uo:#04x}"
    await ClockCycles(dut.clk, 1)

    # 4. Error ID 4: p_ctr_overflow via Overlord Reg 23 injection (action_ctr = 0, param_ctr = 81)
    await drv.ovl_write_regs({23: (ZERO_1T << 20) | (0 << 8) | 81})
    await ClockCycles(dut.clk, 2)
    await drv.ovl_idle()
    await ReadOnly()
    uo = safe_int(dut.uo_out)
    assert ((uo >> 6) & 1) == 1 and (uo & 0x7) == 4, f"Expected ID 4 p_ctr_ovf, got uo={uo:#04x}"
    await ClockCycles(dut.clk, 1)

    # Clear counters back to 0 so a_ctr/p_ctr_overflow drop low
    await drv.ovl_clear_cmd_buf()

    # 5. Error ID 2: pc_overflow by setting PC = all +1s (0x555555) and running 0000:1011
    await drv.ovl_write_pc(POS1_MASK_12T)
    await drv.ovl_cmd(OVL_UNFENCE)
    await drv.run_until_quiet()
    drv.pc_updates.clear()
    await drv.send_direct_uop(0x0, 0xB, wait_quiet=False)  # Wraps 0x555555 -> 0x000000 & auto-fences!
    await ClockCycles(dut.clk, 6)
    await drv.ovl_idle()
    await ReadOnly()
    uo = safe_int(dut.uo_out)
    assert ((uo >> 6) & 1) == 1 and (uo & 0x7) == 2, f"Expected ID 2 pc_ovf, got uo={uo:#04x}"
    await ClockCycles(dut.clk, 1)

    # Verify all 4 error counters == 1 via telemetry (1101 + 1010)
    counts = await drv.ovl_read_error_counters()
    assert counts == {2: 1, 3: 1, 4: 1, 5: 1}, f"Expected all 4 error counters == 1, got {counts}"

    # Clear all errors (1110), unfence (1000), drain wrapped TX_SET_PC (0x000000), and verify recovery
    await drv.ovl_cmd(OVL_CLR_ALL_ERRS)
    await drv.ovl_cmd(OVL_UNFENCE)
    await drv.run_until_quiet()
    assert NEG1_MASK_12T in drv.pc_updates, f"Expected wrapped PC 0x000000 in {drv.pc_updates}"

    counts_after = await drv.ovl_read_error_counters()
    assert counts_after == {2: 0, 3: 0, 4: 0, 5: 0}, f"OVL_CLR_ALL_ERRS failed: {counts_after}"

    imm_ok = int_to_trits(-5, 4)
    await drv.send_direct_uop(0xB, (imm_ok >> 4) & 0xF, imm_ok & 0xF, (0 << 1) | 0)
    drv.shown_regs.clear()
    await drv.send_direct_uop(0x1, 0xD, 0x1)
    assert trits_to_int(drv.shown_regs[0] & 0xFF, 4) == -5, "Chip failed to recover after all-error test"
    dut._log.info("test_08_fencing_step_and_all_errors PASSED")
