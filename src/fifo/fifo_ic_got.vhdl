-- EMACS settings: -*-  tab-width: 2; indent-tabs-mode: t -*-
-- vim: tabstop=2:shiftwidth=2:noexpandtab
-- kate: tab-width 2; replace-tabs off; indent-width 2;
-- =============================================================================
-- Authors:					Thomas B. Preusser
--									Steffen Koehler
--									Martin Zabel
--
-- Entity:					FIFO, independent clocks (ic), first-word-fall-through mode
--
-- Description:
-- -------------------------------------
-- Independent clocks meens that read and write clock are unrelated.
--
-- This implementation uses dedicated block RAM for storing data.
--
-- First-word-fall-through (FWFT) mode is implemented, so data can be read out
-- as soon as ``valid`` goes high. After the data has been captured, then the
-- signal ``got`` must be asserted.
--
-- Synchronous reset is used. Both resets may overlap.
--
-- ``DATA_REG`` (=true) is a hint, that distributed memory or registers should be
-- used as data storage. The actual memory type depends on the device
-- architecture. See implementation for details.
--
-- ``*STATE_*_BITS`` defines the granularity of the fill state indicator
-- ``*state_*``. ``fstate_rd`` is associated with the read clock domain and outputs
-- the guaranteed number of words available in the FIFO. ``estate_wr`` is
-- associated with the write clock domain and outputs the number of words that
-- is guaranteed to be accepted by the FIFO without a capacity overflow. Note
-- that both these indicators cannot replace the ``full`` or ``valid`` outputs as
-- they may be implemented as giving pessimistic bounds that are minimally off
-- the true fill state.
--
-- If a fill state is not of interest, set *STATE_*_BITS = 0.
--
-- ``fstate_rd`` and ``estate_wr`` are combinatorial outputs and include an address
-- comparator (subtractor) in their path.
--
-- Examples:
-- - FSTATE_RD_BITS = 1: fstate_rd == 0 => 0/2 full
--                       fstate_rd == 1 => 1/2 full (half full)
--
-- - FSTATE_RD_BITS = 2: fstate_rd == 0 => 0/4 full
--                       fstate_rd == 1 => 1/4 full
--                       fstate_rd == 2 => 2/4 full
--                       fstate_rd == 3 => 3/4 full
--
-- License:
-- =============================================================================
-- Copyright 2007-2014 Technische Universitaet Dresden - Germany
--                     Chair of VLSI-Design, Diagnostics and Architecture
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--		http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
-- =============================================================================

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library poc;
use PoC.utils.all;
use poc.ocram.all;                      -- "all" required by Quartus RTL simulation

entity fifo_ic_got is
    generic(
        D_BITS         : positive;      -- Data Width
        MIN_DEPTH      : positive;      -- Minimum FIFO Depth
        DATA_REG       : boolean := false; -- Store Data Content in Registers
        OUTPUT_REG     : boolean := false; -- Registered FIFO Output
        ESTATE_WR_BITS : natural := 0;  -- Empty State Bits
        FSTATE_RD_BITS : natural := 0   -- Full State Bits
    );
    port(
        -- Write Interface
        writeClkIn    : in  std_logic;
        writeRstIn    : in  std_logic;
        putIn         : in  boolean;
        dataIn        : in  std_logic_vector(D_BITS - 1 downto 0);
        fullOut       : out boolean;
        writeStateOut : out std_logic_vector(imax(ESTATE_WR_BITS - 1, 0) downto 0);
        -- Read Interface
        readClkIn     : in  std_logic;
        readRstIn     : in  std_logic;
        gotIn         : in  boolean;
        valid         : out boolean;
        dataOut       : out std_logic_vector(D_BITS - 1 downto 0);
        readStateOut  : out std_logic_vector(imax(FSTATE_RD_BITS - 1, 0) downto 0)
    );
end entity fifo_ic_got;

architecture rtl of fifo_ic_got is

    -- Constants
    constant A_BITS : positive := log2ceilnz(MIN_DEPTH);
    constant AN     : positive := A_BITS + 1; -- to prevent overflow?

    -- Types
    subtype Data_T is std_logic_vector(D_BITS - 1 downto 0);
    subtype Addr_T is unsigned(A_BITS - 1 downto 0);
    subtype Addr_Ptr_T is std_logic_vector(AN - 1 downto 0);
    subtype Valid_Addr_Range is natural range A_BITS - 1 downto 0;

    -- Registers, writeClkIn domain
    signal writePtrNext : Addr_Ptr_T;   -- IP + 1
    signal writePtr     : Addr_Ptr_T := (others => '0'); -- Write Pointer IP
    signal writePtrPrev : Addr_Ptr_T := (others => '0'); -- IP delayed by one clock
    signal readPtrSync  : Addr_Ptr_T := (others => '0'); -- Sync stage: OP0 -> OPc
    signal readPtrCopy  : Addr_Ptr_T := (others => '0'); -- Copy of OP
    signal ramFull      : boolean    := false; -- RAM full

    -- Registers, readClkIn domain
    signal readPtrNext      : Addr_Ptr_T; -- OP + 1
    signal readPtr          : Addr_Ptr_T := (others => '0'); -- Read Pointer OP
    signal writePtrSync     : Addr_Ptr_T := (others => '0'); -- Sync stage: IPz -> IPc
    signal writePtrCopy     : Addr_Ptr_T := (others => '0'); -- Copy of IP
    signal memDataAvailable : boolean    := false; -- RAM Data available
    signal outputValid      : boolean    := false; -- Output Valid

    -- Memory Connectivity
    signal memWriteAddr : Addr_T;
    signal memDataIn    : Data_T;
    signal putNewData   : boolean;

    signal memReadAddr : Addr_T;
    signal memDataOut  : Data_T;
    signal getNewData  : boolean;

    signal gotData : boolean;           -- Internal Read ACK

begin

    -----------------------------------------------------------------------------
    -- Write clock domain
    -----------------------------------------------------------------------------

    -- compute next pointer value
    blkIP : block
        signal currCount : unsigned(AN - 1 downto 0) := to_unsigned(1, AN);
    begin
        process(writeClkIn)
        begin
            if rising_edge(writeClkIn) then
                if writeRstIn = '1' then
                    currCount <= to_unsigned(1, AN);
                elsif putNewData then
                    currCount <= currCount + 1;
                end if;
            end if;
        end process;
        -- compute next pointer value (circular fifo?)
        writePtrNext <= std_logic_vector(currCount(A_BITS) & (currCount(A_BITS - 1 downto 0) xor ('0' & currCount(A_BITS - 1 downto 1))));
    end block blkIP;

    -- Update write Pointer upon putNewData
    process(writeClkIn)
    begin
        if rising_edge(writeClkIn) then
            if writeRstIn = '1' then
                writePtr     <= (others => '0');
                writePtrPrev <= (others => '0');
                readPtrSync  <= (others => '0');
                readPtrCopy  <= (others => '0');
                ramFull      <= false;
            else
                writePtrPrev <= writePtr;
                readPtrSync  <= readPtr;
                readPtrCopy  <= readPtrSync;

                if putNewData then
                    writePtr <= writePtrNext;
                    if writePtrNext(Valid_Addr_Range) = readPtrCopy(Valid_Addr_Range) then
                        ramFull <= true;
                    else
                        ramFull <= false;
                    end if;
                end if;

                if ramFull then
                    if writePtr = (not readPtrCopy(A_BITS) & readPtrCopy(A_BITS - 1 downto 0)) then
                        ramFull <= true;
                    else
                        ramFull <= false;
                    end if;
                end if;

            end if;
        end if;
    end process;
    putNewData <= putIn and not ramFull;
    fullOut    <= ramFull;

    memDataIn    <= dataIn;
    memWriteAddr <= unsigned(writePtr(A_BITS - 1 downto 0));

    -----------------------------------------------------------------------------
    -- Read clock domain
    -----------------------------------------------------------------------------
    blkOP : block
        signal currCount : unsigned(AN - 1 downto 0) := to_unsigned(1, AN);
    begin
        process(readClkIn)
        begin
            if rising_edge(readClkIn) then
                if readRstIn = '1' then
                    currCount <= to_unsigned(1, AN);
                elsif getNewData then
                    currCount <= currCount + 1;
                end if;
            end if;
        end process;
        -- compute next pointer value
        readPtrNext <= std_logic_vector(currCount(A_BITS) & (currCount(A_BITS - 1 downto 0) xor ('0' & currCount(A_BITS - 1 downto 1))));
    end block blkOP;

    process(readClkIn)
    begin
        if rising_edge(readClkIn) then
            if readRstIn = '1' then
                readPtr          <= (others => '0');
                writePtrSync     <= (others => '0');
                writePtrCopy     <= (others => '0');
                memDataAvailable <= false;
                outputValid      <= false;
            else
                writePtrSync <= writePtrPrev;
                writePtrCopy <= writePtrSync;
                if getNewData then
                    readPtr     <= readPtrNext;
                    if readPtrNext(Valid_Addr_Range) = writePtrCopy(Valid_Addr_Range) then
                        memDataAvailable <= false;
                    else
                        memDataAvailable <= true;
                    end if;
                    outputValid <= true;
                elsif gotData then
                    outputValid <= false;
                end if;

                if not memDataAvailable then
                    if readPtr = writePtrCopy then
                        memDataAvailable <= false;
                    else
                        memDataAvailable <= true;
                    end if;
                end if;

            end if;
        end if;
    end process;
    getNewData  <= (not outputValid or gotData) and memDataAvailable;
    memReadAddr <= unsigned(readPtr(A_BITS - 1 downto 0));

    -----------------------------------------------------------------------------
    -- Add register to data output
    --
    -- Not needed if DATA_REG = true, because "dout" is already feed from a
    -- register in that case.
    -----------------------------------------------------------------------------
    genRegN : if DATA_REG or not OUTPUT_REG generate
        gotData <= gotIn;
        dataOut <= memDataOut;
        valid   <= outputValid;
    end generate genRegN;
    genRegY : if (not DATA_REG) and OUTPUT_REG generate
        signal dataOutBuf : Data_T  := (others => '-');
        signal VldB       : boolean := false;
    begin
        process(readClkIn)
        begin
            if rising_edge(readClkIn) then
                if readRstIn = '1' then
                    dataOutBuf <= (others => '-');
                    VldB       <= false;
                elsif gotData then
                    dataOutBuf <= memDataOut;
                    VldB       <= outputValid;
                end if;
            end if;
        end process;
        gotData <= not VldB or gotIn;
        dataOut <= dataOutBuf;
        valid   <= VldB;
    end generate genRegY;

    -----------------------------------------------------------------------------
    -- Fill State
    -----------------------------------------------------------------------------
    -- Write Clock Domain
    gEstateWr : if ESTATE_WR_BITS >= 1 generate
        signal d : unsigned(A_BITS - 1 downto 0);
    begin
        d             <= unsigned(gray2bin(readPtrCopy(A_BITS - 1 downto 0))) + not unsigned(gray2bin(writePtr(A_BITS - 1 downto 0)));
        writeStateOut <= (others => '0') when ramFull else std_logic_vector(d(d'left downto d'left - ESTATE_WR_BITS + 1));
    end generate gEstateWr;
    gNoEstateWr : if ESTATE_WR_BITS = 0 generate
        writeStateOut <= "X";
    end generate gNoEstateWr;

    -- Read Clock Domain
    gFstateRd : if FSTATE_RD_BITS >= 1 generate
        signal d : unsigned(A_BITS - 1 downto 0);
    begin
        d            <= unsigned(gray2bin(writePtrCopy(A_BITS - 1 downto 0))) + not unsigned(gray2bin(readPtr(A_BITS - 1 downto 0)));
        readStateOut <= (others => '0') when (not memDataAvailable) else std_logic_vector(d(d'left downto d'left - FSTATE_RD_BITS + 1));
    end generate gFstateRd;
    gNoFstateRd : if FSTATE_RD_BITS = 0 generate
        readStateOut <= "X";
    end generate gNoFstateRd;

    -----------------------------------------------------------------------------
    -- Memory Instantiation
    -----------------------------------------------------------------------------
    gLarge : if not DATA_REG generate
        ram : entity PoC.ocram_sdp
            generic map(
                A_BITS => A_BITS,
                D_BITS => D_BITS
            )
            port map(
                wclk => writeClkIn,
                rclk => readClkIn,

                wce  => '1',
                rce  => to_sl(getNewData),
                we   => to_sl(putNewData),
                ra   => memReadAddr,
                wa   => memWriteAddr,
                d    => memDataIn,
                q    => memDataOut
            );
    end generate gLarge;

    gSmall : if DATA_REG generate
        -- Memory modelled as Array
        type regfile_t is array (0 to 2**A_BITS - 1) of std_logic_vector(D_BITS - 1 downto 0);
        signal regfile      : regfile_t;
        attribute ram_style : string;   -- XST specific
        attribute ram_style of regfile : signal is "distributed";

        -- Altera Quartus II: Allow automatic RAM type selection.
        -- For small RAMs, registers are used on Cyclone devices and the M512 type
        -- is used on Stratix devices. Pass-through logic is not required as
        -- reads do not occur on write addresses.
        -- Warning about undefined read-during-write behaviour can be ignored.
        attribute ramstyle : string;
        attribute ramstyle of regfile : signal is "no_rw_check";
    begin

        -- Memory State
        process(writeClkIn)
        begin
            if rising_edge(writeClkIn) then
                --synthesis translate_off
                if SIMULATION and (writeRstIn = '1') then
                    regfile <= (others => (others => '-'));
                else
                    --synthesis translate_on
                    if putNewData then
                        regfile(to_integer(memWriteAddr)) <= memDataIn;
                    end if;
                    --synthesis translate_off
                end if;
                --synthesis translate_on
            end if;
        end process;

        -- Memory Output
        process(readClkIn)
        begin                           -- process
            if rising_edge(readClkIn) then
                if SIMULATION and (readRstIn = '1') then
                    memDataOut <= (others => 'U');
                elsif getNewData then
                    if Is_X(std_logic_vector(memReadAddr)) then
                        memDataOut <= (others => 'X');
                    else
                        memDataOut <= regfile(to_integer(memReadAddr));
                    end if;
                end if;
            end if;
        end process;
    end generate gSmall;

end rtl;
