# Triple T specifications

## operation information and design specifications

*clock speed*
targeted clock speed: 50MHz
theoretical overclock (before total alu corruption) : ~135MHz

*voltage range*
designed       : 5v0, 2v5
minimum viable : 3v3, 1v65 
maximum        : 8v0, 4v0

*power draw*
static         : ~23mA
transient peak : ~0.3A



# specifications
1 core, 3 threads
up to 2 macro-ops per thread

## register information
4 trit scratchpads : 2x3 + 1 hidden (part of 12t gp)
1 trit flags       : 8 + 1 carry (part of 12t gp)
12 trit mac ladder : 1 , 3x4 trit slices
program counter    : 1, 12 trits, up to ~531k
12 trit gp regs    : 4
macro-op parameter : 6+6 , 1 per macro op

*addressless registers*
12 trit mask / alias register
4 trit mask / alias register
1 trit mask / alias register
4t microcode action counter, microcode parameter counter

fetching (per thread)
2 macro op prefetch
2 micro op prefetch

profiling support : error tracker, fencing, breakpoints, overflow trackers
analysis          : register extraction, register write, deterministic execution
word length       : 12 trits, ~531k values
address length    : 48 bits, 256 TiB addressable memory space
maximum word length : unlimited (given enough swap)

standalone specs:
512 bytes + up to 1.5kb from shared video memory ram (for firmware)
1x spi
3x kind of spi kind of normal gpio
5x io pins

## ALU specs

full adder   : 4+4   in, 5 out
multiplier   : 4+4   in, 8 out
Add3         : 4+4+4 in, 5 out

multiplication : yes, with software fallback
subtraction    : yes
tritshifting   : yes
signed math    : yes
tritwise       : 21 unique operations

# theoretical performance
90 MIPS
10 megaflops

## videocore specs
color depth : 20 trits, 5 trits RGBA
depth testing : 18 trits available depth
max fill rate : 120 megapixels/s
anistropic filtering : no
anti aliasing  : MXAA
max texture size : 243x243
mipmapping levels: 2. solid color and full texture
shader cores : 2
geometry cores : 0
ray tracing cores : 0
texture sampling rate : every so or often
video memory : 1.5KB, 0.5KB shared with threads

## design use cases
~no clue~
