# Triple T specifications

## operation information and design specifications

*clock speed*<br>
targeted clock speed: 50MHz<br>
theoretical overclock (before total alu corruption) : ~135MHz<br>

*voltage range*
designed       : 5v0, 2v5<br>
minimum viable : 3v3, 1v65<br>
maximum        : 8v0, 4v0<br>

*power draw*
static         : ~23mA<br>
transient peak : ~0.3A<br>



# specifications
1 core, 3 threads<br>
up to 2 macro-ops per thread<br>

## register information
4 trit scratchpads : 2x3 + 1 hidden (part of 12t gp)<br>
1 trit flags       : 8 + 1 carry (part of 12t gp)<br>
12 trit mac ladder : 1 , 3x4 trit slices<br>
program counter    : 1, 12 trits, up to ~531k<br>
12 trit gp regs    : 4<br>
macro-op parameter : 6+6 , 1 per macro op<br>

*addressless registers*
12 trit mask / alias register<br>
4 trit mask / alias register<br>
1 trit mask / alias register<br>
4t microcode action counter, microcode parameter counter<br>

fetching (per thread)<br>
2 macro op prefetch<br>
2 micro op prefetch<br>


profiling support : error tracker, fencing, breakpoints, overflow trackers<br>
analysis          : register extraction, register write, deterministic execution<br>
word length       : 12 trits, ~531k values<br>
address length    : 48 bits, 256 TiB addressable memory space<br>
maximum word length : unlimited (given enough swap)<br>

standalone specs:

512 bytes + up to 1.5kb from shared video memory ram (for firmware)<br>
1x spi<br>
3x kind of spi kind of normal gpio<br>
5x io pins<br>

## ALU specs

full adder   : 4+4   in, 5 out<br>
multiplier   : 4+4   in, 8 out<br>
Add3         : 4+4+4 in, 5 out<br>

multiplication : yes, with software fallback<br>
subtraction    : yes<br>
tritshifting   : yes<br>
signed math    : yes<br>
tritwise       : 21 unique operations<br>

# theoretical performance
90 MIPS<br>
10 megaflops<br>

## videocore specs
color depth : 20 trits, 5 trits RGBA<br>
depth testing : 18 trits available depth<br>
max fill rate : 120 megapixels/s<br>
anistropic filtering : no<br>
anti aliasing  : MXAA<br>
max texture size : 243x243<br>
mipmapping levels: 2. solid color and full texture<br>
shader cores : 2<br>
geometry cores : 0<br>
ray tracing cores : 0<br>
texture sampling rate : every so or often<br>
video memory : 1.5KB, 0.5KB shared with threads<br>

## design use cases
~no clue~
