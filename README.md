this is a asic

dear god

theres more

no


uh anyways this is a ternary cpu
3 threads, 12 trits, infinite possibilities
technically more efficient than a binary chip but i'm the one designing it so no
pretty innovative if you ask me


can access up to ~9.7GB of ram, but the very poorly thought out design leaves much to be desired

has it's own isa and microcode architecture, with 7+1 modes designed to run in conjunction with an fpga to act as the chipset/ bridge, or can act standalone by reading spi flash

will update isa but it's in a google doc for now so until i do, this is where i get off


actual specs below

per thread:
16 registers of 12 trits each
2 registers can be addressed as 3x4 trit chunks
1 register can be addressed as 8x1+4 trits (flag register)

additionally up to 6 12t registers can be submitted with each command,
each thread holds up to 2 commands, thus the real register count is 22+A-E hidden registers
it gets more confusing

leveraging the advantages of balanced ternary logic, the alu can process addition, subtraction, multiplication in discrete 4 trit chunks, while an arbiter distributes requests from the 3 threads evenly to maximize uptime

below is a list of sub muxes that the core uses to set internal operation modes
