# Mango2 Computer Emulator
Our FPGA emulator for the Apple II. This emulator does not have a Disk Operating System and only supports 64 KB of memory.

**Apple2_top.sv** in the main directory. contains all of the systemVerilog files packaged into one file for ease of access. If you would like to view the files separately, you can do so in the SystemVerilogFiles folder. 

All ROMs, games, and simulator files are located in IP files folder

BIN folder has the binary files for non-volatile memory

Scripts contain the scripts used to convert .bin files to hex files for memory population

**DISCLAIMER** This is intended to run on the Basys3FPGA board and the NexysA7 FPGA board. All other boards will not work with the current configuration!
Also note, this will only run if you have Vivado Design Suite!

**insts** To run any project version on the FPGA, find the bit file in the project folder and run that bit file on Vivado for the FPGA.

More Instructions:

This will be located within the following ProjectFiles Folder:
1) apple2_lode.bit will run loderunner implementation without terminal
2) apple2_frogger.bit will run frogger without terminal
3) apple2_applesoft.bit will run apple soft with terminal

Team organization:

- Andrew Bauer (Memory and Audio Subsystem)
- Jerry Chen (Processor Integration and Bus System Design Subsystem)
- Ivan Usynin (Video Subsystem)
- Santiago Pineiros (Keyboard Subsystem)
