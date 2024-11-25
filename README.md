
# RISC-V Multicycle CPU Starter Code

## Guide to Run ECI ModelSim

1. Download this directory to ECI.
2. `cd` to the downloaded directory using a terminal.
3. Open a terminal and cd to your ECI directory.
4. Run `make run-gui` to compile your design and open ModelSim
5. Run simulations as normal.

Note that you may have to add this to your `"~/.bashrc"`:

```bash
# ModelSim
export MODEL_TECH=/ece/mentor/ModelSimSE-10.7d/modeltech/bin
export PATH=$PATH:$MODEL_TECH
export LM_LICENSE_FILE=1717@license.ece.ucsb.edu
```

## (Optional) Guide to Assemble MIPS Programs

With the given Makefile, you are able to assemble your own MIPS programs into hex.

1. To access SPIM, (not QtSpim), either login to CSIL, or [compile spim from source](https://sourceforge.net/p/spimsimulator/code/HEAD/tree/README#l130).
2. Then, run `make <filename>_text.dat` (ex. `make test_text.dat`).
