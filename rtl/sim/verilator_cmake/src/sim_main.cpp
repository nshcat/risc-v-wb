#include <memory>
#include <string>
#include <iostream>

#include <verilated.h>
#include <verilated_fst_c.h>

#include "Vtop_verilator.h"

using top_module_t = Vtop_verilator;

vluint64_t main_time{ };

// Called by $time in Verilog
double sc_time_stamp()
{
    return main_time;
}

void simulate(const std::unique_ptr<top_module_t>& top)
{
	// VCD tracing output object
    const auto tfp = std::make_unique<VerilatedFstC>();

	Verilated::traceEverOn(true);  	// Verilator must compute traced signals
	top->trace(tfp.get(), 99);  			// Trace 99 levels of hierarchy
	Verilated::mkdir("logs");
	tfp->open("waveforms.fst");
	
	// Set some inputs
    top->clk_i = 0;
	top->reset_i = 0;
	main_time++;

	top->eval();

	tfp->dump(main_time);

	main_time++;
	top->reset_i = 0;

	top->eval();

	tfp->dump(main_time);
	
	main_time++;

	top->clk_i = 1;

	top->eval();

	tfp->dump(main_time);
	
	main_time++;

	top->reset_i = 1;

	top->eval();

    // Dump trace data for this cycle
	tfp->dump(main_time);

	main_time++;

	top->clk_i = 0;

	top->eval();

	tfp->dump(main_time);

	const auto maxCycles = 500;

    // Simulate until $finish, or maximum number of cycles reached
    while (!Verilated::gotFinish() && (main_time / 10) < maxCycles)
	{
        main_time++;  // Time passes...
			
        // Toggle clocks and such
		/*if((main_time % 10) == 2 && main_time == 152)
			top->int_ext1 = 0;*/

        if ((main_time % 10) == 3) {
            top->clk_i = 1;
        }
        if ((main_time % 10) == 8) {
            top->clk_i = 0;
			//top->int_ext1 =1;
        }

        // Evaluate model
        top->eval();

        // Dump trace data for this cycle
		tfp->dump(main_time);
    }

    // Final model cleanup
    top->final();

    // Close trace if opened
	tfp->close();
}

int main(int argc, char** argv, char** env)
{
    Verilated::debug(0);
    Verilated::commandArgs(argc, argv);
	
	if(argc <= 1)
	{
		std::cout << "socsim - no arguments given" << std::endl;
	}
	
	std::string mode{ argv[1] };
	
	if(mode == "--sim")
	{
		// Construct the Verilated model, from Vtop.h generated from Verilating "top.v"
		const auto tb = std::make_unique<Vtop_verilator>();

		simulate(tb);
	}
	else if (mode == "--interactive")
	{
		std::cout << "socsim - interactive mode not implemented" << std::endl;
	}
	else
	{
		std::cout << "socsim - invalid mode argument" << std::endl;
	}

    

    return 0;
}
