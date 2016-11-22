#pragma once

#define PARCOUNT(Inputs, Neurons, Outputs) (1 + Inputs) * Neurons + Outputs * (1 + Neurons)

#define AOAINPUTS 4
#define AOANEURONS 8
#define AOAOUTPUTS 1

#define AOAPARS PARCOUNT(AOAINPUTS, AOANEURONS, AOAOUTPUTS)