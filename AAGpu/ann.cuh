#pragma once

#include "matrix.cuh"
#include <math.h>

template <unsigned Inputs, unsigned Neurons, unsigned Outputs> struct ann
{
    const static unsigned Pars = (1 + Inputs) * Neurons + Outputs * (1 + Neurons);

    matrix<Neurons, Inputs> weights1;
    matrix<Neurons, 1> biases1;
    matrix<Outputs, Neurons> weights2;
    matrix<Outputs, 1> biases2;

    matrix<Inputs, 2> input_norm;
    matrix<Outputs, 2> output_norm;

    __device__ __host__ void init(const matrix<Pars, 1> &pars)
    {
        int i = 0;
        for (int j = 0; j < Neurons * Inputs; j++)
            weights1.data[j] = pars.data[i++];
        for (int j = 0; j < Neurons; j++)
            biases1.data[j] = pars.data[i++];
        for (int j = 0; j < Outputs * Neurons; j++)
            weights2.data[j] = pars.data[i++];
        for (int j = 0; j < Outputs; j++)
            biases2.data[j] = pars.data[i++];
    }

    __device__ __host__ matrix<Outputs, 1> eval(const matrix<Inputs, 1> &input)
    {
        // first normalize
        matrix<Inputs, 1> ninput;
        for (int i = 0; i < Inputs; i++)
            ninput(i, 0) = 2.0f * (input(i, 0) - 0.5f * (input_norm(i, 0) + 
                input_norm(i, 1))) / (input_norm(i, 1) - input_norm(i, 0));

        // eval
        auto net1 = weights1 * ninput + biases1;

        // apply tansig
        for (int i = 0; i < Neurons; i++)
            net1(i, 0) = tanhf(net1(i, 0));

        // eval 2nd layer
        auto net2 = weights2 * net1 + biases2;

        // denormalize output
        for (int i = 0; i < Outputs; i++)
            net2(i, 0) = 0.5f * (net2(i, 0) * (output_norm(i, 1) - output_norm(i, 0)) +
                (output_norm(i, 1) + output_norm(i, 0)));

        return net2;
    }
};
