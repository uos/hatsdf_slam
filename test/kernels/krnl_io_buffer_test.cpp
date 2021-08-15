/**
 * @file krnl_io_buffer_test.cpp
 * @author Julian Gaal
 */

extern "C" {
    void krnl_io_buffer_test(unsigned int* io, int size)
    {
#pragma HLS INTERFACE m_axi port = io1 offset = slave bundle = gmem
#pragma HLS INTERFACE s_axilite port = io1 bundle = control
#pragma HLS INTERFACE s_axilite port = size bundle = control
#pragma HLS INTERFACE s_axilite port=return bundle=control
        for (int i = 0; i < size; i++)
        {
            io[i] = 2;
        }
    }
}
