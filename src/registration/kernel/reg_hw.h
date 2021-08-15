#pragma once

/**
 * @file reg_hw.h
 * @author Marc Eisoldt (meisoldt@uos.de)
 */

namespace fastsense::registration
{

// Prototype of top level function for C-synthesis
template<typename T, int a_rows, int a_cols, int b_cols>
void MatrixMul(
    const T a[a_rows][a_cols],
    const T b[a_cols][b_cols],
    T res[a_rows][b_cols])
{
#pragma HLS INLINE

    for (int i = 0; i < a_rows; i++)
    {
#pragma HLS unroll

        for (int j = 0; j < b_cols; j++)
        {
#pragma HLS unroll
            res[i][j] = 0;

            for (int k = 0; k < a_cols; k++)
            {
#pragma HLS unroll
                res[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

template<typename T>
void transform_point(
    const T mat[4][4],
    const T point[3],
    T res[3])
{
#pragma HLS INLINE

    // Iterate over the rows of the A matrix
    for (int i = 0; i < 3; i++)
    {
#pragma HLS unroll
        // translation
        res[i] = mat[i][3];

        // rotation
        for (int k = 0; k < 3; k++)
        {
#pragma HLS unroll
            res[i] += mat[i][k] * point[k];
        }
    }
}

} // namespace fastsense::registration
