#pragma once

/**
 * @file linear_solver.h
 * @author Marcel Flottmann
 */

namespace fastsense::registration
{

template<typename T, int N>
void lu_decomposition(T A[N][N])
{
#pragma HLS inline
    for (int i = 0; i < N - 1; i++)
    {
#pragma HLS unroll
        for (int k = i + 1; k < N; k++)
        {
#pragma HLS unroll
            A[k][i] /= A[i][i];
            for (int j = i + 1; j < N; j++)
            {
#pragma HLS unroll
                A[k][j] -= A[k][i] * A[i][j];
            }
        }
    }
}

template<typename T, int N>
void lu_solve(T A[N][N], T b[N], T x[N])
{
#pragma HLS inline
    for (int i = 0; i < N; i++)
    {
#pragma HLS unroll
        x[i] = b[i];
        for (int k = 0; k < i; k++)
        {
#pragma HLS unroll
            x[i] -= A[i][k] * x[k];
        }
    }

    for (int i = N - 1; i >= 0; i--)
    {
#pragma HLS unroll
        for (int k = i + 1; k < N; k++)
        {
#pragma HLS unroll
            x[i] -= A[i][k] * x[k];
        }
        x[i] /= A[i][i];
    }
}

template<typename T, int N>
void lu_decomposition(T A[N][N], T L[N][N], T R[N][N])
{
#pragma HLS inline
    for (int i = 0; i < N; i++)
    {
#pragma HLS unroll
        for (int j = 0 ; j < N; j++)
        {
#pragma HLS unroll
            R[i][j] = A[i][j];
            L[i][j] = i == j ? 1 : 0;
        }
    }

    for (int i = 0; i < N - 1; i++)
    {
#pragma HLS unroll
        for (int k = i + 1; k < N; k++)
        {
#pragma HLS unroll
            L[k][i] = R[k][i] / R[i][i];
            for (int j = i; j < N; j++)
            {
#pragma HLS unroll
                R[k][j] -= L[k][i] * R[i][j];
            }
        }
    }
}

}