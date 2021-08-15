/**
 * @author Marc Eisoldt
 * @author Malte Hillmann
 * @author Marcel Flottmann
 */

#include <map/local_map_hw.h>
#include <util/constants.h>
#include <util/point_hw.h>
#include <util/tsdf_hw.h>

#include <iostream>

#include <hls_stream.h>

using namespace fastsense::map;

/// An estimate of the number of points in a cloud for the Vitis Cycle estimation
constexpr int NUM_POINTS = 6000;
/// An estimate of the number of cells in the map for the Vitis Cycle estimation
constexpr int NUM_CELLS = 200 * 200 * 100;

struct StreamMessage
{
    TSDFValueHW value;
    PointHW index;
    PointHW interpolation_start;
    PointArith interpolation_step;
    int iter_steps;
    int iter_middle;
};

extern "C"
{
    /**
     * @brief generates data for update_tsdf
     *
     * executes raymarching from the Scanner to the Point, calculates the TSDF Value for each Cell
     * and sends that data to update_tsdf for interpolation
     *
     * @param scanPoints an array of Points
     * @param numPoints length of scanPoints
     * @param map metadata of the LocalMap. Needed for Scanner position and Map size
     * @param tau the truncation distance for tsdf values
     * @param up up-vector for the orientation of the Scanner
     * @param message_fifo fifo to send data to update_tsdf
     */
    void read_points(PointHW* scanPoints,
                     int numPoints,
                     const LocalMapHW& map,
                     TSDFValueHW::ValueType tau,
                     int dz_per_distance,
                     const PointHW& up,
                     hls::stream<StreamMessage>& message_fifo)
    {
        // Position of the Scanner
        PointHW map_pos{map.posX, map.posY, map.posZ};

        // grace period around the Point before the weight of a Point decreases
        TSDFValueHW::ValueType weight_epsilon = tau / 10;

        // rough estimate for the maximum raymarching distance
        int max_distance = (map.sizeX / 2 + map.sizeY / 2 + map.sizeZ / 2) * MAP_RESOLUTION;

    points_loop:
        for (int point_idx = 0; point_idx < numPoints; point_idx++)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/TSDF_SPLIT_FACTOR

            PointHW scan_point = scanPoints[point_idx];

            PointHW direction = scan_point - map_pos.to_mm();

            int distance = direction.norm();
            // end of raymarching: truncation distance behind the Point
            int distance_tau = distance + tau;

            auto normed_direction_vector = (PointArith(direction.x, direction.y, direction.z) * MATRIX_RESOLUTION) / distance;

            // interpolation vector should be perpendicular to the direction vector and should be on the same Plane as the up-vector
            // => calculate adjusted-up-vector = direction X right, with right-vector = direction X original-up-vector
            auto interpolation_vector = (normed_direction_vector.cross(normed_direction_vector.cross(PointArith(up.x, up.y, up.z)) / MATRIX_RESOLUTION));

            auto normed_interpolation_vector = (interpolation_vector * MATRIX_RESOLUTION) / interpolation_vector.norm();

            if (distance_tau > max_distance)
            {
                distance_tau = max_distance;
            }

            // the main Raymarching Loop
            // start at MAP_RESOLUTION to avoid problems with the Scanner pos
            // step in half-cell-size steps to possibly catch multiple cells on a slope
            //     this would lead to some cells being visited twice, which is filtered by update_tsdf
            //     the filter used to be inside this function, but that would prevent pipelining with II=1,
            //     because the rest of the loop would then depend on that calculation and Vitis says no
        tsdf_loop:
            for (int len = MAP_RESOLUTION; len <= distance_tau; len += MAP_RESOLUTION / 2)
            {
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=0 max=128

                PointHW proj = map_pos.to_mm() + direction * len / distance;
                PointHW index = proj.to_map();

                if (!map.in_bounds(index.x, index.y, index.z))
                {
                    continue;
                }

                TSDFValueHW tsdf;
                auto value = (scan_point - index.to_mm()).norm();
                if (value > tau)
                {
                    tsdf.value = tau;
                }
                else
                {
                    tsdf.value = value;
                }

                if (len > distance)
                {
                    // tsdf is negative behind the Point
                    tsdf.value = -tsdf.value;
                }

                tsdf.weight = WEIGHT_RESOLUTION;

                // weighting function:
                // weight = 1 (aka WEIGHT_RESOLUTION) from Scanner to Point
                // linear descent to 0 after the Point, starting at Point + weight_epsilon
                if (tsdf.value < -weight_epsilon)
                {
                    tsdf.weight = WEIGHT_RESOLUTION * (tau + tsdf.value) / (tau - weight_epsilon);
                }

                if (tsdf.weight == 0)
                {
                    continue;
                }

                StreamMessage msg;
                msg.value = tsdf;
                msg.index = index;

                // delta_z == how many cells should be interpolated to fill the area between rings
                int delta_z = dz_per_distance * len / MATRIX_RESOLUTION;

                msg.iter_steps = (delta_z * 2) / MAP_RESOLUTION + 1;
                msg.iter_middle = delta_z / MAP_RESOLUTION;

                auto lowest = PointArith(proj.x, proj.y, proj.z) - ((normed_interpolation_vector * delta_z) / MATRIX_RESOLUTION);
                msg.interpolation_start = PointHW(lowest.x, lowest.y, lowest.z);
                msg.interpolation_step = normed_interpolation_vector;

                message_fifo << msg;
            }
        }
        // send a final dummy message to terminate the update_loop
        StreamMessage dummy_msg;
        dummy_msg.value.weight = 0;
        message_fifo << dummy_msg;
    }

    /**
     * @brief interpolates a tsdf-value along a path to fill the area between rings
     *
     * @param map metadata of the LocalMap
     * @param new_entries temporary local map for newly generated values
     * @param value_fifo fifo to receive the calculated tsdf value from read_points
     * @param index_fifo fifo to receive the current cell from read_points
     * @param bounds_fifo fifo to receive (starting point, interpolation vector) from read_points
     * @param iter_steps_fifo fifo to receive (number of interpolation steps, index of current cell) from read_points
     */
    void update_tsdf(const LocalMapHW& map,
                     TSDFValueHW* new_entries,
                     hls::stream<StreamMessage>& message_fifo)
    {
        StreamMessage msg;
        msg.iter_steps = 0;
        PointHW index{0, 0, 0}, old_index{0, 0, 0};
        int step = 1;

    update_loop:
        while (true)
        {
#pragma HLS loop_tripcount max=NUM_POINTS/TSDF_SPLIT_FACTOR*128*2
#pragma HLS pipeline II=1
#pragma HLS dependence variable=new_entries inter false

            if (step > msg.iter_steps)
            {
                old_index = index;

                message_fifo >> msg;

                index = msg.index;

                // the weight is only 0 in the dummy message
                if (msg.value.weight == 0)
                {
                    break;
                }
                step = 0;
            }

            // tsdf_loop iterates in half-MAP_RESOLUTION steps => might visit the same cell twice
            // this check used to be done at the beginning of the tsdf_loop, but that leads to timing violations
            if (index != old_index)
            {
                auto index_arith = PointArith(msg.interpolation_start.x, msg.interpolation_start.y, msg.interpolation_start.z) + ((msg.interpolation_step * (step * MAP_RESOLUTION)) / MATRIX_RESOLUTION);
                index = PointHW(index_arith.x, index_arith.y, index_arith.z) / MAP_RESOLUTION;

                int map_index = map.getIndex(index.x, index.y, index.z);

                TSDFValueHW old_entry = new_entries[map_index];

                // interpolated values have a negative weight
                bool old_is_interpolated = old_entry.weight <= 0;
                bool current_is_interpolated = step != msg.iter_middle;
                bool current_is_better = hls_abs(msg.value.value) < hls_abs(old_entry.value) || old_entry.weight == 0;

                if ((current_is_better || old_is_interpolated) && map.in_bounds(index.x, index.y, index.z))
                {
                    TSDFValueHW tmp_value = old_entry;
                    // we always want the smallest tsdf value in each cell, even from interpolated values
                    if (current_is_better)
                    {
                        tmp_value.value = msg.value.value;
                    }

                    if (old_is_interpolated)
                    {
                        // weight is only negative when old an new are both interpolated
                        tmp_value.weight = msg.value.weight * (current_is_interpolated ? -1 : 1);
                    }

                    new_entries[map_index] = tmp_value;
                }

                step++;
            }
            else
            {
                // The current Point has already been processed
                // => skip interpolation and take next Point from fifo
                step = msg.iter_steps + 1;
            }
        }
    }

    /**
     * @brief update the local map with the new values using floating average
     *
     * this function is called multiple times in parallel with different start and end indices
     *
     * @param mapData the local map
     * @param start the starting index of the range of this instance
     * @param end the end index of the range of this instance
     * @param new_entries the temporary map with the new values
     * @param max_weight the maximum weight for the floating average
     */
    void sync_loop(
        TSDFValueHW* mapData,
        int start,
        int end,
        TSDFValueHW* new_entries,
        TSDFValueHW::WeightType max_weight)
    {
        // Update the current map based on the new generated entries
        for (int index = start; index < end; index++)
        {
#pragma HLS loop_tripcount min=NUM_CELLS/TSDF_SPLIT_FACTOR max=NUM_CELLS/TSDF_SPLIT_FACTOR
#pragma HLS pipeline II=1
#pragma HLS dependence variable=mapData inter false
#pragma HLS dependence variable=new_entries inter false

            TSDFValueHW map_entry = mapData[index];
            TSDFValueHW new_entry = new_entries[index];

            int new_weight = map_entry.weight + new_entry.weight;

            // Averaging is only performed based on real measured entries and not on interpolated ones
            if (new_entry.weight > 0 && map_entry.weight > 0)
            {
                map_entry.value = (map_entry.value * map_entry.weight + new_entry.value * new_entry.weight) / new_weight;

                // Upper bound for the total weight. Ensures, that later updates have still an impact.
                if (new_weight > max_weight)
                {
                    new_weight = max_weight;
                }

                map_entry.weight = new_weight;
            }
            // An interpolated value will always be overwritten by a new one. Real values are always preferred
            else if (new_entry.weight != 0 && map_entry.weight <= 0)
            {
                map_entry.value = new_entry.value;
                map_entry.weight = new_entry.weight;
            }

            mapData[index] = map_entry;
        }
    }

    void tsdf_dataflower(PointHW* scanPoints0, // MARKER: TSDF SPLIT
                         PointHW* scanPoints1,
                         PointHW* scanPoints2,
                         PointHW* scanPoints3,
                         int step,
                         int last_step,
                         TSDFValueHW* new_entries0, // MARKER: TSDF SPLIT
                         TSDFValueHW* new_entries1,
                         TSDFValueHW* new_entries2,
                         TSDFValueHW* new_entries3,
                         const LocalMapHW& map,
                         TSDFValueHW::ValueType tau,
                         int dz_per_distance,
                         const PointHW& up)
    {
#pragma HLS dataflow

        // MARKER: TSDF SPLIT
        hls::stream<StreamMessage> message_fifo0;
#pragma HLS stream depth=16 variable=message_fifo0
        read_points(scanPoints0, step,
                    map, tau, dz_per_distance, up,
                    message_fifo0);
        update_tsdf(map, new_entries0, message_fifo0);

        hls::stream<StreamMessage> message_fifo1;
#pragma HLS stream depth=16 variable=message_fifo1
        read_points(scanPoints1, step,
                    map, tau, dz_per_distance, up,
                    message_fifo1);
        update_tsdf(map, new_entries1, message_fifo1);

        hls::stream<StreamMessage> message_fifo2;
#pragma HLS stream depth=16 variable=message_fifo2
        read_points(scanPoints2, step,
                    map, tau, dz_per_distance, up,
                    message_fifo2);
        update_tsdf(map, new_entries2, message_fifo2);

        hls::stream<StreamMessage> message_fifo3;
#pragma HLS stream depth=16 variable=message_fifo3
        read_points(scanPoints3, last_step,
                    map, tau, dz_per_distance, up,
                    message_fifo3);
        update_tsdf(map, new_entries3, message_fifo3);

    }

    void sync_looper(TSDFValueHW* mapData0,
                     TSDFValueHW* mapData1,
                     TSDFValueHW* mapData2,
                     TSDFValueHW* mapData3,
                     int end0,
                     int end1,
                     int end2,
                     int end3,
                     TSDFValueHW* new_entries0,
                     TSDFValueHW* new_entries1,
                     TSDFValueHW* new_entries2,
                     TSDFValueHW* new_entries3,
                     TSDFValueHW::WeightType max_weight)
    {
#pragma HLS dataflow
        // MARKER: TSDF SPLIT
        sync_loop(mapData0, 0, end0, new_entries0, max_weight);
        sync_loop(mapData1, end0, end1, new_entries1, max_weight);
        sync_loop(mapData2, end1, end2, new_entries2, max_weight);
        sync_loop(mapData3, end2, end3, new_entries3, max_weight);
    }

    /**
     * @brief Hardware implementation of the TSDF generation and update algorithm using bresenham
     *
     * @param scanPoints0 Point reference from which the TSDF data should be calculated
     * @param scanPoints1 Point reference from which the TSDF data should be calculated
     * @param scanPoints2 Point reference from which the TSDF data should be calculated
     * @param scanPoints3 Point reference from which the TSDF data should be calculated
     * @param numPoints Number of points which should be
     * @param mapData0 Map reference which should be used for the update
     * @param mapData1 Map reference which should be used for the update
     * @param mapData2 Map reference which should be used for the update
     * @param mapData3 Map reference which should be used for the update
     * @param sizeX Number of map cells in x direction
     * @param sizeY Number of map cells in y direction
     * @param sizeZ Number of map cells in z direction
     * @param posX X coordinate of the scanner
     * @param posY Y coordinate of the scanner
     * @param posZ Z coordinate of the scanner
     * @param offsetX X offset of the local map
     * @param offsetY Y offset of the local map
     * @param offsetZ Z offset of the local map
     * @param new_entries0 Reference to the temporal buffer for the calculated TSDF values
     * @param new_entries1 Reference to the temporal buffer for the calculated TSDF values
     * @param new_entries2 Reference to the temporal buffer for the calculated TSDF values
     * @param new_entries3 Reference to the temporal buffer for the calculated TSDF values
     * @param tau Truncation distance for the TSDF values (in map resolution)
     * @param max_weight Maximum for the weight of the map entries
     */
    void krnl_tsdf(PointHW* scanPoints0, // MARKER: TSDF SPLIT
                   PointHW* scanPoints1,
                   PointHW* scanPoints2,
                   PointHW* scanPoints3,
                   int numPoints,
                   TSDFValueHW* mapData0, // MARKER: TSDF SPLIT
                   TSDFValueHW* mapData1,
                   TSDFValueHW* mapData2,
                   TSDFValueHW* mapData3,
                   int sizeX,   int sizeY,   int sizeZ,
                   int posX,    int posY,    int posZ,
                   int offsetX, int offsetY, int offsetZ,
                   TSDFValueHW* new_entries0, // MARKER: TSDF SPLIT
                   TSDFValueHW* new_entries1,
                   TSDFValueHW* new_entries2,
                   TSDFValueHW* new_entries3,
                   TSDFValueHW::ValueType tau,
                   TSDFValueHW::WeightType max_weight,
                   int dz_per_distance,
                   int up_x, int up_y, int up_z)
    {
        // MARKER: TSDF SPLIT
#pragma HLS INTERFACE m_axi port=scanPoints0  offset=slave bundle=scan0mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=mapData0     offset=slave bundle=map0mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries0 offset=slave bundle=entry0mem latency=22 depth=18491

#pragma HLS INTERFACE m_axi port=scanPoints1  offset=slave bundle=scan1mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=mapData1     offset=slave bundle=map1mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries1 offset=slave bundle=entry1mem latency=22 depth=18491

#pragma HLS INTERFACE m_axi port=scanPoints2  offset=slave bundle=scan2mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=mapData2     offset=slave bundle=map2mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries2 offset=slave bundle=entry2mem latency=22 depth=18491

#pragma HLS INTERFACE m_axi port=scanPoints3  offset=slave bundle=scan3mem  latency=22 depth=360
#pragma HLS INTERFACE m_axi port=mapData3     offset=slave bundle=map3mem   latency=22 depth=18491
#pragma HLS INTERFACE m_axi port=new_entries3 offset=slave bundle=entry3mem latency=22 depth=18491

        PointHW up(up_x, up_y, up_z);


        LocalMapHW map{sizeX,   sizeY,   sizeZ,
                       posX,    posY,    posZ,
                       offsetX, offsetY, offsetZ};

        int step = numPoints / TSDF_SPLIT_FACTOR;
        int last_step = numPoints - (TSDF_SPLIT_FACTOR - 1) * step;
        tsdf_dataflower(scanPoints0, // MARKER: TSDF SPLIT
                        scanPoints1 + 1 * step,
                        scanPoints2 + 2 * step,
                        scanPoints3 + 3 * step,
                        step,
                        last_step,
                        new_entries0, // MARKER: TSDF SPLIT
                        new_entries1,
                        new_entries2,
                        new_entries3,
                        map, tau, dz_per_distance, up);

        int total_size = sizeX * sizeY * sizeZ;
        int sync_step = total_size / TSDF_SPLIT_FACTOR + 1;

        sync_looper(mapData0, // MARKER: TSDF SPLIT
                    mapData1,
                    mapData2,
                    mapData3,
                    1 * sync_step, // MARKER: TSDF SPLIT
                    2 * sync_step,
                    3 * sync_step,
                    total_size,
                    new_entries0, // MARKER: TSDF SPLIT
                    new_entries1,
                    new_entries2,
                    new_entries3,
                    max_weight);
    }
}
