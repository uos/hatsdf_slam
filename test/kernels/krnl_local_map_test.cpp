/**
 *
 * @author Marcel Flottmann
 */

#include <map/local_map_hw.h>
#include <util/tsdf_hw.h>

extern "C"
{

    void krnl_local_map_test(TSDFValueHW* mapData,
                             int sizeX,
                             int sizeY,
                             int sizeZ,
                             int posX,
                             int posY,
                             int posZ,
                             int offsetX,
                             int offsetY,
                             int offsetZ
                            )
    {
#pragma HLS DATA_PACK variable=mapData
#pragma HLS INTERFACE m_axi port=mapData offset=slave bundle=gmem
#pragma HLS INTERFACE s_axilite port=mapData bundle=control
#pragma HLS INTERFACE s_axilite port=sizeX bundle=control
#pragma HLS INTERFACE s_axilite port=sizeY bundle=control
#pragma HLS INTERFACE s_axilite port=sizeZ bundle=control
#pragma HLS INTERFACE s_axilite port=posX bundle=control
#pragma HLS INTERFACE s_axilite port=posY bundle=control
#pragma HLS INTERFACE s_axilite port=posZ bundle=control
#pragma HLS INTERFACE s_axilite port=offsetX bundle=control
#pragma HLS INTERFACE s_axilite port=offsetY bundle=control
#pragma HLS INTERFACE s_axilite port=offsetZ bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

        fastsense::map::LocalMapHW map{sizeX, sizeY, sizeZ,
                                       posX, posY, posZ,
                                       offsetX, offsetY, offsetZ};

        for (int i = map.posX - map.sizeX / 2; i <= map.posX + map.sizeX / 2; i++)
        {
            for (int j = map.posY - map.sizeY / 2; j <= map.posY + map.sizeY / 2; j++)
            {
                for (int k = map.posZ - map.sizeZ / 2; k <= map.posZ + map.sizeZ / 2; k++)
                {
#pragma HLS PIPELINE
                    TSDFValueHW tmp = map.get(mapData, i, j, k);
                    tmp.value *= 2;
                    tmp.weight /= 2;
                    map.set(mapData, i, j, k, tmp);
                }
            }
        }
    }

}
