/**
 * @file disparity_map.h
 *
 * @author Longfei Wang (longfei.wang@sjtu.edu.cn)
 *
 * @brief This file is designed for achieve Stereo Vision Disparity Map
 * Algorithm.
 *
 * Current version includes:
 *   1. Matching Cost Compution
 *      - Cencus Transform.
 *
 * References
 * [1] R. A. Hamzah and H. Ibrahim, “Literature Survey on Stereo Vision
 *     Disparity Map Algorithms,” Journal of Sensors, vol. 2016, pp. 1–23,
 *     2016, doi: 10.1155/2016/8742920.
 *
 * @version 0.1
 *
 * @date 2021-08-31
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef DISPARITY_MAP_UTILITY_H_LF
#define DISPARITY_MAP_UTILITY_H_LF
#include <cstdint>

#ifndef SAFE_DELETE
#define SAFE_DELETE(P) {if(P) delete[](P);(P)=nullptr;}
#endif

/**
 * @brief dm is the abbreviation of disparity map.
 */
namespace dm_util
{
    void debug(const uint16_t* cost, uint16_t width, uint16_t height, uint16_t d_range);
    void debug(const float* cost, uint16_t width, uint16_t height);

/**
 * @brief Census transform, which is a method for Matching Cost Computation.
 *
 * Reference:
 * R. A. Hamzah and H. Ibrahim, “Literature Survey on Stereo Vision Disparity
 * Map Algorithms,” Journal of Sensors, vol. 2016, pp. 1–23, 2016,
 * doi: 10.1155/2016/8742920.
 * 
 * @param src The input image.
 * @param dst The output census value corresponding to the image.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param rw The radius in width direction.
 * @param rh The radius in height direction.
 */
void censusTransform(const uint8_t* src, uint32_t* dst,
                     uint16_t width, uint16_t height,
                     uint8_t rw = 2, uint8_t rh = 2);


/**
 * @brief The calculation of hamming distance.
 * 
 * @param x The first binary value (bitstring), denoted as integer
 * @param y The second binary value (bitstring), denoted as interger
 * @return uint8_t 
 */
uint32_t hammingDistance(uint32_t x, uint32_t y);


/**
 * @brief The type of aggregation path.
 * There are 8 supported aggregation path
 * 1. Left      <->   Right
 * 2. Up        <->   Down
 * 3. Left-Top  <->   Right-Down
 * 4. Right-Top <->   Left-Down
 *   ↘ ↓ ↙   5  3  7
 *   →   ← 	 1     2
 *   ↗ ↑ ↖   8  4  6
 */
enum AggregationPath
{
    AGGREGATION_PATH_L2R    = 0,  //!< left --> right
    AGGREGATION_PATH_R2L    = 1,  //!< right --> right
    AGGREGATION_PATH_T2B    = 2,  //!< top --> bottom
    AGGREGATION_PATH_B2T    = 3,  //!< bottom --> top
    AGGREGATION_PATH_LT2RB  = 4,  //!< left top --> right bottom
    AGGREGATION_PATH_RB2LT  = 5,  //!< right bottom --> left top
    AGGREGATION_PATH_RT2LB  = 6,  //!< right top --> left bottom
    AGGREGATION_PATH_LB2RT  = 7   //!< left bottom --> right top
};


/**
 * @brief The cost aggregation process to get aggregated cost value.
 * 
 * @param src The input image.
 * @param cost_init The initial cost calculated by Census Transform.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param d_range The possibal disparity range.
 * @param P1 The penalty for pixel which neighborhood's disparity changes a
 *           little bit (that is, 1 pixel).
 * @param P2 The penalty for larger disparity change.
 * @param path The aggregation path.
 * @param dst The output aggregated cost value.
 */
void costAggregation(const uint8_t* src, const uint16_t* cost_init,
                     uint16_t width, uint16_t height, uint16_t d_range,
                     int P1, int P2, AggregationPath path, uint16_t* dst);


/**
 * @brief Median filter
 * 
 * @param src The input image.
 * @param dst The output image.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param radius The radius of the filter kernel.
 */
void medianFilter(const float *src, float *dst,
                  uint16_t width, uint16_t height, uint16_t radius = 1);


/**
 * @brief A region search method to find the speckles.
 * 
 * @param disparity_map The input disparity map.
 * @param width The width of the input.
 * @param height The height of the input.
 * @param thresh The thresh to judge the same disparity value, when the
 *               difference of two pixel disparity value no greater than this
 *               thresh, these two disparity value will be considered same.
 * @param min_speckle_area The area to judge the speckle region, when a speckle
 *                         region's area less than this \param, the region will
 *                         be considered as speckles.
 * @param invalid_val The invalid valid to fill the speckles region.
 */
void removeSpeckles(float *disparity_map, uint16_t width, uint16_t height, int32_t thresh, uint32_t min_speckle_area, float invalid_val);

} // namespace::dm_util

#endif // DISPARITY_MAP_UTILITY_H_LF
