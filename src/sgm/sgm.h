/**
 * @file sgm.h
 *
 * @author Longfei Wang (longfei.wang@sjtu.end.cn)
 *
 * @brief This class is designed to calculate the stereo-disparity.
 *
 * References:
 * [1] H. Hirschmuller, “Stereo Processing by Semiglobal Matching and Mutual
 *     Information,” IEEE Transactions on Pattern Analysis and Machine Intelligence,
 *     vol. 30, no. 2, pp. 328–341, Feb. 2008, doi: 10.1109/TPAMI.2007.1166.
 * [2] https://github.com/ethan-li-coding/SemiGlobalMatching
 *
 * This file is completed with much help of the shared online documents, including
 * the blogs and codes, by Dr. Yingsong Li [2].
 *
 * NOTE. Since the uniqueness constraints and speckles removal processes are time
 * comsuming, these two step can be omitted when processed on GPU.
 *
 * @version 0.1
 *
 * @date 2021-08-31
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef SGM_H_LF
#define SGM_H_LF
#include <vector>
#include <cstdint>
#include <array>
#include <limits>

namespace mlayer{

class SGM
{
public:
    static constexpr auto FLOAT_INF = std::numeric_limits<float>::infinity();

    SGM();
    ~SGM();

    struct Option
    {
        Option()
            : min_disparity(1)
            , max_disparity(64)
            , census_block_radius(2)
            , aggr_path_num(8)
            ,
                     is_check_uniqueness(true), uniqueness_ratio(0.95f),
                     is_check_lr(true), lr_check_thresh(1.0f),
                     is_remove_speckles(true), min_speckle_area(20),
                     is_fill_holes(true),
                     P1(10), P2(150)
        { }

        bool isValid() const
        {
            int d_range = max_disparity - min_disparity;
            if(d_range <= 0) return false;

            return true;
        }

        int16_t min_disparity;		//!< The possibly minmum disparity value
        int16_t	max_disparity;		//!< The possibly maxmum disparity value

        uint8_t census_block_radius;

        uint8_t	aggr_path_num;      //!< The number of aggregation path, 4 or 8.

        bool	is_check_uniqueness;//!< Whether do consistency check for permitting one to one mappings only
        float	uniqueness_ratio;	//!< The threshold for uniquesness constraints

        bool	is_check_lr;		//!< Whether do LR check.
        float	lr_check_thresh;	//!< The threshold for LR check;

		bool	is_remove_speckles;	//!< Whether remove speckles
        int		min_speckle_area;	//!< The minimum speckle area

		bool	is_fill_holes;		//!< Whether fill holes

		// P1,P2 
		// P2 = P2_init / (Ip-Iq)
        int16_t  P1;
        int16_t  P2;
	};
public:

    bool initialize(uint16_t width, uint16_t height, const Option &option);
    bool match(const uint8_t* l_image, const uint8_t* r_image, float* l_disparity);
    bool reset(uint16_t width, uint16_t height, const Option &option);

private:
    void release();

    void computeCost();

    void aggregateCost();

    /**
     * @brief The disparity image that corresponds to the base image is determined as
     * in local stereo methods by selecting, for each pixel, the disparity d that
     * corresponds to the minimum cost.
     * @param disparity Left or Right disparity
     * @param Specify the left or right, left -> true, right -> false.
     */
    void computeDisparity(float* disparity, bool is_left);

    void lrCheck();

    void fillHoles();

private:

    Option _option;

    uint16_t _image_width;
    uint16_t _image_height;

    const uint8_t* _l_image;
    const uint8_t* _r_image;

    uint32_t*   _l_census_values;
    uint32_t*   _r_census_values;

    uint32_t    _cost_size;        // _image_width * _image_height * disparity_range
    uint16_t*   _cost_init;
    uint16_t*   _cost_aggr;

    std::array<uint16_t*, 8> _cost_aggr_path;

    float* _l_disparity_map;
    float* _r_disparity_map;

    bool _is_initialized;

    std::vector<std::pair<int, int>> _occlusions;
    std::vector<std::pair<int, int>> _mismatches;
};

} // namespace::mlayer
#endif // SGM_H_LF