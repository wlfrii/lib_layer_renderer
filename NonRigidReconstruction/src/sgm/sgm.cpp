#include "sgm.h"
#include "dm_util.h"
#include <algorithm>
#include <vector>
#include <cassert>
#include <chrono>
#include <cstring>
#include <cmath>
using namespace std::chrono;


SGM::SGM()
    : _image_width(0), _image_height(0)
    , _l_image(nullptr), _r_image(nullptr)
    , _cost_size(0), _cost_init(nullptr)
    , _cost_aggr(nullptr)
    , _l_disparity_map(nullptr)
    , _r_disparity_map(nullptr)
    , _is_initialized(false)
{
    for(int i = 0; i < 8; i++){
        _cost_aggr_path[i] = nullptr;
    }
}


SGM::~SGM()
{
    release();
    _is_initialized = false;
}

#define d_min _option.min_disparity
#define d_max _option.max_disparity
#define d_range (d_max - d_min)

bool SGM::initialize(uint16_t width, uint16_t height, const Option& option)
{
    if(width <= 0 || height <= 0 || !option.isValid()) {
        return false;
    }

    _option = option;

    _image_width = width;
    _image_height = height;

    _cost_size = width * height * d_range;
    _cost_init = new uint16_t[_cost_size]();
    _cost_aggr = new uint16_t[_cost_size]();
    for(int i = 0; i < 8; i++){
        _cost_aggr_path[i] = new uint16_t[_cost_size]();
    }

    int img_size = width * height;
    _l_census_values = new uint32_t[img_size]();
    _r_census_values = new uint32_t[img_size]();
    _l_disparity_map = new float[img_size]();
    _r_disparity_map = new float[img_size]();

    _is_initialized = _cost_init && _cost_aggr && _l_census_values && _l_disparity_map;

    return _is_initialized;
}

bool SGM::match(const uint8_t *l_image, const uint8_t *r_image, float *l_disparity_map)
{
    if(!_is_initialized) {
        return false;
    }
    if (l_image == nullptr || r_image == nullptr) {
        return false;
    }

    _l_image = l_image;
    _r_image = r_image;

    computeCost();
    aggregateCost();
    computeDisparity(_l_disparity_map, true);

    if (_option.is_check_lr) {
        computeDisparity(_r_disparity_map, false);
        lrCheck();
    }

    if (_option.is_remove_speckles) {
        dm_util::removeSpeckles(_l_disparity_map, _image_width, _image_height, 1, _option.min_speckle_area, FLOAT_INF);
    }

    if(_option.is_fill_holes) {
        fillHoles();
	}

    dm_util::medianFilter(_l_disparity_map, _l_disparity_map, _image_width, _image_height);

    memcpy(l_disparity_map, _l_disparity_map, _image_width*_image_height*sizeof(float));

	return true;
}

bool SGM::reset(uint16_t width, uint16_t height, const Option& option)
{
    release();
    _is_initialized = false;

    return initialize(width, height, option);
}

void SGM::release()
{
    SAFE_DELETE(_cost_init);
    SAFE_DELETE(_cost_aggr);

    for(int i = 0; i < 8; i++){
        SAFE_DELETE(_cost_aggr_path[i]);
    }
    SAFE_DELETE(_l_census_values);
    SAFE_DELETE(_r_census_values);
    SAFE_DELETE(_l_disparity_map);
    SAFE_DELETE(_r_disparity_map);
}

void SGM::computeCost()
{
    uint8_t r = _option.census_block_radius;
    dm_util::censusTransform(_l_image, _l_census_values, _image_width, _image_height, r, r); 
    dm_util::censusTransform(_r_image, _r_census_values, _image_width, _image_height, r, r);

    // Calculate Cost base on Census transform which evaluates the Cost
    // by Hamming Distance
    for (int32_t v = 0; v < _image_height; v++) {
        for (int32_t u = 0; u < _image_width; u++)
        {
            // Compute Cost value
            for (int32_t d = d_min; d < d_max; d++) {
                int id = v * _image_width * d_range + u * d_range + (d - d_min);
                auto& cost = _cost_init[id];
                if (u - d < 0 || u - d >= _image_width) {
                    cost = UINT16_MAX;///2;
                    continue;
                }
                
                uint32_t census_val_l = _l_census_values[v * _image_width + u];
                uint32_t census_val_r = _r_census_values[v * _image_width + u - d];
                cost = dm_util::hammingDistance(census_val_l, census_val_r);
            }
        }
    }
}

void SGM::aggregateCost()
{
    auto P1 = _option.P1;
    auto P2 = _option.P2;

    for(int i = 0; i < _option.aggr_path_num; i++){
        dm_util::costAggregation(
                    _l_image, _cost_init, _image_width, _image_height, d_range, P1, P2,
                    dm_util::AggregationPath(i), _cost_aggr_path[i]);
    }

    // Summing the 4/8 aggregated cost
    for (uint32_t n = 0; n < _cost_size; n++) {
        uint16_t res = 0;
        for(int i = 0; i < _option.aggr_path_num; i++){
            res += _cost_aggr_path[i][n];
        }
        _cost_aggr[n] = res;
    }
}

void SGM::computeDisparity(float* disparity, bool is_left)
{
    std::vector<uint16_t> cost_local(d_range);
    
    // Compute the optimized disparity pixel by pixel
    for (uint16_t v = 0; v < _image_height; v++)
    {
        for (uint16_t u = 0; u < _image_width; u++)
        {
            uint32_t pixel_idx = v * _image_width + u;

            uint16_t min_cost_1st = UINT16_MAX;
            uint16_t min_cost_2nd = UINT16_MAX;
            uint16_t best_disparity = 0;
            // --------------------
            // Find the disparity corresponding to the minimum cost value
            // while finding the second minimum cost value for consistency check
            for(uint16_t d = d_min; d < d_max; d++)
            {
                int d_idx = d - d_min;
                uint16_t cost = cost_local[d_idx] = _cost_aggr[pixel_idx * d_range + d_idx];
                // For right disparity, based on, R_cost(xr+d,yr,d) = L_cost(xr,yl,d)
                if(!is_left){
                    if(u < _image_width - d){
                        cost = cost_local[d_idx] = _cost_aggr[(pixel_idx + d) * d_range + d_idx];
                    }
                    else{
                        cost_local[d_idx] = UINT16_MAX;
                        continue;
                    }
                }

                if(min_cost_1st > cost) {
                    min_cost_2nd = min_cost_1st;
                    min_cost_1st = cost;
                    best_disparity = d;
                }
                else if(min_cost_2nd > cost) {
                    min_cost_2nd = cost;
                }
            }

            // Check the invalid disparity value
            // --------------------
            if (best_disparity == d_min || best_disparity == d_max - 1) {
                disparity[pixel_idx] = FLOAT_INF;
                continue;
            }
            if (_option.is_check_uniqueness) {
                // Do consistency check, referring to Eq.(15)
                // if (min_2nd)/min_1st < min_1st*(1-uniquness)，it is invalid
                uint16_t thresh = static_cast<uint16_t>(min_cost_1st * (1 - _option.uniqueness_ratio));
                if (min_cost_2nd - min_cost_1st <= thresh) {
                    disparity[pixel_idx] = FLOAT_INF;
                    continue;
                }
            }
            // Estimating the disparity of subpixel by a quadratic curve fitted through the
            // neighboring costs.
            // --------------------
            // Denoting the _1 and _2 as the former and later of the most optimized disprity.
            const uint16_t cost_1 = cost_local[best_disparity - 1 - d_min];
            const uint16_t cost_2 = cost_local[best_disparity + 1 - d_min];
            // Get disparity via the quadratic curve
            const uint16_t denom = std::max(1, cost_1 + cost_2 - 2 * min_cost_1st);
            disparity[pixel_idx] = static_cast<float>(best_disparity) +
                    static_cast<float>(cost_1 - cost_2) / (denom * 2.0f);
        }
    }
}

void SGM::lrCheck()
{
    _occlusions.clear();
    _mismatches.clear();

    // Check LR mapping pixel by pixel
    for (uint16_t v = 0; v < _image_height; v++)
    {
        for (uint16_t u = 0; u < _image_width; u++)
        {
            // Disparity value of pixel in left image
            auto& l_d = _l_disparity_map[v * _image_width + u];
            if(l_d == FLOAT_INF){
                _mismatches.emplace_back(v, u);
				continue;
			}

            // Find the correspoinding pixel in the right image
            const auto r_idx = static_cast<int32_t>(u - l_d + 0.5);
            
            if(r_idx >= 0 && r_idx < _image_width) {
                // Disparity value of correspinding pixel in right image
                const auto& r_d = _r_disparity_map[v * _image_width + r_idx];
                
                // LR check
                if (abs(l_d - r_d) > _option.lr_check_thresh) {
                    /* Distinguish the occlusion and mismatch by mapping the right
                     * disparity value into left image, and get the dr_in_L
                     *   if (mapped l_d from r_d > l_d)
                     *      pixel in occlusions
                     *   else
                     *      pixel in mismatches
                     */
                    const int32_t l_idx = static_cast<int32_t>(r_idx + r_d + 0.5);
                    if(l_idx > 0 && l_idx < _image_width){
                        const auto& mapped_l_d = _l_disparity_map[v*_image_width + l_idx];
                        if(mapped_l_d > l_d) {
                            _occlusions.emplace_back(v, u);
						}
						else {
                            _mismatches.emplace_back(v, u);
						}
					}
                    else _mismatches.emplace_back(v, u);

                    l_d = FLOAT_INF;
                }
            }
            else{
                // The invalid situation
                l_d = FLOAT_INF;
                _mismatches.emplace_back(v, u);
            }
        }
    }

}

void SGM::fillHoles()
{
    std::vector<float> d_collects;

    // Define 8 different direction for finding holes
    const float pi = 3.1415926f;
    float angles_1[8] = { pi, 3 * pi / 4, pi / 2, pi / 4, 0, 7 * pi / 4, 3 * pi / 2, 5 * pi / 4 };
    float angles_2[8] = { pi, 5 * pi / 4, 3 * pi / 2, 7 * pi / 4, 0, pi / 4, pi / 2, 3 * pi / 4 };
    float *angles = angles_1;

    // There is no need to search the far pixel
    const uint16_t max_search_length = _option.max_disparity;

    // Loop three times to process the occlusions, mismatches, and others
    for (uint8_t k = 0; k < 3; k++)
    {
        auto& coords = (k == 0) ? _occlusions : _mismatches;

        std::vector<std::pair<int, int>> invalid_coord;
		if (k == 2) {
            // Loop 3, process the others
            for (int i = 0; i < _image_height; i++) {
                for (int j = 0; j < _image_width; j++) {
                    if (_l_disparity_map[i * _image_width + j] == FLOAT_INF) {
                        invalid_coord.emplace_back(i, j);
					}
				}
			}
            coords = invalid_coord;
		}

        if (coords.empty()) {
            continue;
        }

        std::vector<float> fill_disparity(coords.size());

        for (auto n = 0u; n < coords.size(); n++) {
            auto& coord = coords[n];
            const uint16_t v = coord.first;
            const uint16_t u = coord.second;

            if (v == _image_height / 2) {
                angles = angles_2;
			}

            // Collect the number of invalid value in eight direction of current pixel
            d_collects.clear();
            for (int32_t count = 0; count < 8; count++) {
                float angle = angles[count];
                for (int32_t m = 1; m < max_search_length; m++) {
                    const int32_t vv = lround(v + m * sinf(angle));
                    const int32_t uu = lround(u + m * cosf(angle));
                    if (vv < 0 || vv >= _image_height || uu < 0 || uu >= _image_width) {
						break;
					}
                    float d = _l_disparity_map[vv*_image_width + uu];
                    if (d != FLOAT_INF) {
                        d_collects.push_back(d);
						break;
					}
				}
			}
            if(d_collects.empty()) {
				continue;
			}

            std::sort(d_collects.begin(), d_collects.end());

            if (k == 0) { // For occulusion, filling the disparity by secondly minimum disparity
                if (d_collects.size() > 1) {
                    fill_disparity[n] = d_collects[1];
				}
				else {
                    fill_disparity[n] = d_collects[0];
				}
			}
            else { // For mismatches, filling the disparity by the median disparity
                fill_disparity[n] = d_collects[d_collects.size() / 2];
            }
		}

        // Since the _l_disparity_map should not be changed before the above for-loop
        // We update the value of _l_disparity_map in this step
        for (auto n = 0u; n < coords.size(); n++) {
            auto& coord = coords[n];
            const int32_t v = coord.first;
            const int32_t u = coord.second;
            _l_disparity_map[v * _image_width + u] = fill_disparity[n];
        }
	}
}

