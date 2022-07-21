#include "dm_util.h"
#include <algorithm>
#include <vector>
#include <queue>
#include <cmath>
#include <array>
#include <cstdio>

void dm_util::debug(const uint16_t* cost, uint16_t width, uint16_t height, uint16_t d_range)
{
    for (int v = 0; v < height; v++){
        for (int u = 0; u < width; u++)
        {
            if ((v+1)%20 == 0 && (u+1)%50 == 0)
            {
                printf("[v,u]=[%03d,%03d]: \t", v,u);
                for (int32_t d = 0; d < d_range; d++) {
                    int id = v * width * d_range + u * d_range + d;
                    printf("%d ",cost[id]);
                }
            }
            if ((v+1)%20 == 0 && (u+1)%50 == 0)
                printf("\n");
        }
    }
    printf("\n\n");
}

void dm_util::debug(const float* cost, uint16_t width, uint16_t height)
{
    for (int v = 0; v < height; v++){
        for (int u = 0; u < width; u++)
        {
            if ((v+1)%15 == 0 && (u+1)%40 == 0)
            {
                int id = v * width + u;
                printf("[v,u]=[%03d,%03d]: %.3f\t", v,u,cost[id]);
            }
        }
        if ((v+1)%15 == 0)
            printf("\n");
    }
    printf("\n\n");
}

void dm_util::censusTransform(const uint8_t* src, uint32_t* dst, uint16_t width,
    uint16_t height, uint8_t rw/* = 2*/, uint8_t rh/* = 2*/)
{
    if (src == nullptr || dst == nullptr) {
		return;
	}

    // Traverse the whole image
    for (int16_t v = rh; v < height - rh; v++) {
        for (int16_t u = rw; u < width - rw; u++) {
            const uint8_t gray_center = src[v * width + u];

            // Traverse the block with size rh*rw
            uint64_t census_val = 0;
            for (int16_t r = -rh; r <= rh; r++) {
                for (int16_t c = -rw; c <= rw; c++) {
                    // Create bitstring
					census_val <<= 1;
                    const uint8_t gray = src[(v + r) * width + u + c];
					if (gray < gray_center) {
						census_val += 1;
					}
				}
			}
            dst[v * width + u] = census_val;
		}
	}
}


uint32_t dm_util::hammingDistance(uint32_t x, uint32_t y)
{
    uint32_t dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

    return dist;
}

/* ---------------------------------------------------------------------------------- */
/*                                   Cost Aggregation                                 */
/* ---------------------------------------------------------------------------------- */
namespace {

/**
 * @brief Intialize some variables for current path
 */
#define preAggregate(gray_prev, cost_array_prev, min_cost_prev)             \
    memcpy(cost_aggr_path, cost_init_path, d_range * sizeof(uint16_t));      \
    /* The previous cost array in the path, two extra elements assigned for
       avoiding out of range in head/tail when aggregation. */              \
    std::vector<uint16_t> cost_array_prev(d_range + 2, UINT16_MAX);           \
    /* Retrieve the initial cost of previous pixel */                       \
    memcpy(&cost_array_prev[1], cost_aggr_path, d_range * sizeof(uint16_t)); \
    /* Caculate the minimum path cost of the the first pixel
       and regard it as the previous object for the later pixels */         \
    uint16_t min_cost_prev = UINT16_MAX;                                      \
    for (auto cost : cost_array_prev) {                                     \
        min_cost_prev = std::min(min_cost_prev, cost);                      \
    }                                                                       \
    /* Retive the gray value of first pixel in this path */                 \
    uint8_t gray_prev = *src_path


/**
 * @brief Refer to Ea.(13), calculating the cost along direction
 * @param P1
 * @param P2
 * @param gray_curr Gray value of current pixel
 * @param gray_prev Gray value of previous pixel
 * @param d_range
 * @param min_cost_prev
 * @param cost_init_path
 * @param cost_aggr_path
 * @param cost_array_prev
 */
void doAggregate(int P1, int P2, uint8_t gray_curr, uint8_t& gray_prev,
               uint16_t d_range, uint16_t& min_cost_prev, const uint16_t* cost_init_path,
               uint16_t* cost_aggr_path, std::vector<uint16_t>& cost_array_prev,
               bool is_print = false)
{
    /* Discontinuities are often visible as intensity changes. This is exploited
     * by adapting P2 to the intensity gradient.
     * It should be ensure that P2 >= P1.
     * A constant 1 added for avoiding dividing by zero.
     */
    int P2_real = std::max(P1, P2 / (abs(gray_curr - gray_prev) + 1));

    uint16_t min_cost_curr = UINT16_MAX;
    for (uint16_t d = 0; d < d_range; d++)
    {
        /* Refer to Eq.(13) in Hirschmuller. 2008. TPAMI
         * The cost Lr(p,d) alone a path traversed in the direction r
         * of the pixel p at disparity d is defined recursively as
         *   Lr(p,d) = C(p,d) + min( Lr(p-r,d),     --> Lr_1
         *             Lr(p-r,d-1) + P1,            --> Lr_2
         *             Lr(p-r,d+1) + P1,            --> Lr_3
         *             min(Lr(p-r,i)) + P2 )        --> Lr_4
         *              - min(Lr(p-r,k))
         */
        unsigned int cost = cost_init_path[d];
        // Note, it's possible to get a Lr_x greater than UINT8_MAX
        unsigned int Lr_1 = cost_array_prev[d + 1];
        unsigned int Lr_2 = uint32_t(cost_array_prev[d]) + P1;
        unsigned int Lr_3 = uint32_t(cost_array_prev[d + 2]) + P1;
        unsigned int Lr_4 = min_cost_prev + P2_real;

        // === 2022.3.21 === 这里有问题，Lr 会犹豫超过255而被截断，所以改成 uint16_t
        uint16_t Lr = std::min(cost + std::min(std::min(Lr_1, Lr_2), std::min(Lr_3, Lr_4)) - min_cost_prev, (unsigned int)UINT16_MAX);
        cost_aggr_path[d] = Lr;
        min_cost_curr = std::min(min_cost_curr, Lr);

        if (is_print){
            printf("cost: %d, Lr_1-4: %d %d %d %d, Lr: %d, P2_real: %d\n", 
                cost, Lr_1, Lr_2, Lr_3, Lr_4, Lr, P2_real);
        }
    }
    min_cost_prev = min_cost_curr;
    gray_prev = gray_curr;
    memcpy(&cost_array_prev[1], cost_aggr_path, d_range * sizeof(uint16_t));
}

#define updateAggregateParams(offset)   \
    cost_init_path += offset * d_range; \
    cost_aggr_path += offset * d_range; \
    src_path += offset


void aggregateLeftRight(const uint8_t* src,
                        const uint16_t* cost_init,
                        uint16_t width, uint16_t height,
                        uint16_t d_range,
                        int P1, int P2,
                        bool is_forward,
                        uint16_t* dst)
{
    int direction = is_forward ? 1 : -1;

    for (uint16_t v = 0; v < height; v++)
    {
        int64_t offset = (is_forward)? v * width : (v + 1) * width - 1;
        auto cost_init_path = cost_init + offset * d_range;
        auto cost_aggr_path = dst + offset * d_range;
        auto src_path = src + offset;

        preAggregate(gray_prev, cost_array_prev, min_cost_prev);

        for (uint16_t u = 1; u < width; u++) {
            offset = direction;
            updateAggregateParams(offset);
            
            ::doAggregate(P1, P2, *src_path, gray_prev, d_range, min_cost_prev,
                        cost_init_path, cost_aggr_path, cost_array_prev, false);
        }
    }

    // dm_util::debug(dst,width,height,d_range);
}

void aggregateTopBottom(const uint8_t* src,
                        const uint16_t* cost_init,
                        uint16_t width, uint16_t height,
                        uint16_t d_range,
                        int P1, int P2,
                        bool is_forward,
                        uint16_t* dst)
{
    int direction = is_forward ? 1 : -1;

    for (uint16_t u = 0; u < width; u++)
    {
        int64_t offset = (is_forward)? u : (height - 1) * width + u;
        auto cost_init_path = cost_init + offset * d_range;
        auto cost_aggr_path = dst + offset * d_range;
        auto src_path = src + offset;

        preAggregate(gray_prev, cost_array_prev, min_cost_prev);

        for (uint16_t v = 1; v < height; v++) {
            offset = direction * width;
            updateAggregateParams(offset);

            ::doAggregate(P1, P2, *src_path, gray_prev, d_range, min_cost_prev,
                        cost_init_path, cost_aggr_path, cost_array_prev);
        }
    }
}

void aggregateDiagonal_1(const uint8_t* src,
                         const uint16_t* cost_init,
                         uint16_t width, uint16_t height,
                         uint16_t d_range,
                         int P1, int P2,
                         bool is_forward,
                         uint16_t* dst)
{
    int direction = is_forward ? 1 : -1;

    int32_t current_row = 0;
    int32_t current_col = 0;

    for (uint16_t u = 0; u < width; u++)
    {
        int64_t offset = (is_forward)? u : (height - 1) * width + u;
        auto cost_init_path = cost_init + offset * d_range;
        auto cost_aggr_path = dst + offset * d_range;
        auto src_path = src + offset;

        preAggregate(gray_prev, cost_array_prev, min_cost_prev);

        current_row = is_forward ? 0 : height - 1;
        current_col = u;
        for (uint16_t v = 1; v < height; v ++)
        {
            if (is_forward && current_col == width - 1 && current_row < height - 1) {
                offset = (current_row + direction) * width;
                cost_init_path = cost_init + offset * d_range;
                cost_aggr_path = dst + offset * d_range;
                src_path = src + offset;
                current_col = 0;
            }
            else if (!is_forward && current_col == 0 && current_row > 0) {
                offset = (current_row + direction + 1) * width - 1;
                cost_init_path = cost_init + offset * d_range;
                cost_aggr_path = dst + offset * d_range;
                src_path = src + offset;
                current_col = width - 1;
            }
            else {
                offset = direction * (width + 1);
                updateAggregateParams(offset);
            }

            ::doAggregate(P1, P2, *src_path, gray_prev, d_range, min_cost_prev,
                        cost_init_path, cost_aggr_path, cost_array_prev);

            current_row += direction;
            current_col += direction;
        }
    }
}

void aggregateDiagonal_2(const uint8_t* src,
                         const uint16_t* cost_init,
                         uint16_t width, uint16_t height,
                         uint16_t d_range,
                         int P1, int P2,
                         bool is_forward,
                         uint16_t* dst)
{
    int direction = is_forward ? 1 : -1;

    int32_t current_row = 0;
    int32_t current_col = 0;

    for (uint16_t u = 0; u < width; u++)
    {
        int64_t offset = (is_forward)? u : (height - 1) * width + u;
        auto cost_init_path = cost_init + offset * d_range;
        auto cost_aggr_path = dst + offset * d_range;
        auto src_path = src + offset;

        preAggregate(gray_prev, cost_array_prev, min_cost_prev);

        current_row = is_forward ? 0 : height - 1;
        current_col = u;
        for (uint16_t i = 0; i < height - 1; i++)
        {
            if (is_forward && current_col == 0 && current_row < height - 1) {
                offset = (current_row + direction + 1) * width - 1;
                cost_init_path = cost_init + offset * d_range;
                cost_aggr_path = dst + offset * d_range;
                src_path = src + offset;
                current_col = width - 1;
            }
            else if (!is_forward && current_col == width - 1 && current_row > 0) {
                offset = (current_row + direction) * width;
                cost_init_path = cost_init + offset * d_range ;
                cost_aggr_path = dst + offset * d_range;
                src_path = src + offset;
                current_col = 0;
            }
            else {
                offset = direction * (width - 1);
                updateAggregateParams(offset);
            }

            ::doAggregate(P1, P2, *src_path, gray_prev, d_range, min_cost_prev,
                        cost_init_path, cost_aggr_path, cost_array_prev);

            current_row += direction;
            current_col -= direction;
        }
    }
}

} // namspace

void dm_util::costAggregation(const uint8_t* src,
                              const uint16_t* cost_init,
                              uint16_t width, uint16_t height,
                              uint16_t d_range,
                              int P1, int P2,
                              AggregationPath path,
                              uint16_t* dst)
{
    switch(path)
    {
    case dm_util::AGGREGATION_PATH_L2R:
        ::aggregateLeftRight(src, cost_init, width, height, d_range, P1, P2, true, dst);
        break;
    case dm_util::AGGREGATION_PATH_R2L:
        ::aggregateLeftRight(src, cost_init, width, height, d_range, P1, P2, false, dst);
        break;
    case dm_util::AGGREGATION_PATH_T2B:
        ::aggregateTopBottom(src, cost_init, width, height, d_range, P1, P2, true, dst);
        break;
    case dm_util::AGGREGATION_PATH_B2T:
        ::aggregateTopBottom(src, cost_init, width, height, d_range, P1, P2, false, dst);
        break;
    case dm_util::AGGREGATION_PATH_LT2RB:
        ::aggregateDiagonal_1(src, cost_init, width, height, d_range, P1, P2, true, dst);
        break;
    case dm_util::AGGREGATION_PATH_RB2LT:
        ::aggregateDiagonal_1(src, cost_init, width, height, d_range, P1, P2, false, dst);
        break;
    case dm_util::AGGREGATION_PATH_RT2LB:
        ::aggregateDiagonal_2(src, cost_init, width, height, d_range, P1, P2, true, dst);
        break;
    case dm_util::AGGREGATION_PATH_LB2RT:
        ::aggregateDiagonal_2(src, cost_init, width, height, d_range, P1, P2, false, dst);
        break;
    default:
        break;
    }
}


/* ---------------------------------------------------------------------------------- */

void dm_util::medianFilter(const float* src, float* dst, uint16_t width, uint16_t height,
    uint16_t radius/* = 1*/)
{
    uint32_t size = std::pow(radius * 2 + 1, 2);

    std::vector<float> block;
    block.reserve(size);

    for (uint16_t v = 0; v < height; v++) {
        for (uint16_t u = 0; u < width; u++) {
            block.clear();
            for (int32_t r = -radius; r <= radius; r++) {
                for (int32_t c = -radius; c <= radius; c++) {
                    int32_t row = v + r;
                    int32_t col = u + c;
                    if (row >= 0 && row < height && col >= 0 && col < width) {
                        block.emplace_back(src[row * width + col]);
					}
				}
			}
            std::sort(block.begin(), block.end());
            dst[v * width + u] = block[block.size() / 2];
		}
	}
}

/* ---------------------------------------------------------------------------------- */

void dm_util::removeSpeckles(float* disparity_map, uint16_t width, uint16_t height,
    int32_t thresh, uint32_t min_speckle_area, float invalid_val)
{
    std::vector<bool> visited(uint32_t(width * height), false);

    for(uint16_t v = 0; v < height; v++)
    {
        for(uint16_t u = 0; u < width; u++)
        {
            uint32_t pixel_idx = v * width + u;
            // Ignore the checked or invalid pixel
            if (visited[pixel_idx] || disparity_map[pixel_idx] == invalid_val) {
				continue;
			}

            visited[pixel_idx] = true;

            std::vector<std::pair<uint16_t, uint16_t>> vec;
            vec.emplace_back(v, u);

            // Find the connected region
            for(uint32_t k = 0; k < vec.size(); k++) {
                const auto& coord = vec[k];
                uint16_t row = coord.first;
                uint16_t col = coord.second;
                const auto& d_base = disparity_map[row * width + col];

                // Center at 'coord', and check the adjacent pixel
                for(int r = row - 1; r <= row + 1; r++) {
                    for(int c = col - 1; c <= col + 1; c++) {
                        if(r == row && c == col)
                            continue;

                        if (r >= 0 && r < height && c >= 0 && c < width) {
                            uint32_t idx = r * width + c;
                            if(!visited[idx] && (disparity_map[idx] != invalid_val) &&
                                abs(disparity_map[idx] - d_base) <= thresh)
                            {
                                vec.emplace_back(r, c);
                                visited[idx] = true;
                            }
                        }
                    }
                }
            }

            if(vec.size() < min_speckle_area) {
                for(auto& coord : vec) {
                    disparity_map[coord.first * width + coord.second] = invalid_val;
				}
			}
		}
	}
}
