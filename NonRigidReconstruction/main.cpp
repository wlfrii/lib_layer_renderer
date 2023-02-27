#include "src/scene_reconstructor.h"
#include <opencv2/opencv.hpp>


int main()
{
    cv::Mat I = cv::imread("../data/frame_0151.png");
    cv::resize(I, I, cv::Size(3840, 1080));
    cv::Mat l_image = I.colRange(0, 1920);
    cv::Mat r_image = I.colRange(1920, 3840);


    mmath::CameraProjector cam_proj(1100, 960, 540, 4);
    SceneReconstructor recons(cam_proj);

    recons.reconstruct(l_image, r_image);




    return 0;
}
