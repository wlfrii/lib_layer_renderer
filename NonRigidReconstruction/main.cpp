#include "src/scene_reconstructor.h"
#include <opencv2/opencv.hpp>


int main()
{
    cv::Mat I = cv::imread("./bg_025.bmp");
//    printf("Read image: [%dx%d]\n", I.rows, I.cols);


    cv::resize(I, I, cv::Size(3840, 1080));
    cv::Mat l_image = I.colRange(0, 1920);
    cv::Mat r_image = I.colRange(1920, 3840);

//    cv::imshow("left", l_image);
//    cv::imshow("right", r_image);
//    cv::waitKey(0);


    mmath::CameraProjector cam_proj(1100, 960, 540, 4);
    SceneReconstructor recons(cam_proj);

    recons.reconstruct(l_image, r_image);
//    recons.plot();



    return 0;
}
