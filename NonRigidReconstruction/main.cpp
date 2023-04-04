#include "src/scene_reconstructor.h"
#include "src/util.h"
#include <opencv2/opencv.hpp>

#include <ceres/cubic_interpolation.h>

int main(int argc, char* argv[])
{
//    float data[] = {1, 1, 1, 1,
//                    2, 2, 2, 2,
//                    3, 3, 3, 4};
//    cv::Mat mat(3, 4, CV_32FC1, data);
//    ceres::Grid2D<float, 1> array(mat.ptr<float>(0), 0, 3, 0, 4);
//    ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>> interpolator(array);
//    double r = 1.5, c = 1.5;
//    double f;
//    interpolator.Evaluate(r, c, &f);
//    printf("r:%f, c:%f, f:%f\n", r, c, f);
//    r = 0.5, c = 0.5;
//    interpolator.Evaluate(r, c, &f);
//    printf("r:%f, c:%f, f:%f\n", r, c, f);
//    r = 1.5, c = 2.5;
//    interpolator.Evaluate(r, c, &f);
//    printf("r:%f, c:%f, f:%f\n", r, c, f);

//    pcl::PointCloud<pcl::PointXYZ> coords;
//    coords.push_back({1,2,3});
//    coords.push_back({2,2,3});
//    coords.push_back({3,2,3});
//    coords.push_back({4,2,3});
//    coords.push_back({5,2,3});
//    pcl::PointCloud<pcl::PointXYZ> coords2;
//    coords.push_back({1,2,5});
//    coords.push_back({2,2,5});
//    coords.push_back({3,2,5});
//    coords.push_back({4,2,5});
//    coords.push_back({5,2,5});

//    pcl::PointCloud<pcl::PointXYZ> coords3;
//    coords3 = coords + coords2;
//    for(size_t i = 0; i < coords3.size(); i++) {
//        printf("%.3f,%.3f,%.3f\n",
//               coords3.at(i).x, coords3.at(i).y, coords3.at(i).z);
//    }

//    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> correspondences;
//    correspondences.push_back({{1,2,3},{11,12,13}});
//    correspondences.push_back({{2,3,3},{11,13.2,12.8}});
//    correspondences.push_back({{11,2,3},{21.1,12.5,13.3}});
//    correspondences.push_back({{13,21,3},{22.9,31.1,13.2}});
//    Eigen::Matrix4f T;
//    util::estimateTransform(correspondences, T);
//    std::cout << "T: \n" << T << "\n";
//    for(size_t i = 0; i < correspondences.size(); i++) {
//        Eigen::Vector3f pt = correspondences[i].first.getVector3fMap();
//        std::cout << (T.topLeftCorner(3, 3) * pt + T.topRightCorner(3, 1)).transpose() << "\n";
//    }
//    return 0;

    if(argc < 2) {
        printf("ARGUMENTS.\n");
        return -1;
    }


    mmath::CameraProjector cam_proj(1100, 960, 540, 4);
    SceneReconstructor recons(cam_proj);

    if(std::stoi(argv[1]) == 0){
        for (int i = 25; i < 26; i++) {
            char impath[64];
            sprintf(impath, "bg_%03d.bmp", i);

            printf("%s\n", impath);
            std::string path = "../data/" + std::string(impath);


            cv::Mat I = cv::imread(path);

            cv::resize(I, I, cv::Size(3840, 1080));
            cv::Mat l_image = I.colRange(0, 1920);
            cv::Mat r_image = I.colRange(1920, 3840);


            recons.reconstruct(l_image, r_image);
            recons.plot();
        }
    }
    else{
        for (int i = 200; i < 330; i++) {
            char imname[64];
            sprintf(imname, "frame_%04d.png", i);

            printf("%s\n", imname);
            std::string path = "../data/sequence_01/" + std::string(imname);

            cv::Mat I = cv::imread(path);

            cv::resize(I, I, cv::Size(3840, 1080));
            cv::Mat l_image = I.colRange(0, 1920);
            cv::Mat r_image = I.colRange(1920, 3840);


            recons.reconstruct(l_image, r_image);
#if 1
            cv::Mat res = recons.getPlotResult();
            cv::imshow("View", res);
            cv::waitKey(10);

            cv::imwrite("./result/"+std::string(imname), util::im2uchar(res));
#else
            recons.plot();
#endif
        }
    }


    return 0;
}
