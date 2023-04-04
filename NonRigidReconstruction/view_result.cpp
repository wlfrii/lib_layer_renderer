#include <opencv2/opencv.hpp>


cv::VideoWriter vid_writer;


int main(int argc, char* argv[])
{
    if(argc < 4) {
        printf("Run the app as:\n");
        printf("  $ viewresult [start_image_index] [stop_image_index] "
               "[waittime] [whether write to video]]\n");
        return -1;
    }
    bool write_to_video = false;

    int start_idx = std::stoi(argv[1]);
    int stop_idx = std::stoi(argv[2]);
    int timegap = std::stoi(argv[3]);

    if(argc == 5){
        write_to_video = std::stoi(argv[4]);

        char vidname[64];
        sprintf(vidname, "result_%d_%d_%d.avi", start_idx, stop_idx, timegap);
        vid_writer.open(vidname, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                        (int)(1000.f/timegap), cv::Size(1920, 1080), true);
    }

    while(true) {
        for(int i = start_idx; i <= stop_idx; i++) {
            char imname[64];
            sprintf(imname, "frame_%04d.png", i);

            cv::Mat I = cv::imread("./result/" + std::string(imname));
            cv::imshow("Result", I);
            char key = cv::waitKey(timegap);
            if(key == 'q') {
                return 0;
            }

            if(write_to_video) {
                vid_writer.write(I);
            }
        }
        cv::waitKey(2000);

        if(write_to_video) break;
    }

    if(write_to_video) vid_writer.release();

    return 0;
}
