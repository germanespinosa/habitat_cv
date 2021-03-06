#include <habitat_cv/util.h>
#include <sys/stat.h>

using namespace std;

namespace habitat_cv {
    cv::Mat to_gray(const cv::Mat &image) {
        cv::Mat grey;
        cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
        return grey;
    }

    bool file_exists(const string &file_path){
        struct stat buffer;
        return  (stat (file_path.c_str(), &buffer) == 0);
    }

    std::vector<cv::Mat> read_images(const std::string &path, const std::vector<std::string> &file_paths){
        std::vector<cv::Mat> images;
        for (auto &f : file_paths)
            images.push_back(to_gray(cv::imread(path + "/" + f)));
        return images;
    }

    void write_images(const std::vector<cv::Mat> &images, const std::string &path, const std::vector<std::string> &file_paths){
        int i=0;
        for (auto &f : file_paths)
            cv::imwrite(path + "/" + f, images[i++]);

    }
}
