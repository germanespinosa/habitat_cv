#include <habitat_cv/image.h>
#include <filesystem>

using namespace std;

namespace habitat_cv{
    Images Images::read(const std::string &path) {
        return read(path, "");
    }

    Images Images::read(const std::string &path, const std::vector<std::string> &file_names) {
        Images images;
        for (auto &f : file_names)
            images.emplace_back(cv::imread(path + "/" + f), f);
        return images;
    }

    Images Images::read(const std::string &path, const std::string &extension) {
        vector<string> file_names;
        for (const auto &entry : std::filesystem::directory_iterator(path)) {
            if (entry.is_directory()) continue;
            string file_name(entry.path());
            if (extension.empty() || file_name.ends_with(extension)) file_names.emplace_back(entry.path());
        }
        return Images::read(path, file_names);
    }

    Images Images::to_rgb() const {
        Images images;
        for (auto &i:*this)
            images.emplace_back(i.to_rgb(), i.file_name);
        return images;
    }

    Images Images::to_gray() const {
        Images images;
        for (auto &i:*this)
            images.emplace_back(i.to_gray(), i.file_name);
        return images;
    }

    Image::Image(cv::Mat m, std::string file_name) : cv::Mat(m), file_name(std::move(file_name)) {
        if(m.channels()==1)
            type = Type::gray;
        else
            type = Type::rgb;
    }

    Image::Image(int rows, int cols, Type type) :
        cv::Mat(rows, cols, type == gray ? CV_8UC1 : CV_8UC3) {
    }

    Image Image::to_rgb() const{
        if (type == rgb) return Image(*this, "");
        cv::Mat rgb;
        cvtColor(*this, rgb, cv::COLOR_GRAY2RGB);
        return Image(rgb, "");
    }

    Binary_image Image::threshold(unsigned char t) const {
        if (type == Type::gray)
            return (*this)>t;

        return this->to_rgb() >t;
    }

    Image Image::to_gray() const {
        if (type == gray) return Image(*this, "");
        cv::Mat gray;
        cv::cvtColor(*this, gray, cv::COLOR_BGR2GRAY);
        return Image(gray, "");
    }

    Binary_image::Binary_image(cv::MatExpr me) : cv::Mat(me) {
    }

    Binary_image::Binary_image(cv::Mat m) : cv::Mat(m){

    }

    Binary_image Binary_image::dilate(unsigned int dilations) {
        cv::Mat dilated;
        cv::dilate(*this, dilated, cv::Mat(), cv::Point(-1, -1), dilations, 1, 2);
        return dilated;
    }

    Binary_image Binary_image::erode(unsigned int erosions) {
        cv::Mat eroded;
        cv::erode(*this, eroded, cv::Mat(), cv::Point(-1, -1), erosions, 1, 1);
        return eroded;
    }
}