#include <habitat_cv/image.h>
#include <filesystem>
#include <utility>

using namespace std;

namespace habitat_cv{
    Images Images::read(const std::string &path) {
        return read(path, "");
    }

    Images Images::read(const std::string &path, const std::vector<std::string> &file_names) {
        Images images;
        for (auto &f : file_names)
            images.emplace_back(Image::read(path, f));
        return images;
    }

    Images Images::read(const std::string &path, const std::string &extension) {
        vector<string> file_names;
        for (const auto &entry : std::filesystem::directory_iterator(path)) {
            if (entry.is_directory()) continue;
            string file_name(entry.path().filename());
            if (extension.empty() || file_name.ends_with(extension)) file_names.emplace_back(file_name);
        }
        return Images::read(path, file_names);
    }

    void Images::save(const string &path, const vector<std::string> &file_names) {
        assert(file_names.size()==size());
        for (unsigned int i=0; i<size(); i++)
            (*this)[i].save(path, file_names[i]);
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

    void Images::save(const string &path) {
        for (auto &i:*this) i.save(path);
    }

    Image &Images::get(const string &file_name) {
        for (auto &i:*this)
            if (i.file_name==file_name) return i;
        throw;
    }

    Images Images::clone() {
        Images new_images;
        for (auto &i:*this) new_images.emplace_back(i.clone(), i.file_name);
        return new_images;
    }

    Image::Image(cv::Mat m, std::string file_name) : cv::Mat(m), file_name(std::move(file_name)) {
        if(m.channels()==1)
            type = Type::gray;
        else
            type = Type::rgb;
    }

    Image::Image(int rows, int cols, Type type) :
        cv::Mat(rows, cols, type == gray ? CV_8UC1 : CV_8UC3), type(type) {
    }

    Image Image::to_rgb() const{
        if (type == rgb) return Image(*this, "");
        cv::Mat rgb;
        cvtColor(*this, rgb, cv::COLOR_GRAY2RGB);
        return {rgb, ""};
    }

    Binary_image Image::threshold(unsigned char t) const {
        if (type == Type::gray)
            return Binary_image((*this) > t);

        return Binary_image(this->to_gray() > t);
    }

    Image Image::to_gray() const {
        if (type == gray) return Image(*this, "");
        cv::Mat gray;
        cv::cvtColor(*this, gray, cv::COLOR_BGR2GRAY);
        return {gray, ""};
    }

    void Image::save(const std::string &path) {
        cv::imwrite(path + "/" + file_name,*this);
    }

    void Image::save(const string &path, const string &f){
        file_name = f;
        cv::imwrite(path + "/" + f,*this);
    }

    Image Image::read(const string &path, const string &f) {
        return {cv::imread(path + "/" + f, cv::IMREAD_UNCHANGED), f};
    }

    void Image::clear() {
        setTo(0);
    }

    cv::Point2f Image::get_point(const cell_world::Location &l) const{
        return {(float)l.x, (float)(size().height - l.y)};
    }

    void Image::line(const cell_world::Location &src, const cell_world::Location &dst, const cv::Scalar &color) {
        int thickness = 2;
        int lineType = cv::LINE_8;
        cv::line( *this, get_point(src), get_point(dst), color, thickness, lineType);
    }

    void Image::polygon(const cell_world::Polygon &polygon, const cv::Scalar &color, bool filled) {
        vector<vector<cv::Point>> points;
        auto &vertices = points.emplace_back();
        for (auto &v: polygon.vertices) vertices.push_back(get_point(v));
        if (filled)
            cv::fillPoly(*this, points, color);
        else
            cv::polylines(*this, points, true, color);
    }

    void Image::circle(const cell_world::Location &center, double radius, const cv::Scalar &color, bool filled) {
        if (filled)
            cv::circle(*this, get_point(center), radius, color, -1);
        else
            cv::circle(*this, get_point(center), radius, color);
    }

    void Image::line(const cell_world::Location &src, double theta, double dist, const cv::Scalar &color) {
        line(src, src.move(theta, dist), color);
    }

    void Image::arrow(const cell_world::Location &src, double theta, double dist, const cv::Scalar &color) {
        arrow(src, src.move(theta, dist), color);
    }

    void Image::arrow(const cell_world::Location &src, const cell_world::Location &dst, const cv::Scalar &color) {
        int thickness = 2;
        int lineType = cv::LINE_8;
        cv::arrowedLine( *this, get_point(src), get_point(dst), color, thickness, lineType, 0, .2);
    }

    Image Image::diff(const Image &image) const{
        assert(image.size == size);
        assert(type == gray);
        assert(image.type == gray);
        cv::Mat subtracted;
        cv::absdiff(image, *this, subtracted);
        return {subtracted, ""};
    }

    Image Image::mask(const Binary_image &mask) {
        Image new_image;
        new_image.type = type;
        cv::bitwise_and(*this,mask,new_image);
        return new_image;
    }

    void Image::text(const cell_world::Location &l, const std::string &t, const cv::Scalar &color, float size, int halign, int valign) {

        auto point = get_point(l);

        if (valign || halign) {
            int baseline = 0;
            auto s = cv::getTextSize(t, cv::FONT_HERSHEY_DUPLEX, size, 1, &baseline);

            if (valign) {
                if (valign == 2) {
                    point.y += s.height;
                } else {
                    point.y += s.height / 2;
                }
            }
            if (halign) {
                if (halign == 2) {
                    point.x -= s.width;
                } else {
                    point.x -= s.width / 2;
                }
            }
        }
        cv::putText(*this,
                    t.c_str(),
                    point, // Coordinates
                    cv::FONT_HERSHEY_DUPLEX, // Font
                    size, // Scale. 2.0 = 2x bigger
                    color, // BGR Color
                    1 // Line Thickness (Optional)
        );
    }

    Image::Image(cv::Size size, Image::Type type) : Image(size.height, size.width, type){

    }

    cell_world::Location Image::get_location(const cv::Point2f &p) const {
        return {(float)p.x, (float)(size().height - p.y)};
    }

    Binary_image::Binary_image(cv::MatExpr me) : cv::Mat(me) {
    }

    Binary_image::Binary_image(cv::Mat m) : cv::Mat(m){

    }

    Binary_image Binary_image::dilate(unsigned int dilations) {
        cv::Mat dilated;
        cv::dilate(*this, dilated, cv::Mat(), cv::Point(-1, -1), dilations, 1, 2);
        return Binary_image(dilated);
    }

    Binary_image Binary_image::erode(unsigned int erosions) {
        cv::Mat eroded;
        cv::erode(*this, eroded, cv::Mat(), cv::Point(-1, -1), erosions, 1, 1);
        return Binary_image(eroded);
    }
}