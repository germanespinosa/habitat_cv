#include <habitat_cv/layout.h>


namespace habitat_cv {

    Content::Content(cv::Size size, Image::Type type): Image(size.height,size.width,type){
    }


    Content_resize::Content_resize(cv::Size size, Image::Type type) : Content(size, type) {
    }

    Content_crop::Content_crop(const cell_world::Location &bottom_left, const cell_world::Location &top_right, Image::Type type) :
            bottom_left(bottom_left),
            top_right(top_right),
            Content ({(int) (top_right.x - bottom_left.x), (int) (top_right.y - bottom_left.y)}, type){

    }

    Layout::Layout(int rows, int cols, Image::Type type) : Image(rows, cols, type) {

    }

    void Layout::add_place_holder(Content &ph, const cell_world::Location &l) {
        assert (ph.type == type);
        place_holders.emplace_back(ph, l);
    }

    Image Layout::get_image() {
        int i = 0;
        clear();
        for (auto &place_holder: place_holders){
            auto point = get_point(place_holder.location);
            auto size = place_holder.content.size();
            point.y -= size.height;
            cv::Rect rect = {point , size};
            place_holder.content.copyTo((*this)(rect));
        }
        return {clone(), ""};

    }

    Layout::Place_holder::Place_holder(Content &content, cell_world::Location location) :
            content(content),
            location(std::move(location)){
    }

    void habitat_cv::Content_text::operator=(const std::string &text) {
        this->setTo(bgcolor);
        this->text(location, text, color, font_size, halign, valign);
    }

    habitat_cv::Content_text::Content_text(cv::Size size,
                                           Image::Type type,
                                           const cv::Scalar &color,
                                           const cv::Scalar &bgcolor,
                                           float font_size,
                                           int halign,
                                           int valign) :
            Content(size, type),
            color(color),
            bgcolor(bgcolor),
            font_size(font_size),
            halign(halign),
            valign(valign){
        location = {0,0};
        if (halign == 1) location.x = size.width / 2;
        if (halign == 2) location.x = size.width;
        if (valign == 1) location.y = size.height / 2;
        if (valign == 2) location.y = size.height;
    }

    void habitat_cv::Content_resize::operator=(const Image &image) {
        cv::resize(image,*this, this->size());
    }

    void habitat_cv::Content_crop::operator=(const Image &image) {
        cv::Rect rect(image.get_point(bottom_left), image.get_point(top_right));
        image(rect).copyTo(*this);
    }
}