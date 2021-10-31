#include <habitat_cv/cleaner.h>

namespace habitat_cv{

    Binary_image Cleaner::clean(const Image &subtracted) {
        auto bw = subtracted.threshold(threshold);
        auto eroded = bw.erode(erosions);
        auto dilated = eroded.dilate(erosions * 2);
        return dilated.erode(erosions);
    }

    Cleaner::Cleaner(unsigned int threshold, unsigned int erosions):
    threshold(threshold),
    erosions(erosions){
    }
}
