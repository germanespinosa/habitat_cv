#pragma once
#include <habitat_cv/core.h>
#include <habitat_cv/image.h>

namespace habitat_cv{
    struct Cleaner {
        Cleaner(unsigned int threshold = 50, unsigned int erosions = 2);
        Binary_image clean (const Image &subtracted);
        unsigned int threshold;
        unsigned int erosions;
    };
}