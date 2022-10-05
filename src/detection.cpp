#include <habitat_cv/detection.h>

namespace habitat_cv{
    Detection_list Detection_list::get_detections(const Image &image, unsigned char threshold, int cleaning_cycles) {
        auto clean_image = image.to_gray().threshold(threshold).
                           erode(cleaning_cycles).dilate(cleaning_cycles * 2).erode(cleaning_cycles);
        cv::Mat centroids;
        cv::Mat labels;
        cv::Mat stats;
        connectedComponentsWithStats(clean_image,labels,stats,centroids,4);
        Detection_list detections;
        for (int i = 0; i< stats.rows; i++)
        {
            Detection detection;
            int area = stats.at<int>(i,4);
            detection.area = area;
            auto x = centroids.at<double>(i, 0);
            auto y = centroids.at<double>(i, 1);
            auto point = image.get_point({ (float)x, (float)y});
            detection.location.x = point.x;
            detection.location.y = point.y;
            detections.push_back(detection);
        }
        return detections;
    }

    Detection_list habitat_cv::Detection_list::filter(const Profile &profile) {
        Detection_list filtered;
        for (Detection &detection:*this){
            if (detection.area >= profile.area_lower_bound &&
                detection.area <= profile.area_upper_bound)
                filtered.push_back(detection);
        }
        return filtered;
    }
}