#include <habitat_cv/detection.h>
#include <performance.h>

namespace habitat_cv{
    Detection_list Detection_list::get_detections(const Binary_image &clean_image) {
        PERF_START("CCC");
        cv::Mat centroids;
        cv::Mat labels;
        cv::Mat stats;
        Image small_clean;
        connectedComponentsWithStats(clean_image, labels, stats, centroids,4);
        PERF_STOP("CCC");
        PERF_START("DETECT");
        Detection_list detections;
        for (int i = 0; i< stats.rows; i++)
        {
            Detection detection;
            int area = stats.at<int>(i,4);
            detection.area = area;
            auto x = centroids.at<double>(i, 0);
            auto y = centroids.at<double>(i, 1);
            auto point = clean_image.get_point({ (float)x, (float)y});
            detection.location.x = point.x;
            detection.location.y = point.y;
            detections.push_back(detection);
        }
        PERF_STOP("DETECT");
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

    Detection_list &Detection_list::scale(float s) {
        for (auto &d: *this){
            d.location = d.location * s;
            d.area = d.area * s * s;
        }
        return *this;
    }
}