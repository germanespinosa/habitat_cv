#include <detection.h>

namespace habitat_cv{
    Detection_location_list Detection::get_detections(cv::Mat &clean_image, Profile_list &profiles) {
        cv::Mat centroids;
        cv::Mat labels;
        cv::Mat stats;
        connectedComponentsWithStats(clean_image,labels,stats,centroids,4);
        Detection_location_list detections;
        for (int i = 0; i< stats.rows; i++)
        {
            int area = stats.at<int>(i,4);
            auto matching_profiles = profiles.match(area);
            if (!matching_profiles.empty()) {
                Detection_location detection;
                detection.area = area;
                detection.location.x = centroids.at<double>(i, 0);
                detection.location.y = centroids.at<double>(i, 1);
                for (auto &p : matching_profiles){
                    detection.profile = p;
                    detections.push_back(detection);
                }
            }
        }
        return detections;
    }
}