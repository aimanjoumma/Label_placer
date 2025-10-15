#include <iostream>
#include <vector>
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

namespace bg = boost::geometry;

using point_t = bg::model::point<double, 2, bg::cs::cartesian>;
using box_t = bg::model::box<point_t>;

struct labeled_point {
    point_t point;
    std::string label;
    box_t label_box;
};

bool hasOverlap(const box_t& candidate, const std::vector<labeled_point>& placed_labels) {
    for (const auto& placed : placed_labels) {
        if (bg::intersects(candidate, placed.label_box)) {
            return true;
        }
    }
    return false;
}

std::vector<labeled_point> placeLabels(const std::vector<std::pair<point_t, std::string>>& input_points) {
    std::vector<labeled_point> result;
    // Reduced label size for better visualization
    const double label_width = 0.4;  // Reduced from 6.0
    const double label_height = 0.2; // Reduced from 2.0

    // Closer offsets - positions relative to point
    std::vector<std::pair<double, double>> offsets = {
        {0.2, 0.2},           // Top-right - much closer
        {-0.2 - label_width, 0.2},  // Top-left  
        {0.2, -0.2 - label_height}, // Bottom-right
        {-0.2 - label_width, -0.2 - label_height} // Bottom-left
    };

    for (size_t i = 0; i < input_points.size(); ++i) {
        const point_t& pt = input_points[i].first;
        const std::string& label_str = input_points[i].second;
        boost::optional<labeled_point> successfully_placed;

        for (size_t j = 0; j < offsets.size(); ++j) {
            double offset_x = offsets[j].first;
            double offset_y = offsets[j].second;

            point_t corner1, corner2;
            // Calculate box corners based on offset from point
            bg::set<0>(corner1, bg::get<0>(pt) + offset_x);
            bg::set<1>(corner1, bg::get<1>(pt) + offset_y);
            bg::set<0>(corner2, bg::get<0>(corner1) + label_width);
            bg::set<1>(corner2, bg::get<1>(corner1) + label_height);

            box_t candidate_box(corner1, corner2);

            if (!hasOverlap(candidate_box, result)) {
                successfully_placed = labeled_point{ pt, label_str, candidate_box };
                break;
            }
        }
        if (successfully_placed) {
            result.push_back(*successfully_placed);
        }
    }
    return result;
}

// Convert from our coordinate system to image coordinates
cv::Point worldToImage(const point_t& world_point, double scale, int image_size) {
    int x = static_cast<int>(bg::get<0>(world_point) * scale);
    int y = image_size - static_cast<int>(bg::get<1>(world_point) * scale); // Flip Y axis
    return cv::Point(x, y);
}

// Get image coordinates for a box
cv::Rect worldBoxToImageRect(const box_t& box, double scale, int image_size) {
    cv::Point top_left = worldToImage(box.min_corner(), scale, image_size);
    cv::Point bottom_right = worldToImage(box.max_corner(), scale, image_size);
    return cv::Rect(top_left, bottom_right);
}

// Visualize results using OpenCV
void visualizeWithOpenCV(const std::vector<labeled_point>& placed_labels,
    const std::vector<std::pair<point_t, std::string>>& all_points) {
    const int IMAGE_SIZE = 600;  // Reduced image size for better density
    const double SCALE = 80.0;   // Increased scale to spread out points more
    const int POINT_RADIUS = 6;

    // Create a white image
    cv::Mat image(IMAGE_SIZE, IMAGE_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

    // First draw all potential points in light gray
    for (const auto& point_pair : all_points) {
        cv::Point img_point = worldToImage(point_pair.first, SCALE, IMAGE_SIZE);
        cv::circle(image, img_point, POINT_RADIUS, cv::Scalar(200, 200, 200), -1);
    }

    // Draw successfully placed labels
    for (const auto& lp : placed_labels) {
        // Draw the point in blue
        cv::Point img_point = worldToImage(lp.point, SCALE, IMAGE_SIZE);
        cv::circle(image, img_point, POINT_RADIUS, cv::Scalar(255, 0, 0), -1);
        cv::circle(image, img_point, POINT_RADIUS, cv::Scalar(0, 0, 0), 1); // Black border

        // Draw the label box in green
        cv::Rect box_rect = worldBoxToImageRect(lp.label_box, SCALE, IMAGE_SIZE);
        cv::rectangle(image, box_rect, cv::Scalar(0, 255, 0), 2);

        // Draw connection line from point to label box center
        cv::Point box_center(box_rect.x + box_rect.width / 2, box_rect.y + box_rect.height / 2);
        cv::line(image, img_point, box_center, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        // Put the label text centered inside the box
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(lp.label, cv::FONT_HERSHEY_SIMPLEX, 0.3, 1, &baseline);
        cv::Point text_org(box_rect.x + (box_rect.width - text_size.width) / 2,
            box_rect.y + (box_rect.height + text_size.height) / 2);

        // Draw semi-transparent background for text
        cv::Mat overlay;
        image.copyTo(overlay);
        cv::rectangle(overlay,
            cv::Point(text_org.x - 2, text_org.y - text_size.height - 2),
            cv::Point(text_org.x + text_size.width + 2, text_org.y + baseline + 2),
            cv::Scalar(255, 255, 255), -1);
        cv::addWeighted(overlay, 0.7, image, 0.3, 0, image);

        // Draw the text
        cv::putText(image, lp.label, text_org,
            cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);
    }

    // Draw unlabeled points in red
    for (const auto& point_pair : all_points) {
        bool has_label = false;
        for (const auto& lp : placed_labels) {
            if (bg::equals(lp.point, point_pair.first)) {
                has_label = true;
                break;
            }
        }
        if (!has_label) {
            cv::Point img_point = worldToImage(point_pair.first, SCALE, IMAGE_SIZE);
            cv::circle(image, img_point, POINT_RADIUS, cv::Scalar(0, 0, 255), -1);
            cv::circle(image, img_point, POINT_RADIUS, cv::Scalar(0, 0, 0), 1); // Black border
        }
    }

    // Add legend
    cv::putText(image, "Blue: Labeled points", cv::Point(20, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    cv::putText(image, "Red: Unlabeled points (overlap)", cv::Point(20, 55),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    cv::putText(image, "Green: Label boxes", cv::Point(20, 80),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    cv::putText(image, "Red lines: Point-label connections", cv::Point(20, 105),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

    // Add title
    cv::putText(image, "Automatic Label Placement Algorithm", cv::Point(IMAGE_SIZE / 2 - 180, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // Display and save the image
    cv::imshow("Automatic Label Placement Results", image);
    cv::imwrite("label_placement_results.png", image);

    std::cout << "Image saved as 'label_placement_results.png'\n";
    std::cout << "Placed " << placed_labels.size() << " out of " << all_points.size() << " labels.\n";
    std::cout << "Press any key to close the window...\n";
    cv::waitKey(0);
}

int main() {
    // Create sample data with more realistic distribution
    std::vector<std::pair<point_t, std::string>> points;

    // Cluster 1
    points.push_back(std::make_pair(point_t(1.0, 1.0), "A"));
    points.push_back(std::make_pair(point_t(1.5, 1.2), "B"));
    points.push_back(std::make_pair(point_t(2.0, 0.8), "C"));

    // Cluster 2  
    points.push_back(std::make_pair(point_t(4.0, 3.0), "D"));
    points.push_back(std::make_pair(point_t(4.5, 3.5), "E"));
    points.push_back(std::make_pair(point_t(3.8, 3.8), "F"));

    // Cluster 3
    points.push_back(std::make_pair(point_t(2.0, 4.0), "G"));
    points.push_back(std::make_pair(point_t(2.5, 4.5), "H"));

    // Isolated points
    points.push_back(std::make_pair(point_t(5.5, 1.0), "I"));
    points.push_back(std::make_pair(point_t(1.0, 5.0), "J"));

    auto results = placeLabels(points);

    // Console output with more details
    std::cout << "\n=== LABEL PLACEMENT RESULTS ===\n";
    for (const auto& lp : results) {
        std::cout << "✓ Point (" << bg::get<0>(lp.point) << ", " << bg::get<1>(lp.point)
            << ") -> '" << lp.label << "' at ["
            << bg::get<0>(lp.label_box.min_corner()) << "," << bg::get<1>(lp.label_box.min_corner())
            << "]-[" << bg::get<0>(lp.label_box.max_corner()) << "," << bg::get<1>(lp.label_box.max_corner()) << "]\n";
    }

    // Find and report unlabeled points - FIXED: use 'points' and 'results' instead of undefined variables
    for (const auto& point_pair : points) {
        bool has_label = false;
        for (const auto& lp : results) {
            if (bg::equals(lp.point, point_pair.first)) {
                has_label = true;
                break;
            }
        }
        if (!has_label) {
            std::cout << "✗ Point (" << bg::get<0>(point_pair.first) << ", " << bg::get<1>(point_pair.first)
                << ") -> NO LABEL for '" << point_pair.second << "' (overlap)\n";
        }
    }

    // OpenCV visualization - FIXED: pass the correct variables
    visualizeWithOpenCV(results, points);

    return 0;
}