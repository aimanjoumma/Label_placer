#include <iostream>
#include <vector>
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/optional.hpp>

namespace bg = boost::geometry;
namespace bt = bg::strategy::transform;

// Define our geometry types
using point_t = bg::model::point<double, 2, bg::cs::cartesian>;
using box_t = bg::model::box<point_t>;

// A structure to hold a point and its successfully placed label
struct labeled_point {
    point_t point;
    std::string label;
    box_t label_box;
};

// Function to check if a candidate label box overlaps with any already placed labels
bool hasOverlap(const box_t& candidate, const std::vector<labeled_point>& placed_labels) {
    for (const auto& placed : placed_labels) {
        if (bg::intersects(candidate, placed.label_box)) {
            return true;
        }
    }
    return false;
}

// The core function to place labels for a set of points
std::vector<labeled_point> placeLabels(const std::vector<std::pair<point_t, std::string>>& input_points) {
    std::vector<labeled_point> result;
    // Define the size of a label (assuming all labels are the same size for simplicity)
    const double label_width = 6.0;
    const double label_height = 2.0;
    // Define candidate positions relative to the point (offsets from the point's center)
    std::vector<std::pair<double, double>> offsets = { {1, 1}, {-1 - label_width, 1}, {1, -1 - label_height}, {-1 - label_width, -1 - label_height} }; // TR, TL, BR, BL

    for (const auto& [pt, label_str] : input_points) {
        boost::optional<labeled_point> successfully_placed;
        // Try each candidate position for the current point
        for (const auto& [offset_x, offset_y] : offsets) {
            point_t corner1(pt), corner2(pt);
            bg::set<0>(corner1, bg::get<0>(pt) + offset_x);
            bg::set<1>(corner1, bg::get<1>(pt) + offset_y);
            bg::set<0>(corner2, bg::get<0>(corner1) + label_width);
            bg::set<1>(corner2, bg::get<1>(corner1) + label_height);

            box_t candidate_box(corner1, corner2);

            if (!hasOverlap(candidate_box, result)) {
                successfully_placed = labeled_point{ pt, label_str, candidate_box };
                break; // Found a valid position, move to the next point
            }
        }
        // If a valid position was found, add it to the result list
        if (successfully_placed) {
            result.push_back(*successfully_placed);
        }
        // If not, the point doesn't get a label for now.
    }
    return result;
}

// A simple function to print the results to the console
void visualize(const std::vector<labeled_point>& placed_labels, double area_size = 20.0) {
    // This is a very simplistic visualization for now.
    std::cout << "Placed " << placed_labels.size() << " labels." << std::endl;
    for (const auto& lp : placed_labels) {
        std::cout << "Point (" << bg::get<0>(lp.point) << ", " << bg::get<1>(lp.point)
            << ") has label '" << lp.label << "' at box corner ("
            << bg::get<0>(lp.label_box.min_corner()) << ", " << bg::get<1>(lp.label_box.min_corner()) << ")." << std::endl;
    }
}

int main() {
    // Create some sample data: points with their labels
    std::vector<std::pair<point_t, std::string>> points = {
        {point_t(1.0, 1.0), "Label A"},
        {point_t(5.0, 5.0), "Label B"},
        {point_t(3.0, 8.0), "Label C"},
        {point_t(8.0, 2.0), "Very Long Label D"},
        {point_t(2.0, 2.0), "Label E"} 
    };

    auto results = placeLabels(points);
    visualize(results);

    return 0;
}