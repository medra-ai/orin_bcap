#include "filter.hpp"

namespace filter {

MeanFilter::MeanFilter(const size_t window_size, const size_t measurement_size) :
    window_size(window_size),
    measurement_size(measurement_size),
    window_count(0),
    sum_vector(std::vector<double>(measurement_size, 0.0))
{}

void MeanFilter::AddValue(std::vector<double> new_value) {
    if (new_value.size() != measurement_size) {
        throw std::invalid_argument("Input vector size does not match measurement size.");
    }

    std::vector<double> oldest_value;
    if (window.size() < window_size) {
        oldest_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        window_count++;
    } else {
        oldest_value = window.front();
        window.pop();
    }

    for (size_t i = 0; i < measurement_size; i++) {
        sum_vector[i] += new_value[i] - oldest_value[i];
    }
    window.push(new_value);
}

std::vector<double> MeanFilter::GetMean() {
    std::vector<double> mean_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (size_t i = 0; i < measurement_size; i++) {
        mean_vector[i] = sum_vector[i] / window_count;
    }
    return mean_vector;
}

}  // end namespace filter