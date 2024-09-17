#include <vector>
#include <queue>
#include <stdexcept>

namespace filter {

// Filters a stream of double-vectors by calculating the mean of the last
// window_size values. The vectors must all have the same length of
// measurement_size.
class MeanFilter {
    public:
        MeanFilter(const size_t window_size, const size_t measurement_size);
        void AddValue(std::vector<double> new_value);
        std::vector<double> GetMean();

    private:
        const size_t window_size;
        const size_t measurement_size;
        size_t window_count;  // Number of values in the window.

        // Stores the sum of values in the window.
        std::queue<std::vector<double>> window;
        std::vector<double> sum_vector;
    };

}  // end namespace filter
