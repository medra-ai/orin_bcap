#include <vector>
#include <queue>
#include <stdexcept>

namespace filter {

// Filters a stream of double-vectors by calculating the mean of the last
// window_size values. The vectors must all have the same length of
// measurement_size.
template <std::size_t N>
class MeanFilter {
    public:
        using Measurement = std::array<double, N>;

        MeanFilter(const size_t window_size) :
            window_size(window_size),
            window_count(0),
            sum_vector(std::array<double, N>{})
        {}

        void AddValue(Measurement new_value) {
            Measurement oldest_value;
            if (window.size() < window_size) {
                oldest_value = {};
                window_count++;
            } else {
                oldest_value = window.front();
                window.pop();
            }

            for (size_t i = 0; i < N; ++i) {
                sum_vector[i] += new_value[i] - oldest_value[i];
            }
            window.push(new_value);
        }
        MeanFilter<N>::Measurement GetMean() {
            Measurement mean_vector;
            for (size_t i = 0; i < N; ++i) {
                mean_vector[i] = sum_vector[i] / window_count;
            }
            return mean_vector;
        }

    private:
        const size_t window_size;
        size_t window_count;  // Number of values in the window.

        // Stores the last window_size values.
        std::queue<Measurement> window;
        // Stores the sum of values in the window.
        Measurement sum_vector;
    };

}  // end namespace filter
