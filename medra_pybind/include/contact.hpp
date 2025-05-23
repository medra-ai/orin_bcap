#include <vector>
#include <deque>
#include <stdexcept>

// Python bindings
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
using namespace pybind11::literals;
namespace py = pybind11;

namespace contact {

// Filters a stream of double-vectors by calculating the mean of the last
// window_size values. The vectors must all have the same length of
// measurement_size.
template <std::size_t N>
class ContactFilter {
    public:
        using Measurement = std::array<double, N>;

        ContactFilter(const size_t window_size) :
            window_size(window_size),
            window_count(0),
            guard()
        {}

        void AddValue(double new_timestamp, Measurement new_value) {
            if (data.size() < window_size) {
                window_count++;
            } else {
                timestamps.pop_front();
                data.pop_front();
            }
            timestamps.push_back(new_timestamp);
            data.push_back(new_value);
        }

        ContactFilter<N>::Measurement GetContact() {
            // copy the entire data and timestamps to std::vector
            std::vector<double> data_vector(window_size * N);
            std::vector<double> timestamps_vector(window_size);
            size_t idx = 0;
            for (const auto& measurement : data) {
                for (size_t i = 0; i < N; ++i) {
                    data_vector[idx++] = measurement[i];
                }
            }
            std::copy(timestamps.begin(), timestamps.end(), timestamps_vector.begin());
            py::array_t<double> data_array = py::array_t<double>(window_size * N, data_vector.data());
            py::array_t<double> timestamps_array = py::array_t<double>(window_size, timestamps_vector.data());

            // Function to process force-torque data for contact detection
            auto lastest_fts = py::array_t<double>(6);
            auto locals = py::dict("timestamps_arr"_a=timestamps_array, "fts_arr"_a=data_array, "lastest_fts"_a=lastest_fts);
            py::exec(R"(
import numpy as np
import scipy.signal as signal
                
# Ensure numpy arrays
t = np.asarray(timestamps_arr)
v = np.asarray(fts_arr).reshape(6, -1).T

# Estimate sampling frequency
dt = np.median(np.diff(t))
fs = 1.0 / dt

hp_cutoff = 0.01    # Hz, for drift removal
lp_cutoff = 5.0     # Hz, for envelope smoothing

# High pass filter to remove low frequency drift
b_hp, a_hp = signal.butter(2, hp_cutoff, 'highpass')
filtered_v = []
for x in v:
    filtered_v.append(signal.filtfilt(b_hp, a_hp, x))
v = np.array(filtered_v)

# Low pass filter to remove high frequency noise
# Normalize cutoff frequency by Nyquist frequency (fs/2)
cutoff_normalized = lp_cutoff / (fs / 2.0)
b_lp, a_lp = signal.butter(2, cutoff_normalized, 'lowpass')
v = signal.filtfilt(b_lp, a_lp, v)

result = v[:, -1]
lastest_fts[:] = result
                )", py::globals(), locals);

            Measurement filtered_data;
            auto buf = lastest_fts.unchecked<1>();
            for (ssize_t i = 0; i < buf.shape(0); ++i) {
                filtered_data[i] = buf(i);
            }

            return filtered_data;
        }

    private:
        const size_t window_size;
        size_t window_count;  // Number of values in the window.
        py::scoped_interpreter guard;

        // Stores the last window_size values.
        std::deque<double> timestamps;
        std::deque<Measurement> data;
    };

}  // end namespace filter
