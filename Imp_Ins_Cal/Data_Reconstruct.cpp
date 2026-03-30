#include "data_reconstruct.h"
#include <iostream>
#include <numeric>   // for accumulate
#include <algorithm> // for sort, min
#include <cmath>     // for sqrt
#include <stdexcept> // for invalid_argument
#include <vector>    // for vector and iota

using namespace std;

void Reconstruct(const double* data_original, size_t length, double* SD_value, double* mean_value) {
    const size_t n = 20; // 滑动窗口大小设定为20

    // 检查输入
    if (length == 0) {
        throw invalid_argument("data length must be non-zero.");
    }

    vector<double> reconstructed_data(length);

    // 滑动平均重构
    for (size_t i = 0; i < length; ++i) {
        size_t start_idx, end_idx;

        if (i < n) {
            start_idx = 0;
            end_idx = i + n;
        } else if (i >= length - n) {
            start_idx = i - n + 1;
            end_idx = length;
        } else {
            start_idx = i - (n / 2);
            end_idx = i + (n / 2);
        }

        // 计算窗口内的平均值
        double sum = accumulate(data_original + start_idx, data_original + end_idx, 0.0);
        reconstructed_data[i] = sum / static_cast<double>(end_idx - start_idx);
    }

    // 选取滤波后的均值
    double original_clean_mean = accumulate(reconstructed_data.begin(), reconstructed_data.end(), 0.0) / static_cast<double>(length);
    vector<double> original_abs_diff(length);

    for (size_t i = 0; i < length; ++i) {
        original_abs_diff[i] = abs(reconstructed_data[i] - original_clean_mean);
    }

    // 获取绝对差值的索引
    vector<int> sorted_indices_data(length);
    iota(sorted_indices_data.begin(), sorted_indices_data.end(), 0);
    sort(sorted_indices_data.begin(), sorted_indices_data.end(),
         [&original_abs_diff](int a, int b) {
             return original_abs_diff[a] < original_abs_diff[b];
         });

    // 选择前120个索引
    size_t select_count = min(static_cast<size_t>(120), length);
    vector<int> selected_indices_data(sorted_indices_data.begin(), sorted_indices_data.begin() + select_count);

    // 计算选择的均值
    *mean_value = 0.0;
    for (const int idx : selected_indices_data) {
        *mean_value += reconstructed_data[idx];
    }
    *mean_value /= static_cast<double>(select_count);

    // 计算选择的标准差
    *SD_value = 0.0;
    for (const int idx : selected_indices_data) {
        *SD_value += (reconstructed_data[idx] - *mean_value) * (reconstructed_data[idx] - *mean_value);
    }
    // *SD_value = sqrt(*SD_value / static_cast<double>(select_count));
    *SD_value = 100 * sqrt(*SD_value / static_cast<double>(select_count)); // 将标准差结果放大100倍
}