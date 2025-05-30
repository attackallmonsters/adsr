#pragma once

template <typename T>
// Clamp min/max function
constexpr T clamp(const T &val, const T &minVal, const T &maxVal)
{
    return (val < minVal) ? minVal : (val > maxVal ? maxVal : val);
}

// Clamp min function
template <typename T>
constexpr T clampmin(const T &val, const T &minVal)
{
    return (val < minVal) ? minVal : val;
}