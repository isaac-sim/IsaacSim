// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// This file is included at the bottom of Array.hpp — do not include it directly.

namespace isaacsim
{
namespace common
{
namespace array
{

template <typename T>
T Array::get() const
{
    if (this->device().isCuda())
    {
        return this->toDevice(Device::Cpu()).get<T>();
    }

    // 0-dim scalar
    if constexpr (std::is_arithmetic_v<T>)
    {
        if (this->size() != 1)
        {
            throw std::runtime_error("Array::get(): scalar type requested but array is not scalar");
        }
        return _getItem<T>(static_cast<const std::byte*>(this->data()));
    }
    // 1-dim std::vector<U>
    else if constexpr (std::is_arithmetic_v<typename T::value_type>)
    {
        if (this->ndim() > 1)
        {
            throw std::runtime_error("Array::get(): 1D vector type requested but array has ndim > 1");
        }
        using U = typename T::value_type;
        const std::byte* data = static_cast<const std::byte*>(this->data());
        const size_t elementSize = m_dtype.size();
        size_t count = (this->ndim() == 0) ? 1 : static_cast<size_t>(m_shape[0]);
        T result(count);
        for (size_t i = 0; i < count; ++i)
        {
            result[i] = _getItem<U>(data + i * elementSize);
        }
        return result;
    }
    // 2-dim std::vector<std::vector<V>>
    else if constexpr (std::is_arithmetic_v<typename T::value_type::value_type>)
    {
        if (this->ndim() > 2)
        {
            throw std::runtime_error("Array::get(): 2D vector type requested but array has ndim > 2");
        }
        using U = typename T::value_type;
        using V = typename U::value_type;
        const std::byte* data = static_cast<const std::byte*>(this->data());
        const size_t elementSize = m_dtype.size();
        if (this->ndim() == 0)
        {
            return T(1, U(1, _getItem<V>(data)));
        }
        else if (this->ndim() == 1)
        {
            size_t count = static_cast<size_t>(m_shape[0]);
            U row(count);
            for (size_t i = 0; i < count; ++i)
            {
                row[i] = _getItem<V>(data + i * elementSize);
            }
            return T(1, std::move(row));
        }
        else
        {
            size_t rows = static_cast<size_t>(m_shape[0]);
            size_t cols = static_cast<size_t>(m_shape[1]);
            T result(rows);
            for (size_t i = 0; i < rows; ++i)
            {
                result[i].resize(cols);
                for (size_t j = 0; j < cols; ++j)
                {
                    result[i][j] = _getItem<V>(data + (i * cols + j) * elementSize);
                }
            }
            return result;
        }
    }
    throw std::runtime_error("Array::get(): unsupported type T");
}

template <typename T>
T Array::item() const
{
    if (this->size() != 1)
    {
        throw std::runtime_error("Array::item() called on a non-scalar array");
    }
    if (this->device().isCuda())
    {
        return this->toDevice(Device::Cpu()).item<T>();
    }
    return _getItem<T>(static_cast<const std::byte*>(this->data()));
}

template <typename T>
void Array::_setItem(std::byte* data, const T& value)
{
    switch (m_dtype.kind())
    {
    case Dtype::Kind::eBool:
    {
        bool x = static_cast<bool>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eInt8:
    {
        int8_t x = static_cast<int8_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eInt16:
    {
        int16_t x = static_cast<int16_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eInt32:
    {
        int32_t x = static_cast<int32_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eInt64:
    {
        int64_t x = static_cast<int64_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eUInt8:
    {
        uint8_t x = static_cast<uint8_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eUInt16:
    {
        uint16_t x = static_cast<uint16_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eUInt32:
    {
        uint32_t x = static_cast<uint32_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eUInt64:
    {
        uint64_t x = static_cast<uint64_t>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eFloat32:
    {
        float x = static_cast<float>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    case Dtype::Kind::eFloat64:
    {
        double x = static_cast<double>(value);
        std::memcpy(data, &x, m_dtype.size());
        break;
    }
    default:
        throw std::runtime_error("Array: unknown dtype in _setItem");
    }
}

template <typename T>
T Array::_getItem(const std::byte* data) const
{
    switch (m_dtype.kind())
    {
    case Dtype::Kind::eBool:
    {
        bool value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eInt8:
    {
        int8_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eInt16:
    {
        int16_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eInt32:
    {
        int32_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eInt64:
    {
        int64_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eUInt8:
    {
        uint8_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eUInt16:
    {
        uint16_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eUInt32:
    {
        uint32_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eUInt64:
    {
        uint64_t value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eFloat32:
    {
        float value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    case Dtype::Kind::eFloat64:
    {
        double value;
        std::memcpy(&value, data, sizeof(value));
        return static_cast<T>(value);
    }
    default:
        throw std::runtime_error("Array::_getItem(): unknown dtype");
    }
}

} // namespace array
} // namespace common
} // namespace isaacsim
