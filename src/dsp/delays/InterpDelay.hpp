/**
 * @file InterpDelay2.hpp
 * @author Dale Johnson, Valley Audio Soft
 * @brief A more optimised version of the linear interpolating delay.
 */

#pragma once
#include <vector>
#include <cstdint>
// #include <iostream>
#include <cassert>
#include <algorithm>
#include "../../utilities/Utilities.hpp"

template<typename T = float, typename IntT = int32_t>
class InterpDelay {
public:
    T input = T(0);
    T output = T(0);

    InterpDelay(uint32_t maxLength = 512, IntT initDelayTime = 0) {
        assert(maxLength != 0);
        l = maxLength;
        buffer = std::vector<T>(l, T(0));
        setDelayTime(initDelayTime);
    }

    void process() {
        assert(w >= 0);
        assert(w < l);
        buffer[w] = input;
        IntT r = w - t;
        
        if (r < 0) {
            r += l;
        }

        ++w;
        if (w == l) {
            w = 0;
        }

        IntT upperR = r - 1;
        if (upperR < 0) {
            upperR += l;
        }

        assert(r >= 0);
        assert(r < l);
        assert(upperR >= 0);
        assert(upperR < l);
        output = linterp(buffer[r], buffer[upperR], f);
    }

    T tap(IntT i) const {

        assert(i < l);
        assert(i >= 0);

        IntT j = w - i;
        if (j < 0) {
            j += buffer.size();
        }
        return buffer[j];
    }

    void setDelayTime(T newDelayTime) {
		t = std::clamp<IntT>(newDelayTime, 0, l-1);
        f = newDelayTime - static_cast<T>(t);
    }

    void clear() {
        std::fill(buffer.begin(), buffer.end(), T(0));
        input = T(0);
        output = T(0);
    }

private:
    std::vector<T> buffer;
    IntT w = 0;
    IntT t = 0;
    T f = T(0);
    IntT l = 512;
};

