#ifndef AABB_H
#define AABB_H

#include <algorithm>
#include <cmath>

#include "ray.h"

class aabb {
    public:
        aabb() {}
        aabb(const vec3& a, const vec3& b) : _min(a), _max(b) {}

        vec3 min() const { return _min; }
        vec3 max() const { return _max; }

        bool hit(const ray& r, float t_min, float t_max) const {
            for (int a = 0; a < 3; a++) {
                float invD = 1.0f / r.direction()[a];
                float t0 = (min()[a] - r.origin()[a]) * invD;
                float t1 = (max()[a] - r.origin()[a]) * invD;
                if (invD < 0.0f) std::swap(t0, t1);
                t_min = t0 > t_min ? t0 : t_min;
                t_max = t1 < t_max ? t1 : t_max;
                if (t_max <= t_min) return false;
            }
            return true;
        }

    private:
        vec3 _min;
        vec3 _max;
};

inline aabb surrounding_box(const aabb& box0, const aabb& box1) {
    vec3 small(std::fmin(box0.min().x(), box1.min().x()),
               std::fmin(box0.min().y(), box1.min().y()),
               std::fmin(box0.min().z(), box1.min().z()));
    vec3 big(std::fmax(box0.max().x(), box1.max().x()),
             std::fmax(box0.max().y(), box1.max().y()),
             std::fmax(box0.max().z(), box1.max().z()));
    return aabb(small, big);
}

#endif // AABB_H
