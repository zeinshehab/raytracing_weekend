#ifndef BVH_H
#define BVH_H

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

#include "hitable.h"
#include "aabb.h"

inline bool box_compare(const hitable *a, const hitable *b, int axis) {
    aabb box_left, box_right;
    if (!a->bounding_box(0, 0, box_left) || !b->bounding_box(0, 0, box_right)) {
        throw std::runtime_error("No bounding box in BVH node constructor.");
    }
    return box_left.min()[axis] < box_right.min()[axis];
}

class bvh_node : public hitable {
    public:
        bvh_node() {}
        bvh_node(hitable **l, int n, float time0, float time1);

        virtual bool hit(const ray& r, float t_min, float t_max, hit_record& rec) const;
        virtual bool bounding_box(float t0, float t1, aabb& box) const;

        hitable *left;
        hitable *right;
        aabb box;
};

bvh_node::bvh_node(hitable **l, int n, float time0, float time1) {
    int axis = int(3 * drand48());
    auto comparator = [axis](hitable *a, hitable *b) {
        return box_compare(a, b, axis);
    };

    std::sort(l, l + n, comparator);

    if (n == 1) {
        left = right = l[0];
    } else if (n == 2) {
        left = l[0];
        right = l[1];
    } else {
        left = new bvh_node(l, n/2, time0, time1);
        right = new bvh_node(l + n/2, n - n/2, time0, time1);
    }

    aabb box_left, box_right;
    if (!left->bounding_box(time0, time1, box_left) ||
        !right->bounding_box(time0, time1, box_right)) {
        throw std::runtime_error("No bounding box in BVH node constructor.");
    }
    box = surrounding_box(box_left, box_right);
}

bool bvh_node::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
    if (!box.hit(r, t_min, t_max)) return false;

    bool hit_left = left->hit(r, t_min, t_max, rec);
    bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);
    return hit_left || hit_right;
}

bool bvh_node::bounding_box(float t0, float t1, aabb& b) const {
    b = box;
    return true;
}

#endif // BVH_H
