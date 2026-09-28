#ifndef CLOTH_INTERNAL_H
#define CLOTH_INTERNAL_H

// 布料/头发共用的内部辅助: 空间哈希(平坦数组) 与 几何工具
// 注意: 仅限扩展内部使用, 不暴露给 GDScript。

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace cloth_internal {

using godot::Vector3;
using godot::PackedVector3Array;
using godot::CLAMP;

// 27 邻域偏移(必须含中心格 (0,0,0) —— 同格顶点对是折叠/贴合的常态)
inline const int (&offs27())[27][3] {
	static const int k[27][3] = {
		{ 0, 0, 0 }, { -1, -1, -1 }, { -1, -1, 0 }, { -1, -1, 1 }, { -1, 0, -1 }, { -1, 0, 0 }, { -1, 0, 1 },
		{ -1, 1, -1 }, { -1, 1, 0 }, { -1, 1, 1 }, { 0, -1, -1 }, { 0, -1, 0 }, { 0, -1, 1 }, { 0, 0, -1 },
		{ 0, 0, 1 }, { 0, 1, -1 }, { 0, 1, 0 }, { 0, 1, 1 }, { 1, -1, -1 }, { 1, -1, 0 }, { 1, -1, 1 },
		{ 1, 0, -1 }, { 1, 0, 0 }, { 1, 0, 1 }, { 1, 1, -1 }, { 1, 1, 0 }, { 1, 1, 1 }
	};
	return k;
}

inline int64_t pack_cell(int x, int y, int z) {
	return ((int64_t)x << 42) | ((int64_t)y << 21) | (int64_t)z;
}

// 空间哈希: 排序键 + 平坦数组(C++ 下 O(n) 构建, 远快于哈希字典)
struct CellGrid {
	std::vector<int64_t> keys;
	std::vector<int32_t> order;
	std::vector<int64_t> ukey;
	std::vector<int32_t> ubeg;

	void build(const godot::PackedVector3Array &pos, real_t inv, int span) {
		const int n = (int)pos.size();
		keys.resize(n);
		order.resize(n);
		for (int i = 0; i < n; i++) {
			const godot::Vector3 &p = pos[i];
			int x = (int)std::floor(p.x * inv) + 512;
			int y = (int)std::floor(p.y * inv) + 512;
			int z = (int)std::floor(p.z * inv) + 512;
			x = x < 0 ? 0 : (x > span - 1 ? span - 1 : x);
			y = y < 0 ? 0 : (y > span - 1 ? span - 1 : y);
			z = z < 0 ? 0 : (z > span - 1 ? span - 1 : z);
			keys[i] = pack_cell(x, y, z);
			order[i] = i;
		}
		std::sort(order.begin(), order.end(), [&](int32_t a, int32_t b) {
			return keys[a] < keys[b];
		});
		ukey.clear();
		ubeg.clear();
		for (int i = 0; i < n; i++) {
			const int64_t k = keys[order[i]];
			if (ukey.empty() || ukey.back() != k) {
				ukey.push_back(k);
				ubeg.push_back(i);
			}
		}
		ubeg.push_back(n);
	}

	bool find(int64_t k, int &b, int &e) const {
		int lo = 0, hi = (int)ukey.size();
		while (lo < hi) {
			const int mid = (lo + hi) >> 1;
			if (ukey[mid] < k) {
				lo = mid + 1;
			} else {
				hi = mid;
			}
		}
		if (lo >= (int)ukey.size() || ukey[lo] != k) {
			return false;
		}
		b = ubeg[lo];
		e = ubeg[lo + 1];
		return true;
	}
};

inline godot::Vector3 closest_on_seg(const godot::Vector3 &a, const godot::Vector3 &b, const godot::Vector3 &p) {
	const godot::Vector3 ab = b - a;
	const real_t len2 = ab.length_squared();
	const real_t t = len2 > 1e-12f ? CLAMP((p - a).dot(ab) / len2, 0.0f, 1.0f) : 0.0f;
	return a + ab * t;
}

} // namespace cloth_internal

#endif
