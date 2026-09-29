#include "xpbd_hair_native.h"

#include "cloth_internal.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace godot;

XPBDHairNative::XPBDHairNative() {}
XPBDHairNative::~XPBDHairNative() {}

void XPBDHairNative::_bind_methods() {
	ClassDB::bind_method(D_METHOD("build"), &XPBDHairNative::build);
	ClassDB::bind_method(D_METHOD("step", "delta", "substeps"), &XPBDHairNative::step);
	ClassDB::bind_method(D_METHOD("update_mesh"), &XPBDHairNative::update_mesh);
	ClassDB::bind_method(D_METHOD("get_positions"), &XPBDHairNative::get_positions);
	ClassDB::bind_method(D_METHOD("get_guide_count"), &XPBDHairNative::get_guide_count);
	ClassDB::bind_method(D_METHOD("get_render_count"), &XPBDHairNative::get_render_count);
	ClassDB::bind_method(D_METHOD("get_segment_count"), &XPBDHairNative::get_segment_count);
	ClassDB::bind_method(D_METHOD("get_tri_r"), &XPBDHairNative::get_tri_r);
	ClassDB::bind_method(D_METHOD("get_asleep"), &XPBDHairNative::get_asleep);
	ClassDB::bind_method(D_METHOD("set_strand_count", "v"), &XPBDHairNative::set_strand_count);
	ClassDB::bind_method(D_METHOD("get_strand_count"), &XPBDHairNative::get_strand_count);
	ClassDB::bind_method(D_METHOD("set_segment_count", "v"), &XPBDHairNative::set_segment_count);
	ClassDB::bind_method(D_METHOD("set_hair_length", "v"), &XPBDHairNative::set_hair_length);
	ClassDB::bind_method(D_METHOD("get_hair_length"), &XPBDHairNative::get_hair_length);
	ClassDB::bind_method(D_METHOD("set_head_radius", "v"), &XPBDHairNative::set_head_radius);
	ClassDB::bind_method(D_METHOD("get_head_radius"), &XPBDHairNative::get_head_radius);
	ClassDB::bind_method(D_METHOD("set_head_center", "v"), &XPBDHairNative::set_head_center);
	ClassDB::bind_method(D_METHOD("get_head_center"), &XPBDHairNative::get_head_center);
	ClassDB::bind_method(D_METHOD("set_damping", "v"), &XPBDHairNative::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &XPBDHairNative::get_damping);
	ClassDB::bind_method(D_METHOD("set_stiffness", "v"), &XPBDHairNative::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &XPBDHairNative::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_render_strands", "v"), &XPBDHairNative::set_render_strands);
	ClassDB::bind_method(D_METHOD("get_render_strands"), &XPBDHairNative::get_render_strands);
	ClassDB::bind_method(D_METHOD("set_strand_width", "v"), &XPBDHairNative::set_strand_width);
	ClassDB::bind_method(D_METHOD("get_strand_width"), &XPBDHairNative::get_strand_width);
	ClassDB::bind_method(D_METHOD("set_tip_width", "v"), &XPBDHairNative::set_tip_width);
	ClassDB::bind_method(D_METHOD("get_tip_width"), &XPBDHairNative::get_tip_width);
	ClassDB::bind_method(D_METHOD("set_root_color", "v"), &XPBDHairNative::set_root_color);
	ClassDB::bind_method(D_METHOD("get_root_color"), &XPBDHairNative::get_root_color);
	ClassDB::bind_method(D_METHOD("set_tip_color", "v"), &XPBDHairNative::set_tip_color);
	ClassDB::bind_method(D_METHOD("get_tip_color"), &XPBDHairNative::get_tip_color);
	ClassDB::bind_method(D_METHOD("set_albedo", "c"), &XPBDHairNative::set_albedo);
	ClassDB::bind_method(D_METHOD("get_albedo"), &XPBDHairNative::get_albedo);
	ClassDB::bind_method(D_METHOD("set_material", "m"), &XPBDHairNative::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &XPBDHairNative::get_material);
	ClassDB::bind_method(D_METHOD("set_obstacle", "on", "c", "r"), &XPBDHairNative::set_obstacle);
	ClassDB::bind_method(D_METHOD("set_body_collision", "on", "a", "b", "r"), &XPBDHairNative::set_body_collision);
	ClassDB::bind_method(D_METHOD("get_obstacle_on"), &XPBDHairNative::get_obstacle_on);
	ClassDB::bind_method(D_METHOD("get_obstacle_c"), &XPBDHairNative::get_obstacle_c);
	ClassDB::bind_method(D_METHOD("get_obstacle_r"), &XPBDHairNative::get_obstacle_r);
	ClassDB::bind_method(D_METHOD("set_head_moving", "v"), &XPBDHairNative::set_head_moving);
	ClassDB::bind_method(D_METHOD("get_mesh"), &XPBDHairNative::get_mesh);
}

// ============================================================================
// 构建: 头皮上均匀采样发根, 每束沿初始方向铺开粒子链
// ============================================================================
void XPBDHairNative::_build_strands() {
	pos.clear();
	prev.clear();
	vel.clear();
	roots.clear();
	root_normals.clear();
	rest_dir.clear();
	seg_len.clear();
	c_ia.clear(); c_ib.clear(); c_rest.clear(); c_type.clear(); c_stiff.clear();

	// 头皮采样: 用斐波那契球面分布(均匀且无网格伪影), 只取上半球偏前
	std::vector<Vector3> scalp;
	std::vector<Vector3> nrm;
	const int total = strand_count * 3; // 过采样后筛选
	const real_t golden = 3.14159265359f * (3.0f - std::sqrt(5.0f));
	for (int i = 0; i < total && (int)scalp.size() < strand_count; i++) {
		const real_t y = 1.0f - (2.0f * (real_t)i + 1.0f) / (real_t)total;
		if (y < 0.05f) {
			continue; // 只取上半球偏上区域(发际线以上)
		}
		const real_t rr = std::sqrt(MAX(0.0f, 1.0f - y * y));
		const real_t th = golden * (real_t)i;
		Vector3 n = Vector3(rr * std::cos(th), y, rr * std::sin(th));
		// 稍微往前偏, 避免后脑勺过密
		if (n.z < -0.9f) {
			continue;
		}
		scalp.push_back(head_center + n * head_radius);
		nrm.push_back(n);
	}

	const int np = segment_count + 1;
	for (size_t s = 0; s < scalp.size(); s++) {
		const Vector3 root = scalp[s];
		const Vector3 n = nrm[s];
		// 初始方向: 沿法线向外 + 重力方向混合(自然垂落趋势)
		Vector3 dir = (n * 0.6f + Vector3(0, -0.6f, 0.1f)).normalized();
		roots.push_back(root);
		root_normals.push_back(n);
		for (int k = 0; k < np; k++) {
			const real_t t = (real_t)k / (real_t)segment_count;
			const Vector3 p = root + dir * (hair_length * t);
			pos.push_back(p);
			prev.push_back(p);
			vel.push_back(Vector3());
			if (k == 0) {
				rest_dir.push_back(dir);
			}
		}
	}
	seg_len.resize(scalp.size() * segment_count);
	for (int s = 0; s < (int)scalp.size(); s++) {
		for (int k = 0; k < segment_count; k++) {
			seg_len.set(s * segment_count + k, hair_length / (real_t)segment_count);
		}
	}

	// 约束: 距离(相邻) + 弯曲(隔一)
	const int strands = (int)scalp.size();
	for (int s = 0; s < strands; s++) {
		const int base = s * np;
		for (int k = 0; k + 1 < np; k++) {
			c_ia.push_back(base + k);
			c_ib.push_back(base + k + 1);
			c_rest.push_back(hair_length / (real_t)segment_count);
			c_type.push_back(0);
			// 刚性: 发根硬、发梢软
			const real_t t = (real_t)k / (real_t)segment_count;
			c_stiff.push_back(CLAMP(root_stiffness + (stiffness - root_stiffness) * t, 0.05f, 1.0f));
		}
		for (int k = 0; k + 2 < np; k++) {
			c_ia.push_back(base + k);
			c_ib.push_back(base + k + 2);
			c_rest.push_back(hair_length * 2.0f / (real_t)segment_count);
			c_type.push_back(1);
			c_stiff.push_back(CLAMP(stiffness * 0.6f, 0.02f, 1.0f));
		}
	}
	built = true;
}

void XPBDHairNative::build() {
	_build_strands();
	// 材质: 顶点色渐变(发根/发梢), 双面
	{
		Ref<StandardMaterial3D> m;
		m.instantiate();
		m->set_shading_mode(BaseMaterial3D::SHADING_MODE_PER_PIXEL);
		m->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		m->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
		m->set_roughness(0.55f);
		m->set_albedo(albedo);
		material = m;
	}
	update_mesh();
}

// ============================================================================
// 物理步进: 子步 + 距离/弯曲约束松弛 + 头部碰撞 + 发束间自碰
// ============================================================================
void XPBDHairNative::step(real_t delta, int substeps) {
	if (!built) {
		return;
	}
	const int n = pos.size();
	const int sub = MAX(substeps, 1);
	const real_t h = delta / (real_t)sub;
	head_prev = head_center;

	for (int s = 0; s < sub; s++) {
		// --- 积分(含头部运动: 甩头时粒子继承头速度) ---
		Vector3 head_vel;
		if (head_moving) {
			head_vel = (head_center - head_prev) / h;
		}
		for (int i = 0; i < n; i++) {
			const int k = i % (segment_count + 1);
			if (k == 0) {
				continue; // 发根钉在头皮上
			}
			prev.set(i, pos[i]);
			Vector3 v = vel[i] + Vector3(0, -9.8f * gravity_scale, 0) * h + head_vel;
			v *= (1.0f - damping);
			vel.set(i, v);
			pos.set(i, pos[i] + v * h);
		}

		// --- 约束松弛(距离 + 弯曲), 迭代 3 次 ---
		const int nc = c_ia.size();
		for (int it = 0; it < 3; it++) {
			for (int c = 0; c < nc; c++) {
				const int ia = c_ia[c];
				const int ib = c_ib[c];
				const real_t rest = c_rest[c];
				const real_t stiff = c_stiff[c];
				Vector3 d = pos[ia] - pos[ib];
				const real_t ln = d.length();
				if (ln < 1e-9f) {
					continue;
				}
				const real_t diff = (ln - rest) / ln * stiff;
				const Vector3 corr = d * diff * 0.5f;
				// 发根粒子固定(权重 0)
				const bool ra = (ia % (segment_count + 1)) == 0;
				const bool rb = (ib % (segment_count + 1)) == 0;
				const real_t wa = ra ? 0.0f : 1.0f;
				const real_t wb = rb ? 0.0f : 1.0f;
				const real_t w = wa + wb;
				if (w <= 0.0f) {
					continue;
				}
				pos.set(ia, pos[ia] - corr * (wa / w) * 2.0f * 0.5f);
				pos.set(ib, pos[ib] + corr * (wb / w) * 2.0f * 0.5f);
			}
			// 发根钉回头皮
			for (int s2 = 0; s2 < (int)roots.size(); s2++) {
				pos.set(s2 * (segment_count + 1), roots[s2]);
			}
			// 松弛会把粒子推回头部内(约束不感知碰撞) -> 每轮松弛后再做一次头部碰撞
			for (int i = 0; i < n; i++) {
				_collide_head(i);
			}
		}

		// --- 头部碰撞 + 障碍球 + 身体胶囊 ---
		for (int i = 0; i < n; i++) {
			_collide_head(i);
			if (obstacle_on) {
				_collide_obstacle(i);
			}
			if (body_on) {
				_collide_body(i);
			}
		}

		// --- 速度恢复: 由位置差反推(含阻尼) ---
		const real_t inv_h = 1.0f / h;
		for (int i = 0; i < n; i++) {
			const int k = i % (segment_count + 1);
			if (k == 0) {
				vel.set(i, Vector3());
				continue;
			}
			vel.set(i, (pos[i] - prev[i]) * inv_h);
		}
	}
}

// 可移动障碍球: 推出 + 摩擦
void XPBDHairNative::_collide_obstacle(int i) {
	const Vector3 d = pos[i] - obstacle_c;
	const real_t rr = obstacle_r + collide_radius;
	const real_t d2 = d.length_squared();
	if (d2 >= rr * rr) {
		return;
	}
	const real_t ln = std::sqrt(MAX(d2, 1e-12f));
	const Vector3 nrm = ln > 1e-9f ? d / ln : Vector3(0, 1, 0);
	pos.set(i, obstacle_c + nrm * rr);
	Vector3 v = vel[i];
	const real_t vn = v.dot(nrm);
	if (vn < 0.0f) {
		v -= nrm * vn * (1.0f - friction);
		vel.set(i, v);
	}
}

// 身体胶囊碰撞: 头发垂到身上不穿模(与布料碰撞同款 线段-点 最近距离)
void XPBDHairNative::_collide_body(int i) {
	const Vector3 nearPt = cloth_internal::closest_on_seg(body_a, body_b, pos[i]);
	const Vector3 d = pos[i] - nearPt;
	const real_t rr = body_r + collide_radius;
	const real_t d2 = d.length_squared();
	if (d2 >= rr * rr) {
		return;
	}
	const real_t ln = std::sqrt(MAX(d2, 1e-12f));
	const Vector3 nrm = ln > 1e-9f ? d / ln : Vector3(0, 1, 0);
	pos.set(i, nearPt + nrm * rr);
	Vector3 v = vel[i];
	const real_t vn = v.dot(nrm);
	if (vn < 0.0f) {
		v -= nrm * vn * (1.0f - friction);
		vel.set(i, v);
	}
}

void XPBDHairNative::_collide_head(int i) {
	const Vector3 d = pos[i] - head_center;
	const real_t rr = head_radius + collide_radius;
	const real_t d2 = d.length_squared();
	if (d2 >= rr * rr) {
		return;
	}
	const real_t ln = std::sqrt(MAX(d2, 1e-12f));
	Vector3 nrm = ln > 1e-9f ? d / ln : Vector3(0, 1, 0);
	// 推出 + 沿头面切向滑移(摩擦)
	Vector3 target = head_center + nrm * rr;
	Vector3 slide = (target - pos[i]);
	pos.set(i, target);
	// 切向速度阻尼
	Vector3 v = vel[i];
	const real_t vn = v.dot(nrm);
	if (vn < 0.0f) {
		v -= nrm * vn * (1.0f - friction);
		vel.set(i, v);
	}
}

// ============================================================================
// 渲染发丝插值: 每根渲染发丝按 K 近邻导向束反距离加权
// ============================================================================
void XPBDHairNative::_build_render_strands() {
	const int ng = roots.size();
	blend_idx.clear();
	blend_w.clear();
	blend_idx.resize(render_strands * blend_k);
	blend_w.resize(render_strands * blend_k);

	// 渲染发根: 在头皮上再采一批(密度更高), 每个找 K 近邻导向发根
	std::vector<Vector3> rr_roots;
	const int total = render_strands * 3;
	const real_t golden = 3.14159265359f * (3.0f - std::sqrt(5.0f));
	for (int i = 0; i < total && (int)rr_roots.size() < render_strands; i++) {
		const real_t y = 1.0f - (2.0f * (real_t)i + 1.0f) / (real_t)total;
		if (y < 0.05f) {
			continue;
		}
		const real_t rr = std::sqrt(MAX(0.0f, 1.0f - y * y));
		const real_t th = golden * (real_t)i + 0.37f; // 偏移采样, 与导向束错开
		Vector3 n = Vector3(rr * std::cos(th), y, rr * std::sin(th));
		if (n.z < -0.9f) {
			continue;
		}
		rr_roots.push_back(head_center + n * head_radius);
	}

	for (int r = 0; r < (int)rr_roots.size(); r++) {
		// K 近邻(简单选择排序, render_strands 通常几百, K=4)
		int idx[8];
		real_t dist[8];
		const int kk = MIN(blend_k, 8);
		for (int k = 0; k < kk; k++) {
			idx[k] = -1;
			dist[k] = 1e9f;
		}
		for (int g = 0; g < ng; g++) {
			const real_t d = rr_roots[r].distance_squared_to(roots[g]);
			for (int k = 0; k < kk; k++) {
				if (d < dist[k]) {
					for (int m = kk - 1; m > k; m--) {
						idx[m] = idx[m - 1];
						dist[m] = dist[m - 1];
					}
					idx[k] = g;
					dist[k] = d;
					break;
				}
			}
		}
		// 反距离权重(平方反比, 归一化)
		real_t wsum = 0.0f;
		for (int k = 0; k < kk; k++) {
			dist[k] = 1.0f / (dist[k] + 1e-6f);
			wsum += dist[k];
		}
		for (int k = 0; k < kk; k++) {
			blend_idx.set(r * blend_k + k, idx[k]);
			blend_w.set(r * blend_k + k, dist[k] / wsum);
		}
	}
	render_strands = (int)rr_roots.size();
}

void XPBDHairNative::update_mesh() {
	if (!built) {
		return;
	}
	// 插值权重只需构建一次
	if (blend_idx.size() == 0) {
		_build_render_strands();
	}

	const int np = segment_count + 1;
	PackedVector3Array verts;
	PackedColorArray colors;
	PackedInt32Array indices;
	verts.resize(render_strands * np * 2);
	colors.resize(render_strands * np * 2);

	// 每渲染发丝的横向铺开方向: 用其发根法线与重力混合的叉积(稳定, 不随相机变化)
	for (int r = 0; r < render_strands; r++) {
		const int g0 = blend_idx[r * blend_k];
		Vector3 lateral = root_normals[g0].cross(Vector3(0, 1, 0));
		if (lateral.length_squared() < 1e-10f) {
			lateral = Vector3(1, 0, 0);
		}
		lateral = lateral.normalized();

		for (int k = 0; k < np; k++) {
			// K 近邻反距离加权插值导向束粒子位置
			Vector3 p;
			for (int kk = 0; kk < blend_k; kk++) {
				const int g = blend_idx[r * blend_k + kk];
				const real_t w = blend_w[r * blend_k + kk];
				p += pos[g * np + k] * w;
			}
			const real_t t = (real_t)k / (real_t)segment_count;
			const real_t half_w = (strand_width + (tip_width - strand_width) * t) * 0.5f;
			const int vi = (r * np + k) * 2;
			verts.set(vi, p - lateral * half_w);
			verts.set(vi + 1, p + lateral * half_w);
			const Color col = root_color.lerp(tip_color, t);
			colors.set(vi, col);
			colors.set(vi + 1, col);
		}
	}

	for (int r = 0; r < render_strands; r++) {
		for (int k = 0; k + 1 < np; k++) {
			const int v00 = (r * np + k) * 2;
			const int v01 = v00 + 1;
			const int v10 = (r * np + k + 1) * 2;
			const int v11 = v10 + 1;
			indices.push_back(v00); indices.push_back(v10); indices.push_back(v01);
			indices.push_back(v01); indices.push_back(v10); indices.push_back(v11);
		}
	}

	Ref<ArrayMesh> m;
	m.instantiate();
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = verts;
	arrays[Mesh::ARRAY_COLOR] = colors;
	arrays[Mesh::ARRAY_INDEX] = indices;
	m->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	if (material.is_valid()) {
		m->surface_set_material(0, material);
	}
	mesh = m;
}
