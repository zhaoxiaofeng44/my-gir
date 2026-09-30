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
	ClassDB::bind_method(D_METHOD("get_render_requested"), &XPBDHairNative::get_render_requested);
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
	ClassDB::bind_method(D_METHOD("set_head_ellipsoid", "r"), &XPBDHairNative::set_head_ellipsoid);
	ClassDB::bind_method(D_METHOD("get_head_ellipsoid"), &XPBDHairNative::get_head_ellipsoid);
	ClassDB::bind_method(D_METHOD("set_damping", "v"), &XPBDHairNative::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &XPBDHairNative::get_damping);
	ClassDB::bind_method(D_METHOD("set_stiffness", "v"), &XPBDHairNative::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &XPBDHairNative::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_root_stiffness", "v"), &XPBDHairNative::set_root_stiffness);
	ClassDB::bind_method(D_METHOD("get_root_stiffness"), &XPBDHairNative::get_root_stiffness);
	ClassDB::bind_method(D_METHOD("set_friction", "v"), &XPBDHairNative::set_friction);
	ClassDB::bind_method(D_METHOD("get_friction"), &XPBDHairNative::get_friction);
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
	ClassDB::bind_method(D_METHOD("translate_follow", "delta"), &XPBDHairNative::translate_follow);
	ClassDB::bind_method(D_METHOD("get_body_on"), &XPBDHairNative::get_body_on);
	ClassDB::bind_method(D_METHOD("get_body_a"), &XPBDHairNative::get_body_a);
	ClassDB::bind_method(D_METHOD("get_body_b"), &XPBDHairNative::get_body_b);
	ClassDB::bind_method(D_METHOD("get_body_r"), &XPBDHairNative::get_body_r);
	ClassDB::bind_method(D_METHOD("get_obstacle_on"), &XPBDHairNative::get_obstacle_on);
	ClassDB::bind_method(D_METHOD("get_obstacle_c"), &XPBDHairNative::get_obstacle_c);
	ClassDB::bind_method(D_METHOD("get_obstacle_r"), &XPBDHairNative::get_obstacle_r);
	ClassDB::bind_method(D_METHOD("set_head_moving", "v"), &XPBDHairNative::set_head_moving);
	ClassDB::bind_method(D_METHOD("set_wind", "v"), &XPBDHairNative::set_wind);
	ClassDB::bind_method(D_METHOD("get_wind"), &XPBDHairNative::get_wind);
	ClassDB::bind_method(D_METHOD("get_mesh"), &XPBDHairNative::get_mesh);
	ClassDB::bind_method(D_METHOD("set_scalp_mesh", "verts"), &XPBDHairNative::set_scalp_mesh);
	ClassDB::bind_method(D_METHOD("clear_scalp_roots"), &XPBDHairNative::clear_scalp_roots);
	ClassDB::bind_method(D_METHOD("get_scalp_ready"), &XPBDHairNative::get_scalp_ready);
	ClassDB::bind_method(D_METHOD("get_scalp_on"), &XPBDHairNative::get_scalp_on);
	ClassDB::bind_method(D_METHOD("get_scalp_count"), &XPBDHairNative::get_scalp_count);
	ClassDB::bind_method(D_METHOD("get_scalp_roots"), &XPBDHairNative::get_scalp_roots);
	ClassDB::bind_method(D_METHOD("get_scalp_normals"), &XPBDHairNative::get_scalp_normals);
	ClassDB::bind_method(D_METHOD("get_scalp_dirs"), &XPBDHairNative::get_scalp_dirs);
	ClassDB::bind_method(D_METHOD("get_scalp_lengths"), &XPBDHairNative::get_scalp_lengths);
	ClassDB::bind_method(D_METHOD("get_scalp_center"), &XPBDHairNative::get_scalp_center);
	ClassDB::bind_method(D_METHOD("set_face_dir", "v"), &XPBDHairNative::set_face_dir);
	ClassDB::bind_method(D_METHOD("get_face_dir"), &XPBDHairNative::get_face_dir);
	ClassDB::bind_method(D_METHOD("set_face_guard", "on", "pt", "nrm", "min_y", "r"), &XPBDHairNative::set_face_guard);
	ClassDB::bind_method(D_METHOD("get_face_guard_on"), &XPBDHairNative::get_face_guard_on);
	ClassDB::bind_method(D_METHOD("set_scalp_grid", "rows", "cols"), &XPBDHairNative::set_scalp_grid);
	ClassDB::bind_method(D_METHOD("set_scalp_lengths", "top", "bottom", "bang", "jitter"), &XPBDHairNative::set_scalp_lengths);
	ClassDB::bind_method(D_METHOD("set_scalp_region", "face_phi", "face_z", "bang_z"), &XPBDHairNative::set_scalp_region);
	ClassDB::bind_method(D_METHOD("follow_skeleton", "root_now", "head_now", "dt"), &XPBDHairNative::follow_skeleton);
	ClassDB::bind_method(D_METHOD("set_render_clamp", "min_y", "snap"), &XPBDHairNative::set_render_clamp);
	ClassDB::bind_method(D_METHOD("get_render_min_y"), &XPBDHairNative::get_render_min_y);
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

	// 真实头皮发根模式(HS2 捏人角色): 发根/法线/长度都由调用方给出,
	// 这里只负责按这些发根铺开粒子链, 并把"骨骼局部静止形"记下来(用于换头皮时保留惯性)。
	if (scalp_on && scalp_rest.size() > 0) {
		const int ns = scalp_rest.size();
		const int np = segment_count + 1;
		for (int s = 0; s < ns; s++) {
			const Vector3 root = scalp_rest[s];
			Vector3 n = scalp_normals.size() == ns ? scalp_normals[s] : Vector3(0, 1, 0);
			if (n.length_squared() < 1e-8f) {
				n = Vector3(0, 1, 0);
			}
			n = n.normalized();
			const real_t L = strand_len.size() == ns ? strand_len[s] : hair_length;
			const real_t seg = L / (real_t)segment_count;
			roots.push_back(root);
			root_normals.push_back(n);
			// 初始形状: 由采样阶段给出的 scalp_dir 起势, 再逐渐转为垂直下落。
			const Vector3 g(0, -1, 0);
			const Vector3 d0 = scalp_dir.size() == ns ? scalp_dir[s] : n;
			for (int k = 0; k < np; k++) {
				const real_t t = (real_t)k / (real_t)segment_count;
				const Vector3 dd = d0.lerp(g, CLAMP(t * 1.0f, 0.0f, 1.0f)).normalized();
				const Vector3 p = k == 0 ? root : pos[pos.size() - 1] + dd * seg;
				pos.push_back(p);
				prev.push_back(p);
				vel.push_back(Vector3());
				if (k == 0) {
					rest_dir.push_back(dd);
				}
			}
		}
		seg_len.resize(ns * segment_count);
		for (int s = 0; s < ns; s++) {
			const real_t L = strand_len.size() == ns ? strand_len[s] : hair_length;
			for (int k = 0; k < segment_count; k++) {
				seg_len.set(s * segment_count + k, L / (real_t)segment_count);
			}
		}
		const int np2 = segment_count + 1;
		for (int s = 0; s < ns; s++) {
			const int base = s * np2;
			const real_t L = strand_len.size() == ns ? strand_len[s] : hair_length;
			for (int k = 0; k + 1 < np2; k++) {
				c_ia.push_back(base + k);
				c_ib.push_back(base + k + 1);
				c_rest.push_back(L / (real_t)segment_count);
				c_type.push_back(0);
				const real_t t = (real_t)k / (real_t)segment_count;
				c_stiff.push_back(CLAMP(root_stiffness + (stiffness - root_stiffness) * t, 0.05f, 1.0f));
			}
			for (int k = 0; k + 2 < np2; k++) {
				c_ia.push_back(base + k);
				c_ib.push_back(base + k + 2);
				c_rest.push_back(L * 2.0f / (real_t)segment_count);
				c_type.push_back(1);
				c_stiff.push_back(CLAMP(stiffness * 0.6f, 0.02f, 1.0f));
			}
		}
		built = true;
		return;
	}

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

void XPBDHairNative::set_model_transform(const Vector3 &origin, real_t yaw_deg) {
	// record model linear velocity (drag inertia)
	model_vel = (origin - model_origin) / 0.016f;
	model_prev_origin = model_origin;
	model_origin = origin;
	model_yaw = yaw_deg * 3.14159265359f / 180.0f;
	const Basis rot = Basis::from_euler(Vector3(0, model_yaw, 0));
	head_center = origin + rot.xform(head_center);
	body_a = origin + rot.xform(body_a);
	body_b = origin + rot.xform(body_b);
	obstacle_c = origin + rot.xform(obstacle_c);
	for (int i = 0; i < roots.size(); i++) {
		Vector3 local = roots[i] - model_prev_origin;
		Vector3 rotated = rot.xform(local);
		roots.set(i, origin + rotated);
	}
	Vector3 delta_p = origin - model_prev_origin;
	if (delta_p.length_squared() > 1e-12f) {
		for (int i = 0; i < pos.size(); i++) {
			if ((i % (segment_count + 1)) == 0) {
				continue;
			}
			pos.set(i, pos[i] + delta_p);
		}
	}
}

void XPBDHairNative::translate_follow(const Vector3 &delta) {
	if (delta.length_squared() < 1e-12f) {
		return;
	}
	for (int i = 0; i < roots.size(); i++) {
		roots.set(i, roots[i] + delta);
	}
	for (int i = 0; i < pos.size(); i++) {
		pos.set(i, pos[i] + delta);
	}
	head_center += delta;
	body_a += delta;
	body_b += delta;
	const Vector3 inherit = -delta * 8.0f;
	for (int i = 0; i < pos.size(); i++) {
		const int k = i % (segment_count + 1);
		if (k != 0) {
			vel.set(i, vel[i] + inherit);
		}
	}
}

// ============================================================================
// 真实头皮采样(HS2 捏人角色使用) —— 全部在插件内完成
// ----------------------------------------------------------------------------
// 调用方只交进"头模顶点云"(静止空间), 之后:
//   1) 球坐标网格 (行=极角, 列=方位角) 覆盖整个头皮
//   2) 每个格点把头心指向该方向, 取方向最一致的头模顶点 -> 发根 + 头皮法线
//      (这样发根天然贴在头模表面上, 而不是球面近似上)
//   3) 剔除正脸区(朝前且低于发际线) -> 头发不会长到脸上
//   4) 最远点采样选出 strand_count 根 -> 蓝噪声式均匀分布(避免规则网格的条带感)
//   5) 按所处区域给发长: 刘海短(到眉毛, 且侧向梳开) / 头顶中 / 颈后长
// ============================================================================
void XPBDHairNative::set_scalp_mesh(const PackedVector3Array &verts) {
	head_verts = verts;
	skel_prev.clear();
	skel_vel = Vector3();
}

void XPBDHairNative::_sample_scalp() {
	scalp_rest.clear();
	scalp_normals.clear();
	scalp_dir.clear();
	strand_len.clear();
	scalp_on = false;
	if (head_verts.size() < 32) {
		return;
	}
	// ---- 头心: 全部头模顶点的质心 ----
	Vector3 hc;
	for (int i = 0; i < head_verts.size(); i++) {
		hc += head_verts[i];
	}
	hc = hc / (real_t)head_verts.size();
	scalp_center = hc;
	// ---- 头部 y 范围(用于发长按高度插值) ----
	real_t ylo = 1e9f, yhi = -1e9f;
	for (int i = 0; i < head_verts.size(); i++) {
		ylo = MIN(ylo, head_verts[i].y);
		yhi = MAX(yhi, head_verts[i].y);
	}
	const real_t yspan = MAX(yhi - ylo, 1e-3f);

	// ---- 脸朝向 ----
	// 由调用方给出(set_face_dir)。**不能**用"离头心竖轴最远"来自动识别 ——
	// 后脑勺(枕骨)比鼻尖凸得更多, 那样会把"脸"识别成后脑, 于是头发全长到前面去,
	// 把整张脸盖住。这里必须拿到真正的解剖学前方。
	Vector3 face_dir = face_axis;
	{
		const Vector3 fl(face_dir.x, 0.0f, face_dir.z);
		face_dir = fl.length_squared() > 1e-8f ? fl.normalized() : Vector3(0, 0, 1);
	}

	// ---- 球坐标网格采样 ----
	struct Cand {
		Vector3 pos;
		Vector3 nrm;
		real_t t;
		Vector3 dir;
		real_t front;    // 水平方向与脸朝向的夹角余弦(-1=后脑, +1=正前)
		real_t bangness; // 0=普通长发, 1=纯刘海(越靠前越短、越要侧向梳开)
	};
	std::vector<Cand> pool;
	const int rows = MAX(scalp_rows, 3);
	const int cols = MAX(scalp_cols, 4);
	for (int r = 0; r < rows; r++) {
		// 极角从 0 起 —— 之前从 0.18 起会把头顶(发旋)整块漏掉, 渲染出来就是"秃顶"
		const real_t phi = 0.0f + (1.62f - 0.0f) * ((real_t)r + 0.5f) / (real_t)rows;
		for (int c = 0; c < cols; c++) {
			const real_t theta = 6.28318530718f * ((real_t)c + 0.5f) / (real_t)cols;
			Vector3 dir = Vector3(std::sin(phi) * std::sin(theta), std::cos(phi), std::sin(phi) * std::cos(theta));
			if (dir.length_squared() < 1e-8f) {
				continue;
			}
			dir = dir.normalized();
			// 正脸区: 朝前且明显低于发际线 -> 不长头发(发际线与两侧鬓角保留)
			const real_t frontness = dir.x * face_dir.x + dir.z * face_dir.z;
			if (frontness > face_z && phi > face_phi) {
				continue;
			}
			// 沿 dir 取头模的"支撑点": r = max_i (v_i - c)·dir
			// —— 这是凸包在 dir 上的表面点。之前用"方向最一致的顶点"会让
			//    phi≈0 的那一整圈全部塌到同一个发旋顶点上, 结果就是**秃顶**。
			real_t rmax = -1e9f;
			for (int i = 0; i < head_verts.size(); i++) {
				const real_t s = (head_verts[i] - hc).dot(dir);
				if (s > rmax) {
					rmax = s;
				}
			}
			if (rmax < 0.01f) {
				continue;
			}
			const Vector3 surf = hc + dir * rmax;
			// 贴到最近的真实头模顶点上(支撑点可能在两个顶点之间)
			real_t bd = 1e18f;
			Vector3 bp = surf;
			for (int i = 0; i < head_verts.size(); i++) {
				const real_t d2 = head_verts[i].distance_squared_to(surf);
				if (d2 < bd) {
					bd = d2;
					bp = head_verts[i];
				}
			}
			Vector3 nrm = bp - hc;
			if (nrm.length_squared() < 1e-8f) {
				nrm = dir;
			}
			nrm = nrm.normalized();
			// 刘海程度(连续量, 不硬切): 越靠前 + 越低于头顶 -> 越像刘海。
			// 硬切会在边界留下"一撮突然变长"的帘子, 正是糊脸的那几根。
			const real_t bangness = CLAMP(frontness, 0.0f, 1.0f) * CLAMP((phi - 0.20f) / 0.60f, 0.0f, 1.0f) * CLAMP((bang_phi_hi + 0.25f - phi) / (bang_phi_hi + 0.25f), 0.0f, 1.0f);
			Cand cd;
			cd.pos = bp + nrm * 0.004f;
			cd.nrm = nrm;
			cd.t = (yhi - cd.pos.y) / yspan;
			cd.dir = dir;
			cd.front = frontness;
			cd.bangness = bangness;
			pool.push_back(cd);
		}
	}
	if (pool.empty()) {
		return;
	}

	// ---- 最远点采样: 选 strand_count 根, 蓝噪声式均匀 ----
	const int want = MIN((int)strand_count, (int)pool.size());
	std::vector<int> chosen;
	chosen.reserve(want);
	std::vector<real_t> dist(pool.size(), 1e18f);
	// 第一根: 取池中偏后的一根(保证后脑一定被覆盖)
	{
		std::vector<int> byz(pool.size());
		for (int i = 0; i < (int)pool.size(); i++) {
			byz[i] = i;
		}
		std::sort(byz.begin(), byz.end(), [&](int a, int b) { return pool[a].pos.z < pool[b].pos.z; });
		const int first = byz[(int)(pool.size() * 0.25)];
		chosen.push_back(first);
		for (int i = 0; i < (int)pool.size(); i++) {
			dist[i] = pool[i].pos.distance_squared_to(pool[first].pos);
		}
	}
	while ((int)chosen.size() < want) {
		int bi = -1;
		real_t bd = -1.0f;
		for (int i = 0; i < (int)pool.size(); i++) {
			if (dist[i] > bd) {
				bd = dist[i];
				bi = i;
			}
		}
		if (bi < 0) {
			break;
		}
		chosen.push_back(bi);
		dist[bi] = -1.0f;
		for (int i = 0; i < (int)pool.size(); i++) {
			const real_t d = pool[i].pos.distance_squared_to(pool[bi].pos);
			if (d < dist[i]) {
				dist[i] = d;
			}
		}
	}

	// ---- 组装发根/法线/初始方向/发长 ----
	uint32_t rng = 20250929u;
	for (int k = 0; k < (int)chosen.size(); k++) {
		const Cand &cd = pool[chosen[k]];
		const Vector3 nrm = cd.nrm;
		const Vector3 g(0, -1, 0);
		real_t L;
		Vector3 dir;
		// 水平"向后"/"侧向"单位向量(相对脸朝向)
		const Vector3 back(-face_dir.x, 0.0f, -face_dir.z);
		const Vector3 side_v(-face_dir.z, 0.0f, face_dir.x);
		const real_t bang = CLAMP(cd.bangness, 0.0f, 1.0f);
		// 长发基准: 头顶短、颈后长
		L = len_top + (len_bottom - len_top) * CLAMP(cd.t, 0.0f, 1.0f);
		L *= 1.0f + 0.10f * MAX(-cd.front, 0.0f);   // 后脑略长
		L *= 1.0f - 0.55f * bang;                   // 越靠前越短
		if (bang > 0.55f) {
			L = MIN(L, bang_len);                   // 真刘海封顶到眉毛
		}
		// 起势方向 = 沿头皮的"下坡"(重力切向) + 向后偏置 + 略微离面。
		// 向后偏置很关键: 没有它, 头顶发丝会顺势往前扑, 盖住整张脸。
		Vector3 tan = g - nrm * g.dot(nrm);
		if (tan.length_squared() < 1e-6f) {
			// 发旋(极点)处切向退化 —— **不能**退回"径向朝外"(那正好朝前), 直接用"向后"。
			tan = back;
		}
		const real_t back_amt = 0.55f + 0.45f * MAX(cd.front, 0.0f);
		dir = (tan.normalized() * 0.45f + back * back_amt + g * 0.30f + nrm * 0.12f).normalized();
		if (bang > 0.01f) {
			// 刘海再额外"向两侧 + 斜后"梳开, 顺着头皮走而不是垂在眼前
			const real_t side = (cd.pos.x * side_v.x + cd.pos.z * side_v.z) >= 0.0f ? 1.0f : -1.0f;
			const Vector3 bang_dir = (side_v * (side * 0.80f) + back * 0.62f + g * 0.18f).normalized();
			dir = dir.lerp(bang_dir, bang).normalized();
		}
		// xorshift 抖动, 避免"齐刷刷"像假发
		rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
		const real_t jit = ((real_t)(rng % 20001) / 10000.0f - 1.0f) * len_jitter;
		L *= 1.0f + jit;
		scalp_rest.push_back(cd.pos);
		scalp_normals.push_back(nrm);
		scalp_dir.push_back(dir);
		strand_len.push_back(L);
	}
	scalp_on = scalp_rest.size() > 0;
	if (scalp_on) {
		strand_count = scalp_rest.size();
	}
	printf("[HairNative] scalp sample: head verts %d -> candidates %d -> strands %d\n",
			(int)head_verts.size(), (int)pool.size(), (int)scalp_rest.size());
}

// 每帧由骨架驱动发根: 位移带着走(贴头皮), 速度给粒子做惯性(甩头/走动时头发会甩)。
void XPBDHairNative::follow_skeleton(const PackedVector3Array &root_now, const Vector3 &head_now, real_t dt) {
	if (!scalp_on || root_now.size() != roots.size()) {
		return;
	}
	const int ns = roots.size();
	const bool has_prev = skel_prev.size() == ns;
	const Vector3 d_head = head_now - skel_head;
	for (int i = 0; i < ns; i++) {
		const Vector3 d = root_now[i] - roots[i];
		if (d.length_squared() > 1e-12f) {
			const int seg = segment_count + 1;
			for (int k = 0; k < seg; k++) {
				const int pi = i * seg + k;
				pos.set(pi, pos[pi] + d);
			}
		}
		roots.set(i, root_now[i]);
	}
	// 骨骼线速度: 发根位移 / dt(取所有发根的平均, 抑制个别根的抖动)
	if (has_prev && dt > 1e-5f) {
		Vector3 sum;
		for (int i = 0; i < ns; i++) {
			sum += (root_now[i] - skel_prev[i]) / dt;
		}
		skel_vel = sum / (real_t)ns;
	} else {
		skel_vel = Vector3();
	}
	skel_prev = root_now;
	skel_head = head_now;
	head_center = head_now;
	// 发根刚性钉住: 位移后立刻复位(约束里的根权重本来就是 0, 这里防止残留漂移)
	const int seg = segment_count + 1;
	for (int i = 0; i < ns; i++) {
		pos.set(i * seg, roots[i]);
	}
}

void XPBDHairNative::build() {
	if (head_verts.size() >= 32) {
		_sample_scalp();   // 调用方交了头模 -> 插件内采样头皮发根
	}
	_build_strands();
	// 头皮采样变了 -> 插值权重必须重建(否则沿用旧权重会索引到不存在的导向束)
	blend_idx.clear();
	blend_w.clear();
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
	model_vel *= 0.5f;

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
			// 骨骼线速度(甩头/走动的头部位移)作为惯性源, 与模型整体速度模型一致
			Vector3 v = vel[i] + Vector3(0, -9.8f * gravity_scale, 0) * h + head_vel + wind + model_vel + skel_vel;
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
			// 松弛会把粒子推回碰撞体内(约束不感知碰撞) -> 每轮松弛后再做一次全碰撞
			for (int i = 0; i < n; i++) {
				_collide_head(i);
				_collide_face(i);
				if (body_on) {
					_collide_body(i);
				}
				if (obstacle_on) {
					_collide_obstacle(i);
				}
			}
		}

		// --- 头部碰撞 + 脸部挡板 + 障碍球 + 身体胶囊 ---
		for (int i = 0; i < n; i++) {
			_collide_head(i);
			_collide_face(i);
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

// 脸部挡板: 把"垂到脸前"的发丝推回脑后。
// 为什么需要它: 发根初始方向朝后只决定**起始形状**, 物理跑稳定后重力照样会把
// 前额那一撮拉直成一道帘子挂在眼前。所以必须在**求解阶段**加一个硬约束。
//
// 挡板是一张位于颜面前方、法线指向脑后的竖直面; 眉毛高度以下才生效,
// 且只约束横向仍在脸宽内的粒子 —— 否则会把两侧的长发也一起吸到脑后。
void XPBDHairNative::_collide_face(int i) {
	if (!face_guard_on) {
		return;
	}
	const Vector3 p = pos[i];
	if (p.y >= face_min_y) {
		return;   // 眉毛以上不挡 —— 刘海本来就该盖在前额上
	}
	const Vector3 rel = p - face_pt;
	// 横向是否在脸宽内: 取脸朝向的水平正交方向
	const Vector3 lat(-face_nrm.z, 0.0f, face_nrm.x);
	const real_t lx = rel.x * lat.x + rel.z * lat.z;
	if (lx * lx > face_guard_r * face_guard_r) {
		return;
	}
	const real_t s = rel.dot(face_nrm);
	if (s >= 0.0f) {
		return;   // 已经在挡板后面(脑后侧)
	}
	// 推到挡板上, 并吃掉朝向脸的速度
	pos.set(i, p - face_nrm * s);
	Vector3 v = vel[i];
	const real_t vn = v.dot(face_nrm);
	if (vn < 0.0f) {
		v -= face_nrm * vn * (1.0f - friction);
		vel.set(i, v);
	}
}

void XPBDHairNative::_collide_head(int i) {
	// 椭球模式: 把点映射到单位球空间, 越界就沿梯度推回去。
	// 比球体贴合得多 —— 前脸/下巴这些"球体让位过多"的地方不会再漏。
	if (head_radii.x > 1e-4f && head_radii.y > 1e-4f && head_radii.z > 1e-4f) {
		const Vector3 d = pos[i] - head_center;
		const Vector3 u = Vector3(d.x / head_radii.x, d.y / head_radii.y, d.z / head_radii.z);
		const real_t ul = u.length();
		// 表面外扩: 用平均半轴把 collide_radius 折算成归一化空间的外扩量
		const real_t mean_r = (head_radii.x + head_radii.y + head_radii.z) / 3.0f;
		const real_t lim = 1.0f + collide_radius / MAX(mean_r, 1e-4f);
		if (ul >= lim) {
			return;
		}
		if (ul < 1e-5f) {
			pos.set(i, head_center + Vector3(0, lim * head_radii.y, 0));
			return;
		}
		const Vector3 uu = u / ul;
		const Vector3 target = head_center + Vector3(uu.x * lim * head_radii.x, uu.y * lim * head_radii.y, uu.z * lim * head_radii.z);
		const Vector3 nrm = (pos[i] - head_center).normalized();
		pos.set(i, target);
		Vector3 v = vel[i];
		const real_t vn = v.dot(nrm);
		if (vn < 0.0f) {
			v -= nrm * vn * (1.0f - friction);
			vel.set(i, v);
		}
		return;
	}
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

	// 渲染发根: 在头皮上再采一批(密度更高), 每个找 K 近邻导向发根。
	// 注意: 早先的写法把 y/n.z 过滤直接叠在定数 total=3n 的循环里, 过滤会丢掉
	// ~50% 采样点(y<0.05 砍掉近半球, n.z<-0.9 砍掉后脑), 于是实际发丝数只有请求的
	// 一半 —— 这正是"头发太稀疏"的根因。现在改为按需扩样, 保证足量。
	std::vector<Vector3> rr_roots;
	rr_roots.reserve(render_strands);
	const real_t golden = 3.14159265359f * (3.0f - std::sqrt(5.0f));

	if (scalp_on && scalp_rest.size() > 0) {
		// 真实头皮模式: 渲染发丝根**必须**落在头皮发根上。
		// 之前这里先按 golden 球面填满了 rr_roots, 于是下面"从头皮发根细分"的
		// while 循环一次都没进 —— 渲染根全落在 head_radius(默认 0.12m)的假想球上,
		// 而真实头只有 ~0.075m, 结果就是大片发丝被吸附到少数几根导向束上,
		// 渲染出来只剩一条窄板(表现为"秃顶")。
		// 做法: 每根导向发根上绕法线错开铺 (render_strands / guides) 根渲染发丝。
		const int ng = MAX((int)roots.size(), 1);
		const real_t g2 = 3.14159265359f * (3.0f - std::sqrt(5.0f));
		int k = 0;
		const int guard = render_strands * 4 + ng * 8;
		while ((int)rr_roots.size() < render_strands && k < guard) {
			const int gi = k % ng;
			const int rot = k / ng;
			const Vector3 n = root_normals[gi];
			// 绕发根法线在切平面里挪一点点, 让同一束上的多根发丝错开
			Vector3 t1 = n.cross(Vector3(0, 1, 0));
			if (t1.length_squared() < 1e-8f) {
				t1 = n.cross(Vector3(1, 0, 0));
			}
			t1 = t1.normalized();
			const Vector3 t2 = n.cross(t1).normalized();
			const real_t a = g2 * (real_t)k + (real_t)rot * 0.7f;
			const real_t rad = (rot == 0) ? 0.0f : 0.004f + 0.003f * (real_t)(rot % 3);
			rr_roots.push_back(roots[gi] + t1 * (std::cos(a) * rad) + t2 * (std::sin(a) * rad));
			k++;
		}
	} else {
		// 独立演示模式: golden 球面采样(过采样 + 过滤, 保证足量)
		const int max_samples = render_strands * 40 + 4096;
		for (int i = 0; i < max_samples && (int)rr_roots.size() < render_strands; i++) {
			const real_t y = 1.0f - (2.0f * (real_t)i + 1.0f) / (real_t)max_samples;
			if (y < 0.05f) {
				continue;
			}
			const real_t rr = std::sqrt(MAX(0.0f, 1.0f - y * y));
			const real_t th = golden * (real_t)i + 0.37f;
			Vector3 n = Vector3(rr * std::cos(th), y, rr * std::sin(th));
			if (n.z < -0.9f) {
				continue;
			}
			rr_roots.push_back(head_center + n * head_radius);
		}
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
	// 插值权重只需构建一次(可按需重建: render_strands 改变时由 setter 清空)
	if (blend_idx.size() == 0) {
		_build_render_strands();
	}
	// 以"实际权重条数"为准, 而不是请求值: 采样过滤可能只落地更少的发丝,
	// 用请求值分配数组会导致索引越界/渲染空条(早先 build() 二次调用崩溃即源于此)。
	const int n_render = blend_k > 0 ? (int)(blend_idx.size() / blend_k) : 0;
	if (n_render <= 0) {
		return;
	}

	const int np = segment_count + 1;
	PackedVector3Array verts;
	PackedVector3Array norms;
	PackedColorArray colors;
	PackedInt32Array indices;
	verts.resize(n_render * np * 2);
	norms.resize(n_render * np * 2);
	colors.resize(n_render * np * 2);

	// 每渲染发丝的横向铺开方向: 用其发根法线与重力混合的叉积(稳定, 不随相机变化)
	for (int r = 0; r < n_render; r++) {
		// 发丝带的横向: 与"发丝主方向(近似重力方向)"叉乘得到的水平向量,
		// 保证发片始终以**最宽的面**朝向观察者(原来用发根法线叉乘, 头顶/后脑会看到薄边)。
		Vector3 lateral = Vector3(0, -1, 0).cross(root_normals[blend_idx[r * blend_k]]);
		if (lateral.length_squared() < 1e-8f) {
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
			Vector3 pv = p;
			// 渲染限位: 头心以下的发丝如果周围没有身体垫着, 说明它悬在半空
			// (物理只做球形头/胶囊身碰撞, 真实头模不是球), 把它推到身体胶囊外侧。
			if (render_min_y > -0.5f && pv.y < render_min_y && body_on) {
				const Vector3 near = cloth_internal::closest_on_seg(body_a, body_b, pv);
				Vector3 d = pv - near;
				d.y = 0.0f; // 只在水平面上推, 不改变高度
				const real_t rr = body_r + 0.012f;
				const real_t dl = d.length();
				if (dl < 1e-6f) {
					d = Vector3(0, 0, 1);
				} else {
					d = d / dl;
				}
				const real_t drop = render_min_y - pv.y;
				if (drop > render_snap) {
					pv.y = render_min_y; // 太长: 整体下移到下限(表现为发梢垂下来)
				}
				if (dl < rr) {
					pv.x = near.x + d.x * rr;
					pv.z = near.z + d.z * rr;
				}
			}
			verts.set(vi, pv - lateral * half_w);
			verts.set(vi + 1, pv + lateral * half_w);
			const Color col = root_color.lerp(tip_color, t);
			colors.set(vi, col);
			colors.set(vi + 1, col);
			// 法线: 发片是"带子", 面法线 = 切向 × 横向。
			// 不写 NORMAL 的话 Godot 拿到的是零法线 —— 发丝会变成纯环境光的死黑
			// (这正是之前头发渲染成"深蓝黑"的原因)。
			Vector3 tangent = (k + 1 < np) ? (pos[blend_idx[r * blend_k] * np + k + 1] - pv) : (pv - pos[blend_idx[r * blend_k] * np + k - 1]);
			if (tangent.length_squared() < 1e-12f) {
				tangent = Vector3(0, -1, 0);
			}
			Vector3 nrm = lateral.cross(tangent.normalized());
			if (nrm.length_squared() < 1e-10f) {
				nrm = Vector3(0, 0, 1);
			}
			nrm = nrm.normalized();
			norms.set(vi, nrm);
			norms.set(vi + 1, nrm);
		}
	}

	for (int r = 0; r < n_render; r++) {
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
	arrays[Mesh::ARRAY_NORMAL] = norms;
	arrays[Mesh::ARRAY_COLOR] = colors;
	arrays[Mesh::ARRAY_INDEX] = indices;
	m->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	if (material.is_valid()) {
		m->surface_set_material(0, material);
	}
	mesh = m;
}
