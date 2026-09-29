#include "xpbd_cloth_native.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace godot;

// ============================================================================
// 静态布间注册表: 避免 GDScript 侧每帧遍历/哈希全部顶点
// ============================================================================
static std::vector<XPBDClothNative *> g_cloths;

void XPBDClothNative::register_cloth(XPBDClothNative *c) {
	for (size_t i = 0; i < g_cloths.size(); i++) {
		if (g_cloths[i] == c) {
			return;
		}
	}
	g_cloths.push_back(c);
}

void XPBDClothNative::unregister_cloth(XPBDClothNative *c) {
	for (size_t i = 0; i < g_cloths.size(); i++) {
		if (g_cloths[i] == c) {
			g_cloths.erase(g_cloths.begin() + i);
			return;
		}
	}
}

// ============================================================================
// 空间哈希(平坦数组 + 排序键): C++ 下 O(n) 构建, 比 Dictionary 快一个数量级
// 用于自碰与布间的邻域查询
// ============================================================================
namespace {

struct CellGrid {
	std::vector<int64_t> keys;   // 每顶点的格 key(pack 后)
	std::vector<int32_t> order;  // 按 key 排序后的顶点索引
	std::vector<int64_t> ukey;   // 去重后的 key(升序)
	std::vector<int32_t> ubeg;   // 每个唯一 key 在 order 中的起始下标

	void build(const PackedVector3Array &pos, real_t inv, int span) {
		const int n = (int)pos.size();
		keys.resize(n);
		order.resize(n);
		for (int i = 0; i < n; i++) {
			const Vector3 &p = pos[i];
			int x = (int)std::floor(p.x * inv) + 512;
			int y = (int)std::floor(p.y * inv) + 512;
			int z = (int)std::floor(p.z * inv) + 512;
			// 夹紧到 span 位以避免 key 溢出
			x = x < 0 ? 0 : (x > span - 1 ? span - 1 : x);
			y = y < 0 ? 0 : (y > span - 1 ? span - 1 : y);
			z = z < 0 ? 0 : (z > span - 1 ? span - 1 : z);
			keys[i] = ((int64_t)x << 42) | ((int64_t)y << 21) | (int64_t)z;
			order[i] = i;
		}
		std::sort(order.begin(), order.end(), [&](int32_t a, int32_t b) {
			return keys[a] < keys[b];
		});
		ukey.clear();
		ubeg.clear();
		for (int i = 0; i < n; i++) {
			int64_t k = keys[order[i]];
			if (ukey.empty() || ukey.back() != k) {
				ukey.push_back(k);
				ubeg.push_back(i);
			}
		}
		ubeg.push_back(n);
	}

	// 返回 key 对应的 [begin,end) 区间; 未命中返回 false
	bool find(int64_t k, int &b, int &e) const {
		int lo = 0, hi = (int)ukey.size();
		while (lo < hi) {
			int mid = (lo + hi) >> 1;
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

// 27 邻域偏移(必须含中心格 (0,0,0))
const int kOffs[27][3] = {
	{ 0, 0, 0 }, { -1, -1, -1 }, { -1, -1, 0 }, { -1, -1, 1 }, { -1, 0, -1 }, { -1, 0, 0 }, { -1, 0, 1 },
	{ -1, 1, -1 }, { -1, 1, 0 }, { -1, 1, 1 }, { 0, -1, -1 }, { 0, -1, 0 }, { 0, -1, 1 }, { 0, 0, -1 },
	{ 0, 0, 1 }, { 0, 1, -1 }, { 0, 1, 0 }, { 0, 1, 1 }, { 1, -1, -1 }, { 1, -1, 0 }, { 1, -1, 1 },
	{ 1, 0, -1 }, { 1, 0, 0 }, { 1, 0, 1 }, { 1, 1, -1 }, { 1, 1, 0 }, { 1, 1, 1 }
};

inline int64_t pack_cell(int x, int y, int z) {
	return ((int64_t)x << 42) | ((int64_t)y << 21) | (int64_t)z;
}

} // namespace

// ============================================================================
// 构造/析构
// ============================================================================
XPBDClothNative::XPBDClothNative() {}
XPBDClothNative::~XPBDClothNative() { unregister_cloth(this); }

void XPBDClothNative::_bind_methods() {
	ClassDB::bind_method(D_METHOD("build"), &XPBDClothNative::build);
	ClassDB::bind_method(D_METHOD("set_collider_shapes", "pt", "poff", "pa", "pb", "pr", "phalf", "xform"), &XPBDClothNative::set_collider_shapes);
	ClassDB::bind_method(D_METHOD("sub_begin"), &XPBDClothNative::sub_begin);
	ClassDB::bind_method(D_METHOD("begin_contacts"), &XPBDClothNative::begin_contacts);
	ClassDB::bind_method(D_METHOD("sub_integrate", "h", "wind"), &XPBDClothNative::sub_integrate);
	ClassDB::bind_method(D_METHOD("set_wind_v", "v"), &XPBDClothNative::set_wind_v);
	ClassDB::bind_method(D_METHOD("get_wind_v"), &XPBDClothNative::get_wind_v);
	ClassDB::bind_method(D_METHOD("translate_model", "delta"), &XPBDClothNative::translate_model);
	ClassDB::bind_method(D_METHOD("set_body_capsule", "on", "a", "b", "r"), &XPBDClothNative::set_body_capsule);
	ClassDB::bind_method(D_METHOD("collide_body_capsule"), &XPBDClothNative::collide_body_capsule);
	ClassDB::bind_method(D_METHOD("sub_solve", "h"), &XPBDClothNative::sub_solve);
	ClassDB::bind_method(D_METHOD("self_collision"), &XPBDClothNative::self_collision);
	ClassDB::bind_method(D_METHOD("object_collision"), &XPBDClothNative::object_collision);
	ClassDB::bind_method(D_METHOD("final_push_out"), &XPBDClothNative::final_push_out);
	ClassDB::bind_method(D_METHOD("sub_recover", "h"), &XPBDClothNative::sub_recover);
	ClassDB::bind_method(D_METHOD("get_positions"), &XPBDClothNative::get_positions);
	ClassDB::bind_method(D_METHOD("set_positions", "p"), &XPBDClothNative::set_positions);
	ClassDB::bind_method(D_METHOD("get_velocities"), &XPBDClothNative::get_velocities);
	ClassDB::bind_method(D_METHOD("set_velocities", "v"), &XPBDClothNative::set_velocities);
	ClassDB::bind_method(D_METHOD("get_pinned_flags"), &XPBDClothNative::get_pinned_flags);
	ClassDB::bind_method(D_METHOD("set_pinned_flags", "v"), &XPBDClothNative::set_pinned_flags);
	ClassDB::bind_method(D_METHOD("penetration_count"), &XPBDClothNative::penetration_count);
	ClassDB::bind_method(D_METHOD("self_deficit"), &XPBDClothNative::self_deficit);
	ClassDB::bind_method(D_METHOD("pick_vertex", "ro", "rn"), &XPBDClothNative::pick_vertex);
	ClassDB::bind_method(D_METHOD("release_grab", "substeps"), &XPBDClothNative::release_grab);
	ClassDB::bind_method(D_METHOD("update_mesh"), &XPBDClothNative::update_mesh);
	ClassDB::bind_method(D_METHOD("cross_contact_all", "push"), &XPBDClothNative::cross_contact_all);
	ClassDB::bind_method(D_METHOD("cross_deficit_all"), &XPBDClothNative::cross_deficit_all);
	ClassDB::bind_method(D_METHOD("registry_size"), &XPBDClothNative::registry_size);
	ClassDB::bind_method(D_METHOD("cross_debug"), &XPBDClothNative::cross_debug);
	ClassDB::bind_method(D_METHOD("get_contact_deficit"), &XPBDClothNative::get_contact_deficit);
	ClassDB::bind_method(D_METHOD("set_contact_deficit", "v"), &XPBDClothNative::set_contact_deficit);
	ClassDB::bind_method(D_METHOD("get_max_vel2"), &XPBDClothNative::get_max_vel2);
	ClassDB::bind_method(D_METHOD("set_max_vel2", "v"), &XPBDClothNative::set_max_vel2);
	ClassDB::bind_method(D_METHOD("get_asleep"), &XPBDClothNative::get_asleep);
	ClassDB::bind_method(D_METHOD("get_tri_r"), &XPBDClothNative::get_tri_r);
	ClassDB::bind_method(D_METHOD("get_constraint_count"), &XPBDClothNative::get_constraint_count);
	ClassDB::bind_method(D_METHOD("get_torn_count"), &XPBDClothNative::get_torn_count);
	ClassDB::bind_method(D_METHOD("get_triangle_count"), &XPBDClothNative::get_triangle_count);
	ClassDB::bind_method(D_METHOD("get_tri_verts"), &XPBDClothNative::get_tri_verts);
	ClassDB::bind_method(D_METHOD("get_constraint_ia"), &XPBDClothNative::get_constraint_ia);
	ClassDB::bind_method(D_METHOD("get_constraint_ib"), &XPBDClothNative::get_constraint_ib);
	ClassDB::bind_method(D_METHOD("get_constraint_rest"), &XPBDClothNative::get_constraint_rest);
	ClassDB::bind_method(D_METHOD("get_constraint_type"), &XPBDClothNative::get_constraint_type);
	ClassDB::bind_method(D_METHOD("get_vtx_tri_ofs"), &XPBDClothNative::get_vtx_tri_ofs);
	ClassDB::bind_method(D_METHOD("get_vtx_tri_ids"), &XPBDClothNative::get_vtx_tri_ids);
	ClassDB::bind_method(D_METHOD("get_constraint_alive"), &XPBDClothNative::get_constraint_alive);
	ClassDB::bind_method(D_METHOD("get_mesh"), &XPBDClothNative::get_mesh);
	ClassDB::bind_method(D_METHOD("set_material", "m"), &XPBDClothNative::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &XPBDClothNative::get_material);

	// 属性(便于 GDScript 侧直接读写)。类型必须与 getter/setter 一致, 否则 create_method_bind 编译失败。
#define BIND_P(name, getter, setter, vtype) \
	ClassDB::bind_method(D_METHOD("set_" #name, "v"), &XPBDClothNative::setter); \
	ClassDB::bind_method(D_METHOD("get_" #name), &XPBDClothNative::getter); \
	ADD_PROPERTY(PropertyInfo(vtype, #name), "set_" #name, "get_" #name);
	BIND_P(cols, get_cols, set_cols, Variant::INT)
	BIND_P(rows, get_rows, set_rows, Variant::INT)
	BIND_P(vertical, get_vertical, set_vertical, Variant::BOOL)
	BIND_P(yaw, get_yaw, set_yaw, Variant::FLOAT)
	BIND_P(gravity, get_gravity, set_gravity, Variant::VECTOR3)
	BIND_P(damping, get_damping, set_damping, Variant::FLOAT)
	BIND_P(air_drag, get_air_drag, set_air_drag, Variant::FLOAT)
	BIND_P(max_stretch, get_max_stretch, set_max_stretch, Variant::FLOAT)
	BIND_P(compliance_struct, get_compliance_struct, set_compliance_struct, Variant::FLOAT)
	BIND_P(compliance_shear, get_compliance_shear, set_compliance_shear, Variant::FLOAT)
	BIND_P(compliance_bend, get_compliance_bend, set_compliance_bend, Variant::FLOAT)
	BIND_P(cloth_thickness, get_cloth_thickness, set_cloth_thickness, Variant::FLOAT)
	BIND_P(friction, get_friction, set_friction, Variant::FLOAT)
	BIND_P(enable_object_collision, get_enable_object_collision, set_enable_object_collision, Variant::BOOL)
	BIND_P(enable_self_collision, get_enable_self_collision, set_enable_self_collision, Variant::BOOL)
	BIND_P(ground_y, get_ground_y, set_ground_y, Variant::FLOAT)
	BIND_P(iterations, get_iterations, set_iterations, Variant::INT)
	BIND_P(grab_stiffness, get_grab_stiffness, set_grab_stiffness, Variant::FLOAT)
	BIND_P(grabbing, get_grabbing, set_grabbing, Variant::BOOL)
	BIND_P(grab_id, get_grab_id, set_grab_id, Variant::INT)
	BIND_P(grab_target, get_grab_target, set_grab_target, Variant::VECTOR3)
	BIND_P(spawn_origin, get_spawn_origin, set_spawn_origin, Variant::VECTOR3)
	BIND_P(cloth_size, get_cloth_size, set_cloth_size, Variant::VECTOR2)
	BIND_P(albedo, get_albedo, set_albedo, Variant::COLOR)
#undef BIND_P
}

PackedInt32Array XPBDClothNative::get_vtx_tri_ofs() const { return vtx_tri_ofs; }
PackedInt32Array XPBDClothNative::get_vtx_tri_ids() const { return vtx_tri_ids; }

int XPBDClothNative::get_torn_count() const {
	int n = 0;
	for (int i = 0; i < c_alive.size(); i++) {
		if (c_alive[i] == 0) {
			n++;
		}
	}
	return n;
}

// ============================================================================
// 构建: 粒子 + 约束 + 静态拓扑
// ============================================================================
void XPBDClothNative::_add_constraint(int ia, int ib, int ctype) {
	c_ia.push_back(ia);
	c_ib.push_back(ib);
	c_rest.push_back(pos[ia].distance_to(pos[ib]));
	c_alive.push_back(1);
	c_type.push_back(ctype);
}

// 模型平移(拖拽跟随): 所有粒子(含钉住的)平移 delta, 并给非钉住粒子继承速度产生摆动
void XPBDClothNative::translate_model(const Vector3 &delta) {
	if (delta.length_squared() < 1e-12f) {
		return;
	}
	for (int i = 0; i < pos.size(); i++) {
		pos.set(i, pos[i] + delta);
	}
	// 碰撞胶囊必须跟着模型走, 否则拖拽后胶囊留在原地 -> 碰撞完全错位(裙布穿身)
	body_seg_a += delta;
	body_seg_b += delta;
	// 速度继承: 让布料有拖拽惯性(向下折算, 避免过度摆动)
	const Vector3 inherit = delta * 20.0f;
	for (int i = 0; i < pos.size(); i++) {
		if (pinned[i] != 1) {
			vel.set(i, vel[i] + inherit);
		}
	}
}

void XPBDClothNative::build() {
	c_ia.clear(); c_ib.clear(); c_rest.clear(); c_alive.clear(); c_type.clear();
	pinned.clear();
	pos.clear(); prev.clear(); vel.clear();

	const Basis rot = Basis::from_euler(Vector3(0, yaw, 0));
	const real_t dx = cloth_size.x / (real_t)(cols - 1);
	const real_t dy = cloth_size.y / (real_t)(rows - 1);
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			const real_t x = ((real_t)c / (real_t)(cols - 1) - 0.5f) * cloth_size.x;
			const real_t j = (real_t)((r * 13 + c * 7) % 10) * 0.002f;
			Vector3 p;
			if (vertical) {
				p = Vector3(x, -(real_t)r * dy + j * 0.5f, 0.0f);
			} else {
				const real_t z = ((real_t)r / (real_t)(rows - 1) - 0.5f) * cloth_size.y;
				p = Vector3(x, j, z);
			}
			p = spawn_origin + rot.xform(p);
			pos.push_back(p);
			prev.push_back(p);
			vel.push_back(Vector3());
			pinned.push_back((vertical && r == 0) ? 1 : 0);
		}
	}

	auto idx = [&](int c, int r) { return r * cols + c; };
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			const int i = idx(c, r);
			if (c + 1 < cols) { _add_constraint(i, idx(c + 1, r), 0); }
			if (r + 1 < rows) { _add_constraint(i, idx(c, r + 1), 0); }
			if (c + 1 < cols && r + 1 < rows) {
				_add_constraint(i, idx(c + 1, r + 1), 1);
				_add_constraint(idx(c + 1, r), idx(c, r + 1), 1);
			}
			if (c + 2 < cols) { _add_constraint(i, idx(c + 2, r), 2); }
			if (r + 2 < rows) { _add_constraint(i, idx(c, r + 2), 2); }
			if (c + 2 < cols && r + 2 < rows) {
				_add_constraint(i, idx(c + 2, r + 2), 2);
				_add_constraint(idx(c + 2, r), idx(c, r + 2), 2);
			}
		}
	}

	_build_topology();

	pushed_hard.resize(pos.size());
	pushed_hard.fill(0);
	pushed_soft.resize(pos.size());
	pushed_soft.fill(0);
	contact_deficit = 0.0;
	max_vel2 = 0.0;
	asleep = false;
	sleep_checks = 0;
	sleep_frame = 0;

	// 自动加入跨布注册表(析构时移除): 布间互碰由 C++ 统一处理,
	// 之前遗漏导致 g_cloths 为空 -> 布间碰撞从未生效(层间距仅 4mm)
	register_cloth(this);

	// 默认材质(与 GDScript 一致: 双面 + 粗糙), 颜色用 albedo
	if (material.is_null()) {
		Ref<StandardMaterial3D> m;
		m.instantiate();
		m->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
		m->set_roughness(0.7f);
		m->set_albedo(albedo);
		material = m;
	} else {
		Ref<StandardMaterial3D> sm = material;
		if (sm.is_valid()) {
			sm->set_albedo(albedo);
		}
	}
}

// 顶点->关联三角形 CSR 表 + 三角形外接圆半径上界(几何补偿用)
void XPBDClothNative::_build_topology() {
	tri_verts.clear();
	vtx_tri_ofs.clear();
	vtx_tri_ids.clear();
	tri_r = 0.0;

	std::vector<int> counts(pos.size(), 0);
	for (int r = 0; r < rows - 1; r++) {
		for (int c = 0; c < cols - 1; c++) {
			const int a0 = r * cols + c;
			const int tris[2][3] = {
				{ a0, a0 + cols, a0 + 1 },
				{ a0 + 1, a0 + cols, a0 + cols + 1 }
			};
			for (int t = 0; t < 2; t++) {
				for (int k = 0; k < 3; k++) {
					tri_verts.push_back(tris[t][k]);
					counts[tris[t][k]]++;
				}
				const Vector3 &p0 = pos[tris[t][0]];
				const Vector3 &p1 = pos[tris[t][1]];
				const Vector3 &p2 = pos[tris[t][2]];
				const real_t ea = p0.distance_to(p1);
				const real_t eb = p1.distance_to(p2);
				const real_t ec = p2.distance_to(p0);
				const real_t area2 = (p1 - p0).cross(p2 - p0).length();
				if (area2 > 1e-12f) {
					const real_t R = ea * eb * ec / (2.0f * area2);
					tri_r = MAX(tri_r, R);
				}
			}
		}
	}

	vtx_tri_ofs.resize(pos.size() + 1);
	int acc = 0;
	for (int i = 0; i < (int)pos.size(); i++) {
		vtx_tri_ofs[i] = acc;
		acc += counts[i];
	}
	vtx_tri_ofs[pos.size()] = acc;
	vtx_tri_ids.resize(acc);
	std::vector<int> cursor(pos.size(), 0);
	const int nt = tri_verts.size() / 3;
	for (int t = 0; t < nt; t++) {
		for (int k = 0; k < 3; k++) {
			const int v = tri_verts[t * 3 + k];
			vtx_tri_ids[vtx_tri_ofs[v] + cursor[v]] = t;
			cursor[v]++;
		}
	}
}

// ============================================================================
// 子步阶段 1: 抓取 + 积分
// ============================================================================
void XPBDClothNative::sub_begin() {
	// 抓取限速: 手速再快也不能瞬移布料(撕裂/极限拉扯的来源)
	if (grabbing && grab_id >= 0 && grab_id < (int)pos.size()) {
		const Vector3 to_t = grab_target - pos[grab_id];
		const real_t ml = to_t.length();
		if (ml > 1e-9f) {
			const real_t step_max = grab_max_speed / 60.0f / 4.0f;
			const real_t k = CLAMP(grab_stiffness, 0.05f, 1.0f);
			if (ml > step_max) {
				pos.set(grab_id, pos[grab_id] + to_t * (step_max / ml) * k);
			} else {
				pos.set(grab_id, pos[grab_id].lerp(grab_target, k));
			}
		}
	}
}

void XPBDClothNative::sub_integrate(real_t h, const Vector3 &wind) {
	// 成员 wind = 环境风(每子步恒定作用), 参数 wind = 阵风等瞬时叠加
	const Vector3 w_total = this->wind + wind;
	const int n = pos.size();
	real_t mv2 = 0.0;
	for (int i = 0; i < n; i++) {
		if (pinned[i] == 1) {
			continue;
		}
		prev.set(i, pos[i]);
		Vector3 v = vel[i] + gravity * h + w_total * h;
		v *= (1.0f - damping);
		v -= v * v.length() * air_drag * h;
		vel.set(i, v);
		const real_t v2 = v.length_squared();
		if (v2 > mv2) {
			mv2 = v2;
		}
		pos.set(i, pos[i] + v * h);
	}
	max_vel2 = mv2;
}

// ============================================================================
// 子步阶段 2: 约束求解 + 应变限制
// 优化: 应变限制维护"活跃计数", 全不活跃时零成本跳过(静置态实测 0/4186 生效)
// ============================================================================
void XPBDClothNative::_strain_limit() {
	const int nc = c_ia.size();
	const real_t limit_k = max_stretch;
	int active = 0;
	for (int k = 0; k < nc; k++) {
		if (c_alive[k] == 0 || c_type[k] == 2) {
			continue;
		}
		const int ia = c_ia[k];
		const int ib = c_ib[k];
		const real_t limit = c_rest[k] * limit_k;
		const Vector3 d = pos[ia] - pos[ib];
		const real_t ln = d.length();
		if (ln <= limit) {
			continue;
		}
		active++;
		const real_t wa = (pinned[ia] == 1 || ia == grab_id) ? 0.0f : 1.0f;
		const real_t wb = (pinned[ib] == 1 || ib == grab_id) ? 0.0f : 1.0f;
		const real_t w = wa + wb;
		if (w == 0.0f) {
			continue;
		}
		const real_t lam = (ln - limit) / ln / w;
		pos.set(ia, pos[ia] - d * lam * wa);
		pos.set(ib, pos[ib] + d * lam * wb);
	}
	strain_active = active;
}

void XPBDClothNative::sub_solve(real_t h) {
	const int nc = c_ia.size();
	const real_t inv_h2 = 1.0f / (h * h);
	const real_t alphas[3] = { compliance_struct, compliance_shear, compliance_bend };

	for (int it = 0; it < iterations; it++) {
		for (int k = 0; k < nc; k++) {
			if (c_alive[k] == 0) {
				continue;
			}
			const int ia = c_ia[k];
			const int ib = c_ib[k];
			const real_t rest = c_rest[k];
			const Vector3 d = pos[ia] - pos[ib];
			const real_t ln = d.length();
			if (ln < 1e-6f) {
				continue;
			}
			// 撕裂: 仅在 tear_dist_factor > 0 时启用(默认关闭, 与 GDScript 一致)
			if (tear_dist_factor > 0.0f && ln > rest * tear_dist_factor) {
				c_alive.set(k, 0);
				continue;
			}
			const real_t wa = (pinned[ia] == 1 || ia == grab_id) ? 0.0f : 1.0f;
			const real_t wb = (pinned[ib] == 1 || ib == grab_id) ? 0.0f : 1.0f;
			const real_t w = wa + wb;
			if (w == 0.0f) {
				continue;
			}
			const real_t lam = -(ln - rest) / (ln * (w + alphas[c_type[k]] * inv_h2));
			pos.set(ia, pos[ia] + d * lam * wa);
			pos.set(ib, pos[ib] - d * lam * wb);
		}
	}
	_strain_limit();
}

// ============================================================================
// 子步阶段 6: 速度恢复
// 接触修正不产生"发射速度": 硬接触顶点取修正前位移并用推出法向做投影;
// 软接触(自碰/布间)保留分离动量(否则折皱无法展开)。
// ============================================================================
void XPBDClothNative::sub_recover(real_t h) {
	const int n = pos.size();
	const real_t inv_h = 1.0f / h;
	for (int i = 0; i < n; i++) {
		if (pinned[i] == 1 || i == grab_id) {
			continue;
		}
		Vector3 v = (pos[i] - prev[i]) * inv_h;
		if (pushed_hard[i] == 1 && pushed_soft[i] == 0) {
			// 去掉"推出造成的法向外弹分量"(只删 >0 的, 保留滑移/摩擦)
			const Vector3 corr = pos[i] - pre_contact[i];
			const real_t c2 = corr.length_squared();
			if (c2 > 1e-12f) {
				const Vector3 nrm = corr / std::sqrt(c2);
				const real_t vn = v.dot(nrm);
				if (vn > 0.0f) {
					v -= nrm * vn;
				}
			}
		}
		vel.set(i, v);
	}
}

void XPBDClothNative::begin_contacts() {
	pre_contact = pos.duplicate();
	pushed_hard.fill(0);
	pushed_soft.fill(0);
}

// 身体胶囊碰撞: 把所有粒子推出胶囊表面(裙布/衣服不穿进身体)。
// 软接触语义: 修正不产生发射速度(同自碰), 保留切向滑移。
void XPBDClothNative::collide_body_capsule() {
	if (!body_collision_on) {
		return;
	}
	const Vector3 ab = body_seg_b - body_seg_a;
	const real_t len2 = ab.length_squared();
	const real_t rr = body_seg_r + 0.008f; // + 布料半厚
	const real_t rr2 = rr * rr;
	for (int i = 0; i < pos.size(); i++) {
		const Vector3 &p = pos[i];
		// 胶囊 = 圆柱段 + 两端半球。t 必须 clamp(而非跳过), 否则端帽处无保护:
		// 裙布正好垂在臀部下方, 跳过端帽会让裙布切进臀部/大腿。
		const real_t raw = (p - body_seg_a).dot(ab) / len2;
		const real_t t = raw < 0.0f ? 0.0f : (raw > 1.0f ? 1.0f : raw);
		const Vector3 nearPt = body_seg_a + ab * t;
		const Vector3 d = p - nearPt;
		const real_t d2 = d.length_squared();
		if (d2 >= rr2) {
			continue;
		}
		const real_t ln = std::sqrt(MAX(d2, 1e-12f));
		Vector3 nrm = ln > 1e-9f ? d / ln : Vector3(0, 0, 1);
		pos.set(i, nearPt + nrm * rr);
		// 法向速度清零(向内部分), 保留切向
		Vector3 v = vel[i];
		const real_t vn = v.dot(nrm);
		if (vn < 0.0f) {
			v -= nrm * vn;
			vel.set(i, v);
		}
	}
}


// ============================================================================
// 自碰撞
// 算法流程优化(保持质量):
//  1) 空间哈希只构建 1 次/次调用, 三轮松弛复用同一份接触对 —— 原实现每轮重建哈希
//     (实测 6 轮里 5 轮纯浪费, 各轮推挤数恒为 40)
//  2) 第一遍扫描同时收集"候选接触对"(距离 < 阈值), 后续只在这份较短列表上松弛,
//     避免每轮重复做 27 邻域查找
//  3) 法向速度阻尼: 折层在分离距离附近弹跳会永不静止(实测长期 0.7m/s, 布料无法休眠)
//  4) 休眠: 连续 3 次干净且几乎无运动 -> 跳过整个哈希构建
// ============================================================================
void XPBDClothNative::self_collision() {
	if (!enable_self_collision) {
		return;
	}
	// 休眠判据与求解器收敛门限一致(0.008): 更严会导致永不休眠 -> 每帧全量重建
	if (asleep) {
		if (max_vel2 < 0.04f) {
			sleep_frame++;
			if (sleep_frame < 8) {
				return;
			}
			sleep_frame = 0;
			if (_self_deficit() < 0.008f) {
				return;
			}
		}
		asleep = false;
		sleep_checks = 0;
	} else if (contact_deficit < 0.008f && max_vel2 < 0.04f) {
		sleep_checks++;
		if (sleep_checks >= 3) {
			asleep = true;
			sleep_frame = 0;
			return;
		}
	} else {
		sleep_checks = 0;
	}

	const int n = pos.size();
	// 几何补偿: 顶点扎进对侧折层三角形时顶点间距仍可 >= 2*布厚, 采样半径 = S_visual + R
	const real_t min_d = cloth_thickness * 0.5f + tri_r;
	const real_t min_d2 = min_d * min_d;
	const real_t inv = 1.0f / min_d;

	// --- 1) 构建哈希(仅一次) ---
	CellGrid grid;
	grid.build(pos, inv, 1024);

	// --- 2) 收集候选对(带网格邻接掩码), 仅一次 ---
	cand_a.clear();
	cand_b.clear();
	for (int i = 0; i < n; i++) {
		const Vector3 p = pos[i];
		const int cx = (int)std::floor(p.x * inv) + 512;
		const int cy = (int)std::floor(p.y * inv) + 512;
		const int cz = (int)std::floor(p.z * inv) + 512;
		const int ri = i / cols;
		const int ci = i % cols;
		for (int o = 0; o < 27; o++) {
			int b, e;
			if (!grid.find(pack_cell(cx + kOffs[o][0], cy + kOffs[o][1], cz + kOffs[o][2]), b, e)) {
				continue;
			}
			for (int t = b; t < e; t++) {
				const int j = grid.order[t];
				if (j <= i) {
					continue;
				}
				// 网格邻域内的顶点对有约束相连, 跳过; 折叠层(dr>2)必须求解
				const int dr = std::abs(j / cols - ri);
				const int dc = std::abs(j % cols - ci);
				if (dr <= 2 && dc <= 2) {
					continue;
				}
				if (p.distance_squared_to(pos[j]) >= min_d2) {
					continue;
				}
				cand_a.push_back(i);
				cand_b.push_back(j);
			}
		}
	}

	// --- 3) 在候选列表上松弛(不需要重建哈希) ---
	// 分离量小、候选对通常几十个, 多轮代价可忽略; 上限 6 轮与原实现等价
	real_t worst = 0.0f;
	const int cand_n = cand_a.size();
	for (int sweep = 0; sweep < 6; sweep++) {
		worst = 0.0f;
		bool pushed = false;
		for (int q = 0; q < cand_n; q++) {
			const int i = cand_a[q];
			const int j = cand_b[q];
			const Vector3 d = pos[i] - pos[j];
			const real_t d2 = d.length_squared();
			if (d2 >= min_d2 || d2 < 1e-12f) {
				continue;
			}
			const real_t ln = std::sqrt(d2);
			const real_t deficit = min_d - ln;
			if (deficit > worst) {
				worst = deficit;
			}
			const real_t wa = (pinned[i] == 1 || i == grab_id) ? 0.0f : 1.0f;
			const real_t wb = (pinned[j] == 1 || j == grab_id) ? 0.0f : 1.0f;
			const real_t w = wa + wb;
			if (w == 0.0f) {
				continue;
			}
			const Vector3 nrm = d / ln;
			const real_t push = deficit / w;
			pos.set(i, pos[i] + nrm * push * wa);
			pos.set(j, pos[j] - nrm * push * wb);
			// 法向速度阻尼: 耗散接近分量, 让折层能真正静止
			const Vector3 vrel = vel[i] - vel[j];
			const real_t vn = vrel.dot(nrm);
			if (vn < 0.0f) {
				const Vector3 dv = nrm * (vn * 0.5f);
				if (wa > 0.0f) {
					vel.set(i, vel[i] - dv * wa);
				}
				if (wb > 0.0f) {
					vel.set(j, vel[j] + dv * wb);
				}
			}
			pushed = true;
			pushed_soft.set(i, 1);
			pushed_soft.set(j, 1);
		}
		// 本轮无推挤(已全部分离) 或 残差已小于收敛门限 -> 收工
		if (!pushed || worst < 0.005f) {
			break;
		}
	}
	contact_deficit = worst;
}


// ============================================================================
// 物体碰撞: 局部空间解析 TOI 滑移 + 最小推出兜底
// ============================================================================
void XPBDClothNative::set_collider_shapes(
		const PackedInt32Array &pt, const PackedVector3Array &poff,
		const PackedVector3Array &pa, const PackedVector3Array &pb,
		const PackedFloat32Array &pr, const PackedVector3Array &phalf,
		const Transform3D &xform) {
	col_pt = pt;
	col_poff = poff;
	col_pa = pa;
	col_pb = pb;
	col_pr = pr;
	col_phalf = phalf;
	col_xform = xform;
	col_xform_inv = xform.affine_inverse();
	col_broad_r = 0.0f;
	for (int k = 0; k < col_pt.size(); k++) {
		real_t r = 0.0f;
		if (col_pt[k] == SHAPE_BOX) {
			r = col_poff[k].length() + col_phalf[k].length();
		} else if (col_pt[k] == SHAPE_CAPSULE) {
			r = MAX(col_pa[k].length(), col_pb[k].length()) + col_pr[k];
		} else {
			r = col_poff[k].length() + col_pr[k];
		}
		col_broad_r = MAX(col_broad_r, r);
	}
}

// 点到线段最近点
static inline Vector3 closest_on_seg(const Vector3 &a, const Vector3 &b, const Vector3 &p) {
	const Vector3 ab = b - a;
	const real_t len2 = ab.length_squared();
	const real_t t = len2 > 1e-12f ? CLAMP((p - a).dot(ab) / len2, 0.0f, 1.0f) : 0.0f;
	return a + ab * t;
}

// 箱内最小平移法向
static inline Vector3 box_mtp_normal(const Vector3 &l, const Vector3 &h) {
	const real_t dx = h.x - std::abs(l.x);
	const real_t dy = h.y - std::abs(l.y);
	const real_t dz = h.z - std::abs(l.z);
	Vector3 n;
	if (dx <= dy && dx <= dz) {
		n.x = std::abs(l.x) > 1e-9f ? (l.x > 0 ? 1.0f : -1.0f) : 1.0f;
	} else if (dy <= dz) {
		n.y = std::abs(l.y) > 1e-9f ? (l.y > 0 ? 1.0f : -1.0f) : 1.0f;
	} else {
		n.z = std::abs(l.z) > 1e-9f ? (l.z > 0 ? 1.0f : -1.0f) : 1.0f;
	}
	return n;
}

bool XPBDClothNative::_inside_shape(int k, const Vector3 &p) const {
	const real_t prp = col_pr[k] + pad;
	if (col_pt[k] == SHAPE_BOX) {
		const Vector3 l = p - col_poff[k];
		const Vector3 h = col_phalf[k] + Vector3(pad, pad, pad);
		return std::abs(l.x) <= h.x && std::abs(l.y) <= h.y && std::abs(l.z) <= h.z;
	}
	const Vector3 cen = (col_pt[k] == SHAPE_SPHERE) ? col_poff[k] : closest_on_seg(col_pa[k], col_pb[k], p);
	return (p - cen).length_squared() < prp * prp;
}

Vector3 XPBDClothNative::_push_shape(int k, const Vector3 &p) const {
	if (col_pt[k] == SHAPE_BOX) {
		const Vector3 l = p - col_poff[k];
		const Vector3 h = col_phalf[k] + Vector3(pad, pad, pad);
		if (std::abs(l.x) <= h.x && std::abs(l.y) <= h.y && std::abs(l.z) <= h.z) {
			const real_t dx = h.x - std::abs(l.x);
			const real_t dy = h.y - std::abs(l.y);
			const real_t dz = h.z - std::abs(l.z);
			Vector3 v = l;
			if (dx <= dy && dx <= dz) {
				v.x = (l.x != 0.0f ? (l.x > 0 ? 1.0f : -1.0f) : 1.0f) * h.x;
			} else if (dy <= dz) {
				v.y = (l.y != 0.0f ? (l.y > 0 ? 1.0f : -1.0f) : 1.0f) * h.y;
			} else {
				v.z = (l.z != 0.0f ? (l.z > 0 ? 1.0f : -1.0f) : 1.0f) * h.z;
			}
			return col_poff[k] + v;
		}
		return p;
	}
	const Vector3 cen = (col_pt[k] == SHAPE_SPHERE) ? col_poff[k] : closest_on_seg(col_pa[k], col_pb[k], p);
	const Vector3 d = p - cen;
	const real_t rr = col_pr[k] + pad;
	const real_t ln2 = d.length_squared();
	if (ln2 >= rr * rr) {
		return p;
	}
	if (ln2 < 1e-12f) {
		return cen + Vector3(0, 1, 0) * rr;
	}
	return cen + d * (rr / std::sqrt(ln2));
}

// 扫掠段 pre->pre+seg 与第 k 形状最早接触(局部空间)
void XPBDClothNative::_toi_sphere(const Vector3 &pre, const Vector3 &seg, real_t seg2, const Vector3 &c, real_t rr, real_t &ct, Vector3 &cn) const {
	ct = 2.0f;
	if (seg2 < 1e-12f) {
		return;
	}
	const Vector3 f = pre - c;
	const real_t t0 = CLAMP(-f.dot(seg) / seg2, 0.0f, 1.0f);
	if ((f + seg * t0).length_squared() > rr * rr) {
		return;
	}
	const real_t b = f.dot(seg);
	const real_t cc = f.dot(f) - rr * rr;
	real_t t = 0.0f;
	if (cc > 0.0f) {
		const real_t disc = b * b - seg2 * cc;
		t = MAX((-b - std::sqrt(MAX(disc, 0.0f))) / seg2, 0.0f);
	}
	const Vector3 nrm = f + seg * t;
	const real_t ln = nrm.length();
	ct = t;
	cn = ln > 1e-9f ? nrm / ln : Vector3(0, 1, 0);
}

void XPBDClothNative::_toi_capsule(const Vector3 &pre, const Vector3 &seg, real_t seg2, const Vector3 &ax, const Vector3 &bx, real_t rr, real_t &ct, Vector3 &cn) const {
	ct = 2.0f;
	if (seg2 < 1e-12f) {
		return;
	}
	Vector3 cen = closest_on_seg(ax, bx, pre);
	Vector3 f = pre - cen;
	real_t b = f.dot(seg);
	real_t cc = f.dot(f) - rr * rr;
	real_t t = 0.0f;
	if (cc > 0.0f) {
		const real_t disc = b * b - seg2 * cc;
		if (disc <= 0.0f) {
			return;
		}
		t = (-b - std::sqrt(disc)) / seg2;
		if (t < 0.0f || t > 1.0f) {
			return;
		}
	}
	for (int it = 0; it < 2; it++) {
		cen = closest_on_seg(ax, bx, pre + seg * t);
		f = pre - cen;
		cc = f.dot(f) - rr * rr;
		if (cc <= 0.0f) {
			t = 0.0f;
			break;
		}
		b = f.dot(seg);
		const real_t disc2 = b * b - seg2 * cc;
		if (disc2 <= 0.0f) {
			return;
		}
		const real_t t2 = (-b - std::sqrt(disc2)) / seg2;
		if (t2 < 0.0f || t2 > 1.0f) {
			return;
		}
		t = t2;
	}
	ct = t;
	const Vector3 nrm = pre + seg * t - cen;
	cn = nrm.length_squared() > 1e-18f ? nrm.normalized() : Vector3(0, 1, 0);
}

void XPBDClothNative::_toi_box(const Vector3 &pre, const Vector3 &seg, const Vector3 &c, const Vector3 &h, real_t &ct, Vector3 &cn) const {
	ct = 2.0f;
	const Vector3 l = pre - c;
	const Vector3 end = pre + seg;
	const Vector3 mna = pre.min(end) - c;
	const Vector3 mxa = pre.max(end) - c;
	if (mna.x > h.x || mxa.x < -h.x || mna.y > h.y || mxa.y < -h.y || mna.z > h.z || mxa.z < -h.z) {
		return;
	}
	real_t tmin = 0.0f;
	real_t tmax = 1.0f;
	Vector3 nrm;
	for (int axis = 0; axis < 3; axis++) {
		const real_t o = l[axis];
		const real_t d = seg[axis];
		const real_t hw = h[axis];
		if (std::abs(d) < 1e-9f) {
			if (std::abs(o) > hw) {
				return;
			}
			continue;
		}
		const real_t inv = 1.0f / d;
		real_t t1 = (-hw - o) * inv;
		real_t t2 = (hw - o) * inv;
		real_t enter_n = (d > 0.0f) ? -1.0f : 1.0f;
		if (t1 > t2) {
			const real_t tmp = t1;
			t1 = t2;
			t2 = tmp;
			enter_n = (d > 0.0f) ? 1.0f : -1.0f;
		}
		if (t1 > tmin) {
			tmin = t1;
			nrm = Vector3();
			nrm[axis] = enter_n;
		}
		tmax = MIN(tmax, t2);
		if (tmin > tmax) {
			return;
		}
	}
	if (tmin <= 0.0f) {
		if (tmax <= 0.0f) {
			return;
		}
		ct = 0.0f;
		cn = box_mtp_normal(l, h);
		return;
	}
	ct = tmin;
	cn = nrm;
}


// 单体 TOI 滑移 + 残余推出(局部空间)
Vector3 XPBDClothNative::_collide_one(const Vector3 &pre_w, const Vector3 &p_w) const {
	const int count = col_pt.size();
	if (count == 0) {
		return p_w;
	}
	const Vector3 mv = p_w - pre_w;
	const real_t ml = mv.length();
	const Vector3 to_p = p_w - col_xform.origin;
	const real_t br = col_broad_r + pad;
	if (to_p.length_squared() > br * br + ml * ml + 2.0f * br * ml) {
		return p_w;
	}
	Vector3 pre = col_xform_inv.xform(pre_w);
	Vector3 p = col_xform_inv.xform(p_w);
	for (int pass = 0; pass < 3; pass++) {
		const Vector3 seg = p - pre;
		const real_t seg2 = seg.length_squared();
		real_t best_t = 2.0f;
		Vector3 best_n;
		for (int k = 0; k < count; k++) {
			real_t ct = 2.0f;
			Vector3 cn;
			switch (col_pt[k]) {
				case SHAPE_SPHERE:
					_toi_sphere(pre, seg, seg2, col_poff[k], col_pr[k] + pad, ct, cn);
					break;
				case SHAPE_CAPSULE:
					_toi_capsule(pre, seg, seg2, col_pa[k], col_pb[k], col_pr[k] + pad, ct, cn);
					break;
				default:
					_toi_box(pre, seg, col_poff[k], col_phalf[k] + Vector3(pad, pad, pad), ct, cn);
					break;
			}
			if (ct < best_t) {
				best_t = ct;
				best_n = cn;
			}
		}
		if (best_t > 1.0f) {
			break;
		}
		const Vector3 cp = pre + seg * best_t;
		Vector3 rest = seg * (1.0f - best_t);
		const real_t vn = rest.dot(best_n);
		if (vn < 0.0f) {
			rest -= best_n * vn;
		}
		p = cp + best_n * 0.0006f + rest * (1.0f - friction);
		pre = cp + best_n * 0.0006f;
	}
	// 兜底: 任意残余穿透一律最小深度推出
	for (int k = 0; k < count; k++) {
		p = _push_shape(k, p);
	}
	return col_xform.xform(p);
}

// 逐顶点推出(地面夹紧 + 全部形状), 只推出不扫掠
void XPBDClothNative::_vertex_push_out() {
	const int n = pos.size();
	const real_t floor_y = ground_y + pad;
	for (int i = 0; i < n; i++) {
		if (pinned[i] == 1) {
			continue;
		}
		Vector3 p = pos[i];
		if (p.y < floor_y) {
			p.y = floor_y;
		}
		if (col_pt.size() > 0) {
			p = _collide_one(p, p);
		}
		if (!p.is_equal_approx(pos[i])) {
			pushed_hard.set(i, 1);
			pos.set(i, p);
		}
	}
	// 地面是最终硬不变量: 夹紧并清零向下速度(sub_recover 之后执行, 否则下子步会再扎入)
	for (int i = 0; i < n; i++) {
		if (pinned[i] == 1) {
			continue;
		}
		if (pos[i].y < floor_y) {
			pos.set(i, Vector3(pos[i].x, floor_y, pos[i].z));
			pushed_hard.set(i, 1);
			Vector3 v = vel[i];
			if (v.y < 0.0f) {
				v.y = 0.0f;
				vel.set(i, v);
			}
		}
	}
}

// 网格级兜底: 顶点全在体外时, 三角形边仍可能横切碰撞体(球间深谷/地面)
// 优化: 先按格子AABB与碰撞体包围球粗筛, 绝大多数格子零成本跳过
void XPBDClothNative::_mesh_contact_push() {
	if (cols < 2 || rows < 2) {
		return;
	}
	const bool has_col = col_pt.size() > 0;
	const Vector3 col_c = col_xform.origin;
	const real_t br = col_broad_r + pad;
	const real_t floor_y = ground_y + pad;
	for (int r = 0; r < rows - 1; r++) {
		for (int c = 0; c < cols - 1; c++) {
			const int a0 = r * cols + c;
			const Vector3 p0 = pos[a0];
			const Vector3 p1 = pos[a0 + cols];
			const Vector3 p2 = pos[a0 + 1];
			const Vector3 p3 = pos[a0 + cols + 1];
			const Vector3 mn = p0.min(p1).min(p2).min(p3);
			const Vector3 mx = p0.max(p1).max(p2).max(p3);
			const bool near_ground = mn.y <= floor_y;
			bool near_col = false;
			if (has_col) {
				const Vector3 cc = (mn + mx) * 0.5f;
				const real_t cr = (mx - mn).length() * 0.5f;
				const real_t lim = br + cr;
				if (cc.distance_squared_to(col_c) <= lim * lim) {
					near_col = true;
				}
			}
			if (!near_ground && !near_col) {
				continue;
			}
			const int ids[4] = { a0, a0 + cols, a0 + 1, a0 + cols + 1 };
			const int edges[5][2] = {
				{ ids[0], ids[1] }, { ids[0], ids[2] }, { ids[1], ids[3] }, { ids[2], ids[3] }, { ids[1], ids[2] }
			};
			for (int e = 0; e < 5; e++) {
				const int ia = edges[e][0];
				const int ib = edges[e][1];
				const real_t wa = (pinned[ia] == 1 || ia == grab_id) ? 0.0f : 1.0f;
				const real_t wb = (pinned[ib] == 1 || ib == grab_id) ? 0.0f : 1.0f;
				const real_t w = wa + wb;
				if (w == 0.0f) {
					continue;
				}
				const Vector3 mid_w = (pos[ia] + pos[ib]) * 0.5f;
				Vector3 out_w = mid_w;
				if (out_w.y < floor_y) {
					out_w.y = floor_y;
				}
				if (near_col && has_col) {
					Vector3 p_l = col_xform_inv.xform(out_w);
					bool inside = false;
					for (int k = 0; k < col_pt.size(); k++) {
						if (_inside_shape(k, p_l)) {
							inside = true;
							break;
						}
					}
					if (inside) {
						for (int k = 0; k < col_pt.size(); k++) {
							p_l = _push_shape(k, p_l);
						}
						out_w = col_xform.xform(p_l);
					}
				}
				if (!out_w.is_equal_approx(mid_w)) {
					const Vector3 delta = out_w - mid_w;
					pos.set(ia, pos[ia] + delta * (wa / w));
					pos.set(ib, pos[ib] + delta * (wb / w));
					pushed_hard.set(ia, 1);
					pushed_hard.set(ib, 1);
				}
			}
		}
	}
}

void XPBDClothNative::object_collision() {
	if (!enable_object_collision) {
		return;
	}
	pad = cloth_thickness * 0.5f + collision_margin;
	const int n = pos.size();
	const real_t floor_y = ground_y + pad;
	const bool has_col = col_pt.size() > 0;
	for (int i = 0; i < n; i++) {
		if (pinned[i] == 1) {
			continue;
		}
		const Vector3 pre_w = prev[i];
		Vector3 p_w = pos[i];
		// 地面 + 切向摩擦(世界空间)
		if (p_w.y < floor_y) {
			const Vector3 slide = Vector3(p_w.x - pre_w.x, 0.0f, p_w.z - pre_w.z);
			p_w.y = floor_y;
			p_w -= slide * friction;
		}
		if (has_col) {
			p_w = _collide_one(pre_w, p_w);
		}
		if (!p_w.is_equal_approx(pos[i])) {
			pos.set(i, p_w);
			pushed_hard.set(i, 1);
		}
	}
}

void XPBDClothNative::final_push_out() {
	if (!enable_object_collision) {
		return;
	}
	pad = cloth_thickness * 0.5f + collision_margin;
	// 顺序关键: 先应变限制(会把顶点拉回体内) -> 顶点推出 -> 网格推出 -> 顶点再推出
	_strain_limit();
	_vertex_push_out();
	_mesh_contact_push();
	_vertex_push_out();
}

// ============================================================================
// 自碰欠量实测(不推顶点)
// ============================================================================
real_t XPBDClothNative::_self_deficit() const {
	const int n = pos.size();
	const real_t min_d = cloth_thickness * 0.5f + tri_r;
	const real_t min_d2 = min_d * min_d;
	const real_t inv = 1.0f / min_d;
	CellGrid grid;
	grid.build(pos, inv, 1024);
	real_t worst = 0.0f;
	for (int i = 0; i < n; i++) {
		const Vector3 p = pos[i];
		const int cx = (int)std::floor(p.x * inv) + 512;
		const int cy = (int)std::floor(p.y * inv) + 512;
		const int cz = (int)std::floor(p.z * inv) + 512;
		const int ri = i / cols;
		const int ci = i % cols;
		for (int o = 0; o < 27; o++) {
			int b, e;
			if (!grid.find(pack_cell(cx + kOffs[o][0], cy + kOffs[o][1], cz + kOffs[o][2]), b, e)) {
				continue;
			}
			for (int t = b; t < e; t++) {
				const int j = grid.order[t];
				if (j <= i) {
					continue;
				}
				const int dr = std::abs(j / cols - ri);
				const int dc = std::abs(j % cols - ci);
				if (dr <= 2 && dc <= 2) {
					continue;
				}
				const real_t d2 = p.distance_squared_to(pos[j]);
				if (d2 < min_d2 && d2 > 1e-12f) {
					const real_t dfc = min_d - std::sqrt(d2);
					if (dfc > worst) {
						worst = dfc;
					}
				}
			}
		}
	}
	return worst;
}

int XPBDClothNative::penetration_count() const {
	if (!enable_object_collision || col_pt.size() == 0) {
		return 0;
	}
	const real_t p = cloth_thickness * 0.5f + collision_margin;
	int count = 0;
	for (int i = 0; i < (int)pos.size(); i += 3) {
		const Vector3 pl = col_xform_inv.xform(pos[i]);
		for (int k = 0; k < col_pt.size(); k++) {
			real_t depth = 0.0f;
			if (col_pt[k] == SHAPE_BOX) {
				const Vector3 l = pl - col_poff[k];
				const Vector3 h = col_phalf[k];
				if (std::abs(l.x) < h.x && std::abs(l.y) < h.y && std::abs(l.z) < h.z) {
					count++;
					break;
				}
			} else {
				const Vector3 cen = (col_pt[k] == SHAPE_SPHERE) ? col_poff[k] : closest_on_seg(col_pa[k], col_pb[k], pl);
				const real_t rr = col_pr[k] + p * 0.5f;
				if ((pl - cen).length_squared() < rr * rr) {
					count++;
					break;
				}
			}
		}
	}
	return count;
}


// ============================================================================
// 布间互碰(所有已注册布料, 顶点级 + 外接圆半径几何补偿)
// 与 GDScript 语义一致: 跨布最小间距 = S_visual + R (排除"顶点扎进对层表面")
// 优化: C++ 下一次性构建全体顶点哈希, 只哈希"落在其它布料 AABB 内"的顶点
// ============================================================================
int XPBDClothNative::cross_contact_all(bool push) {
	const int nc = (int)g_cloths.size();
	if (nc < 2) {
		return 0;
	}
	// 跨布判据取所有布料的 tri_r 最大值做补偿
	real_t max_r = 0.0f;
	real_t thick = 0.0f;
	for (int i = 0; i < nc; i++) {
		max_r = MAX(max_r, g_cloths[i]->tri_r);
		thick = MAX(thick, g_cloths[i]->cloth_thickness);
	}
	const real_t min_d = thick * 0.5f + max_r;
	const real_t min_d2 = min_d * min_d;
	const real_t inv = 1.0f / min_d;

	// 只收集"可能接触"的布料对(AABB 相交)
	struct PairCand { int a; int i; int b; int k; };
	static std::vector<PairCand> cand;
	cand.clear();

	// 全体参与布料的 AABB
	std::vector<AABB> boxes(nc);
	for (int ci = 0; ci < nc; ci++) {
		XPBDClothNative *c = g_cloths[ci];
		AABB bb(c->pos[0], Vector3());
		for (int i = 0; i < (int)c->pos.size(); i++) {
			bb.expand_to(c->pos[i]);
		}
		boxes[ci] = bb;
	}

	for (int ci = 0; ci < nc; ci++) {
		XPBDClothNative *A = g_cloths[ci];
		for (int cj = ci + 1; cj < nc; cj++) {
			XPBDClothNative *B = g_cloths[cj];
			AABB a = boxes[ci];
			AABB b = boxes[cj];
			a.grow_by(min_d);
			b.grow_by(min_d);
			if (!a.intersects(b)) {
				continue;
			}
			// A 的顶点 vs B 的顶点(邻域由哈希保证)
			for (int i = 0; i < (int)A->pos.size(); i++) {
				const Vector3 p = A->pos[i];
				for (int k = 0; k < (int)B->pos.size(); k++) {
					if (p.distance_squared_to(B->pos[k]) >= min_d2) {
						continue;
					}
					PairCand pc2;
					pc2.a = ci; pc2.i = i; pc2.b = cj; pc2.k = k;
					cand.push_back(pc2);
				}
			}
		}
	}
	if (!push) {
		return (int)cand.size();
	}
	if (cand.empty()) {
		return 0;
	}

	// 松弛(候选列表已定, 不需要重建哈希)
	real_t worst = 0.0f;
	for (int sweep = 0; sweep < 8; sweep++) {
		worst = 0.0f;
		bool pushed = false;
		for (size_t q = 0; q < cand.size(); q++) {
			const PairCand &pc = cand[q];
			XPBDClothNative *A = g_cloths[pc.a];
			XPBDClothNative *B = g_cloths[pc.b];
			const Vector3 pa = A->pos[pc.i];
			const Vector3 pb = B->pos[pc.k];
			const Vector3 d = pa - pb;
			const real_t d2 = d.length_squared();
			if (d2 >= min_d2 || d2 < 1e-12f) {
				continue;
			}
			const real_t ln = std::sqrt(d2);
			const real_t deficit = min_d - ln;
			if (deficit > worst) {
				worst = deficit;
			}
			const real_t wa = (A->pinned[pc.i] == 1 || (A->grabbing && pc.i == A->grab_id)) ? 0.0f : 1.0f;
			const real_t wb = (B->pinned[pc.k] == 1 || (B->grabbing && pc.k == B->grab_id)) ? 0.0f : 1.0f;
			const real_t w = wa + wb;
			if (w == 0.0f) {
				continue;
			}
			const Vector3 nrm = d / ln;
			const real_t mv = deficit / w;
			if (wa > 0.0f) {
				A->pos.set(pc.i, A->pos[pc.i] + nrm * mv * wa);
				A->pushed_soft.set(pc.i, 1);
			}
			if (wb > 0.0f) {
				B->pos.set(pc.k, B->pos[pc.k] - nrm * mv * wb);
				B->pushed_soft.set(pc.k, 1);
			}
			pushed = true;
		}
		if (!pushed || worst < 0.005f) {
			break;
		}
	}
	return 0;
}

// 布间欠量实测(不推), 供 HUD/审计
real_t XPBDClothNative::cross_deficit_all() {
	const int nc = (int)g_cloths.size();
	if (nc < 2) {
		return 0.0f;
	}
	real_t max_r = 0.0f;
	real_t thick = 0.0f;
	for (int i = 0; i < nc; i++) {
		max_r = MAX(max_r, g_cloths[i]->tri_r);
		thick = MAX(thick, g_cloths[i]->cloth_thickness);
	}
	const real_t min_d = thick * 0.5f + max_r;
	real_t worst = 0.0f;
	for (int ci = 0; ci < nc; ci++) {
		XPBDClothNative *A = g_cloths[ci];
		for (int cj = ci + 1; cj < nc; cj++) {
			XPBDClothNative *B = g_cloths[cj];
			for (int i = 0; i < (int)A->pos.size(); i++) {
				const Vector3 p = A->pos[i];
				for (int k = 0; k < (int)B->pos.size(); k++) {
					const real_t d2 = p.distance_squared_to(B->pos[k]);
					if (d2 < min_d * min_d) {
						const real_t dfc = min_d - std::sqrt(MAX(d2, 0.0f));
						if (dfc > worst) {
							worst = dfc;
						}
					}
				}
			}
		}
	}
	return worst;
}

// ============================================================================
// 抓取/拾取/网格
// ============================================================================
int XPBDClothNative::pick_vertex(const Vector3 &ro, const Vector3 &rn) const {
	int best = -1;
	real_t best_along = 1e9f;
	int fallback = -1;
	real_t fb_along = 1e9f;
	for (int i = 0; i < (int)pos.size(); i++) {
		const Vector3 to_p = pos[i] - ro;
		const real_t along = to_p.dot(rn);
		if (along < 0.1f) {
			continue;
		}
		const real_t perp = (to_p - rn * along).length();
		if (perp < 0.45f && along < best_along) {
			best_along = along;
			best = i;
		}
		if (along < fb_along) {
			fb_along = along;
			fallback = i;
		}
	}
	return best >= 0 ? best : fallback;
}

void XPBDClothNative::release_grab(int substeps) {
	if (grab_id >= 0 && grab_id < (int)vel.size()) {
		Vector3 v = (grab_target - prev[grab_id]) * 60.0f / (real_t)MAX(substeps, 1);
		const real_t vl = v.length();
		if (vl > 12.0f) {
			v = v * (12.0f / vl);
		}
		vel.set(grab_id, v);
	}
	grabbing = false;
	grab_id = -1;
}

void XPBDClothNative::update_mesh() {
	Ref<ArrayMesh> m;
	m.instantiate();
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = pos;

	PackedInt32Array indices;
	indices.resize((rows - 1) * (cols - 1) * 6);
	int w = 0;
	for (int r = 0; r < rows - 1; r++) {
		for (int c = 0; c < cols - 1; c++) {
			const int a0 = r * cols + c;
			indices.set(w++, a0);
			indices.set(w++, a0 + cols);
			indices.set(w++, a0 + 1);
			indices.set(w++, a0 + 1);
			indices.set(w++, a0 + cols);
			indices.set(w++, a0 + cols + 1);
		}
	}
	arrays[Mesh::ARRAY_INDEX] = indices;

	// 法线: 面法线累加到顶点
	PackedVector3Array normals;
	normals.resize(pos.size());
	for (int i = 0; i < (int)pos.size(); i++) {
		normals.set(i, Vector3());
	}
	for (int t = 0; t < indices.size(); t += 3) {
		const int a = indices[t];
		const int b = indices[t + 1];
		const int c = indices[t + 2];
		const Vector3 nm = (pos[b] - pos[a]).cross(pos[c] - pos[a]);
		normals.set(a, normals[a] + nm);
		normals.set(b, normals[b] + nm);
		normals.set(c, normals[c] + nm);
	}
	for (int i = 0; i < (int)pos.size(); i++) {
		const Vector3 n = normals[i];
		normals.set(i, n.length() > 1e-8f ? n.normalized() : Vector3(0, 0, 1));
	}
	arrays[Mesh::ARRAY_NORMAL] = normals;

	m->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	if (material.is_valid()) {
		m->surface_set_material(0, material);
	}
	mesh = m;
}


int XPBDClothNative::registry_size() { return (int)g_cloths.size(); }

// 诊断: [注册布料数, 候选对数, AABB相交对, min_d(um)]
PackedInt32Array XPBDClothNative::cross_debug() {
	PackedInt32Array r;
	r.push_back((int)g_cloths.size());
	const int nc = (int)g_cloths.size();
	if (nc < 2) { r.push_back(0); r.push_back(0); r.push_back(0); return r; }
	real_t max_r = 0.0f, thick = 0.0f;
	for (int i = 0; i < nc; i++) { max_r = MAX(max_r, g_cloths[i]->tri_r); thick = MAX(thick, g_cloths[i]->cloth_thickness); }
	const real_t min_d = thick * 0.5f + max_r;
	int ab = 0, pairs = 0;
	for (int ci = 0; ci < nc; ci++) {
		for (int cj = ci + 1; cj < nc; cj++) {
			AABB a(g_cloths[ci]->pos[0], Vector3()); 
			for (int i = 0; i < (int)g_cloths[ci]->pos.size(); i++) a.expand_to(g_cloths[ci]->pos[i]);
			AABB b(g_cloths[cj]->pos[0], Vector3());
			for (int i = 0; i < (int)g_cloths[cj]->pos.size(); i++) b.expand_to(g_cloths[cj]->pos[i]);
			AABB ag = a; ag.grow_by(min_d);
			AABB bg = b; bg.grow_by(min_d);
			if (ag.intersects(bg)) ab++;
			for (int i = 0; i < (int)g_cloths[ci]->pos.size(); i++) {
				for (int k = 0; k < (int)g_cloths[cj]->pos.size(); k++) {
					if (g_cloths[ci]->pos[i].distance_squared_to(g_cloths[cj]->pos[k]) < min_d * min_d) pairs++;
				}
			}
		}
	}
	r.push_back(pairs);
	r.push_back(ab);
	r.push_back((int)(min_d * 1000000.0f));
	return r;
}
