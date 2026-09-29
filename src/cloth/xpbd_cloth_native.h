#ifndef XPBD_CLOTH_NATIVE_H
#define XPBD_CLOTH_NATIVE_H

// XPBD 布料求解器 (C++/GDExtension)
// 与 scripts/xpbd_cloth.gd 物理语义逐条对齐, 仅把热路径搬到 C++:
//   - 约束求解(结构/剪切/弯曲) + 应变限制(带"无需生效即跳过")
//   - 自碰撞(空间哈希 + 几何补偿 S_visual+R + 法向阻尼 + 休眠)
//   - 物体碰撞(局部空间解析 TOI 滑移 + 最小推出 + 网格级边推出)
//   - 积分/速度恢复(接触修正不产生发射速度 + 硬接触法向投影)
//   - 布间互碰(静态注册表, 顶点级 + 外接圆半径补偿)
// 场景编排(子步调度/UI/相机)仍由 GDScript 管理。

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <vector>

namespace godot {

class XPBDClothNative : public Node3D {
	GDCLASS(XPBDClothNative, Node3D)

public:
	// ---- 形状类型 ----
	enum ShapeType { SHAPE_SPHERE = 0, SHAPE_CAPSULE = 1, SHAPE_BOX = 2 };

private:
	// ---- 网格拓扑 ----
	int cols = 24;
	int rows = 24;
	Vector2 cloth_size = Vector2(2.0, 2.0);
	Vector3 spawn_origin = Vector3();
	bool vertical = false;
	real_t yaw = 0.0;

	// ---- 粒子状态 ----
	PackedVector3Array pos;
	PackedVector3Array prev;
	PackedVector3Array vel;
	PackedByteArray pinned;

	// ---- 约束(CSR-free, 平坦数组) ----
	PackedInt32Array c_ia;
	PackedInt32Array c_ib;
	PackedFloat32Array c_rest;
	PackedByteArray c_alive;
	PackedInt32Array c_type; // 0=结构 1=剪切 2=弯曲

	// ---- 静态拓扑: 顶点 -> 关联三角形(CSR) ----
	PackedInt32Array vtx_tri_ofs;
	PackedInt32Array vtx_tri_ids;
	PackedInt32Array tri_verts;
	real_t tri_r = 0.0; // 三角形外接圆半径上界
	Color albedo = Color(0.85f, 0.3f, 0.35f); // 布料颜色(材质)

	// ---- 物理参数(与 GDScript 同名) ----
	int iterations = 1;
	Vector3 gravity = Vector3(0, -9.8, 0);
	real_t damping = 0.02;
	real_t air_drag = 0.005;
	real_t tear_dist_factor = 0.0; // 0=禁用撕裂(与 GDScript 一致)
	real_t max_stretch = 1.06;
	real_t compliance_struct = 0.0;
	real_t compliance_shear = 0.00002;
	real_t compliance_bend = 0.0003;
	real_t cloth_thickness = 0.035;
	real_t friction = 0.85;
	real_t collision_margin = 0.012;
	bool enable_object_collision = true;
	bool enable_self_collision = true;
	real_t ground_y = -1.6;
	Vector3 wind = Vector3(); // 风力(加速度)
	Vector3 model_origin = Vector3();
	real_t model_yaw_deg = 0.0f;

	// ---- 身体胶囊碰撞(裙布/衣服不穿进身体) ----
	bool body_collision_on = false;
	Vector3 body_seg_a = Vector3(0, 1.42f, 0);
	Vector3 body_seg_b = Vector3(0, 0.55f, 0);
	real_t body_seg_r = 0.17f;

	// ---- 抓取 ----
	bool grabbing = false;
	int grab_id = -1;
	Vector3 grab_target = Vector3();
	real_t grab_depth = 0.0;
	real_t grab_stiffness = 0.6;
	real_t grab_max_speed = 12.0;

	// ---- 接触修正标记(速度恢复用) ----
	PackedVector3Array pre_contact; // 接触阶段前的位置快照
	PackedByteArray pushed_hard;    // 硬接触: 物体/地面/网格边 -> 法向投影
	PackedByteArray pushed_soft;    // 软接触: 自碰/布间 -> 保留分离动量
	int strain_active = 0;          // 上一次应变限制实际生效的约束数(诊断/优化用)
	// 自碰候选接触对(哈希构建一次后复用, 避免多轮重建)
	std::vector<int32_t> cand_a;
	std::vector<int32_t> cand_b;

	// ---- 自碰哈希 ----
	real_t contact_deficit = 0.0;
	real_t max_vel2 = 0.0;
	bool asleep = false;
	int sleep_checks = 0;
	int sleep_frame = 0;

	// ---- 碰撞体(由 GDScript 每帧下发: 形状局部空间 + 世界变换) ----
	PackedInt32Array col_pt;
	PackedVector3Array col_poff;
	PackedVector3Array col_pa;
	PackedVector3Array col_pb;
	PackedFloat32Array col_pr;
	PackedVector3Array col_phalf;
	Transform3D col_xform = Transform3D();
	Transform3D col_xform_inv = Transform3D();
	real_t col_broad_r = 0.0;
	real_t pad = 0.0;

	// ---- 渲染 ----
	Ref<ArrayMesh> mesh;
	Ref<Material> material;
	void set_albedo(const Color &v) { albedo = v; }
	Color get_albedo() const { return albedo; }

	// ---- 内部方法 ----
	void _add_constraint(int ia, int ib, int ctype);
	void _build_topology();
	void _strain_limit();
	void _vertex_push_out();
	void _mesh_contact_push();
	Vector3 _collide_one(const Vector3 &pre_w, const Vector3 &p_w) const;
	real_t _self_deficit() const;
	// TOI 内核(局部空间)
	void _toi_sphere(const Vector3 &pre, const Vector3 &seg, real_t seg2, const Vector3 &c, real_t rr, real_t &ct, Vector3 &cn) const;
	void _toi_capsule(const Vector3 &pre, const Vector3 &seg, real_t seg2, const Vector3 &ax, const Vector3 &bx, real_t rr, real_t &ct, Vector3 &cn) const;
	void _toi_box(const Vector3 &pre, const Vector3 &seg, const Vector3 &c, const Vector3 &h, real_t &ct, Vector3 &cn) const;
	Vector3 _push_shape(int k, const Vector3 &p) const;
	bool _inside_shape(int k, const Vector3 &p) const;

protected:
	static void _bind_methods();

public:
	XPBDClothNative();
	~XPBDClothNative();

	// ---- 构建/参数 ----
	void build();
	void set_collider_shapes(const PackedInt32Array &pt, const PackedVector3Array &poff,
			const PackedVector3Array &pa, const PackedVector3Array &pb,
			const PackedFloat32Array &pr, const PackedVector3Array &phalf,
			const Transform3D &xform);

	// ---- 子步阶段 ----
	void sub_begin();
	void begin_contacts();
	void sub_integrate(real_t h, const Vector3 &wind);
	void sub_solve(real_t h);
	void self_collision();
	void object_collision();
	void final_push_out();
	void sub_recover(real_t h);

	// ---- 状态访问 ----
	PackedVector3Array get_positions() const { return pos; }
	void set_positions(const PackedVector3Array &p) { pos = p; }
	// 自定义顶点布局后必须调用: 按当前顶点位置重算约束静止长度。
	// 否则静止长度仍是 build() 时网格间距, 求解器会把布撑开去满足它(实测裙布被拉长6倍摊到地上)。
	void refresh_rest_lengths();
	PackedVector3Array get_velocities() const { return vel; }
	void set_velocities(const PackedVector3Array &v) { vel = v; }
	PackedByteArray get_pinned_flags() const { return pinned; }
	void set_pinned_flags(const PackedByteArray &v) { pinned = v; }

	// ---- 审计/抓取 ----
	int penetration_count() const;
	real_t self_deficit() const { return _self_deficit(); }
	int pick_vertex(const Vector3 &ro, const Vector3 &rn) const;
	void release_grab(int substeps);
	void update_mesh();

	// ---- 布间互碰(全局注册表) ----
	// register/unregister 保持 static(供注册表维护); cross_* 是实例方法(可绑定到 GDScript),
	// 内部操作全局注册表 —— 静态方法无法用 ClassDB::bind_method 绑定。
	static void register_cloth(XPBDClothNative *c);
	static void unregister_cloth(XPBDClothNative *c);
	int cross_contact_all(bool push);
	int registry_size();
	PackedInt32Array cross_debug();
	real_t cross_deficit_all(); // 实测最深欠量(不推)

	// ---- 属性读写(GDScript 兼容) ----
	void set_cols(int v) { cols = v; }
	int get_cols() const { return cols; }
	void set_rows(int v) { rows = v; }
	int get_rows() const { return rows; }
	void set_cloth_size(const Vector2 &v) { cloth_size = v; }
	Vector2 get_cloth_size() const { return cloth_size; }
	void set_spawn_origin(const Vector3 &v) { spawn_origin = v; }
	Vector3 get_spawn_origin() const { return spawn_origin; }
	void set_vertical(bool v) { vertical = v; }
	bool get_vertical() const { return vertical; }
	void set_yaw(real_t v) { yaw = v; }
	real_t get_yaw() const { return yaw; }

	void set_gravity(const Vector3 &v) { gravity = v; }
	Vector3 get_gravity() const { return gravity; }
	void set_damping(real_t v) { damping = v; }
	real_t get_damping() const { return damping; }
	void set_air_drag(real_t v) { air_drag = v; }
	real_t get_air_drag() const { return air_drag; }
	void set_max_stretch(real_t v) { max_stretch = v; }
	real_t get_max_stretch() const { return max_stretch; }
	void set_compliance_struct(real_t v) { compliance_struct = v; }
	real_t get_compliance_struct() const { return compliance_struct; }
	void set_compliance_shear(real_t v) { compliance_shear = v; }
	real_t get_compliance_shear() const { return compliance_shear; }
	void set_compliance_bend(real_t v) { compliance_bend = v; }
	real_t get_compliance_bend() const { return compliance_bend; }
	void set_cloth_thickness(real_t v) { cloth_thickness = v; }
	real_t get_cloth_thickness() const { return cloth_thickness; }
	void set_friction(real_t v) { friction = v; }
	real_t get_friction() const { return friction; }
	void set_enable_object_collision(bool v) { enable_object_collision = v; }
	bool get_enable_object_collision() const { return enable_object_collision; }
	void set_enable_self_collision(bool v) { enable_self_collision = v; }
	bool get_enable_self_collision() const { return enable_self_collision; }
	void set_ground_y(real_t v) { ground_y = v; }
	void set_wind_v(const Vector3 &v) { wind = v; }
	Vector3 get_wind_v() const { return wind; }
	void translate_model(const Vector3 &delta);
	void set_body_capsule(bool on, const Vector3 &a, const Vector3 &b, real_t r) {
		body_collision_on = on; body_seg_a = a; body_seg_b = b; body_seg_r = r;
	}
	void collide_body_capsule();
	real_t get_ground_y() const { return ground_y; }
	void set_iterations(int v) { iterations = v; }
	int get_iterations() const { return iterations; }
	void set_grab_stiffness(real_t v) { grab_stiffness = v; }
	real_t get_grab_stiffness() const { return grab_stiffness; }
	void set_grabbing(bool v) { grabbing = v; }
	bool get_grabbing() const { return grabbing; }
	void set_grab_id(int v) { grab_id = v; }
	int get_grab_id() const { return grab_id; }
	void set_grab_target(const Vector3 &v) { grab_target = v; }
	Vector3 get_grab_target() const { return grab_target; }
	void set_material(const Ref<Material> &m) { material = m; }
	Ref<Material> get_material() const { return material; }

	real_t get_contact_deficit() const { return contact_deficit; }
	void set_contact_deficit(real_t v) { contact_deficit = v; }
	real_t get_max_vel2() const { return max_vel2; }
	void set_max_vel2(real_t v) { max_vel2 = v; }
	bool get_asleep() const { return asleep; }
	real_t get_tri_r() const { return tri_r; }
	int get_constraint_count() const { return c_ia.size(); }
	int get_torn_count() const;
	int get_triangle_count() const { return tri_verts.size() / 3; }
	PackedInt32Array get_tri_verts() const { return tri_verts; }
	PackedInt32Array get_vtx_tri_ofs() const;
	PackedInt32Array get_vtx_tri_ids() const;
	PackedByteArray get_constraint_alive() const { return c_alive; }
	Ref<ArrayMesh> get_mesh() const { return mesh; }
	PackedInt32Array get_constraint_ia() const { return c_ia; }
	PackedInt32Array get_constraint_ib() const { return c_ib; }
	PackedFloat32Array get_constraint_rest() const { return c_rest; }
	PackedInt32Array get_constraint_type() const { return c_type; }
};

} // namespace godot

#endif
