#ifndef XPBD_HAIR_NATIVE_H
#define XPBD_HAIR_NATIVE_H

// XPBD 头发求解器 (C++/GDExtension)
//
// 模型: 每根发束 = 一条 1D 粒子链("一条条的布料条带"):
//   - 距离约束(相邻粒子, 刚性): 保持发束长度
//   - 弯曲约束(隔一个粒子): 控制发束硬度, 让发梢自然下垂
//   - 重力 + 阻尼 + 空气阻力
//   - 头部球形碰撞 + 地面
//   - 发束间自碰撞(空间哈希): 防止发丝相互穿插
//
// 插值渲染: 少量"导向发束"(物理模拟) 插值出大量"渲染发丝"(纯视觉):
//   - 对每个渲染发丝, 找最近的 K 个导向发束, 按反距离加权插值其粒子位置
//   - 渲染发丝带横向偏移, 覆盖头皮, 视觉密度远高于物理密度
//
// 渲染: 朝向相机的带状网格(每对相邻粒子一条带), 根粗梢细, 根部色深。

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot {

class XPBDHairNative : public Node3D {
	GDCLASS(XPBDHairNative, Node3D)

public:
private:
	// ---- 拓扑 ----
	int strand_count = 48;      // 导向发束数(物理模拟)
	int segment_count = 12;     // 每束段数(粒子数 = segment_count + 1)
	real_t hair_length = 0.55f; // 发束总长(m)
	real_t head_radius = 0.12f; // 头部碰撞球半径(m)
	// 头部椭球半轴(m)。非零时优先用椭球做头部碰撞 —— 真实头模不是球,
	// 球体在前脸会"让位过多", 发丝会垂到脸上; 椭球贴合得多。
	Vector3 head_radii = Vector3();
	Vector3 head_center = Vector3(0, 0.2f, 0);
	Vector3 head_up = Vector3(0, 1, 0);

	// ---- 物理 ----
	real_t gravity_scale = 1.0f;
	real_t damping = 0.06f;
	Vector3 wind = Vector3();  // 风力(加速度, m/s^2), 方向可调
	real_t stiffness = 0.7f;     // 弯曲刚度 0..1 (映射到弯曲约束刚性)
	real_t root_stiffness = 0.9f; // 靠发根处额外刚性(发根更硬)
	real_t collide_radius = 0.008f; // 发丝碰撞半径(m)
	real_t friction = 0.4f;

	// ---- 插值渲染 ----
	int render_strands = 240; // 渲染发丝数
	real_t strand_width = 0.006f;
	real_t tip_width = 0.0015f;
	Color root_color = Color(0.20f, 0.13f, 0.09f);
	Color tip_color = Color(0.52f, 0.36f, 0.24f);
	Color albedo = Color(0.45f, 0.35f, 0.25f);

	// ---- 模型变换(头发跟随身体移动/旋转) ----
	// roots/head_center/body 胶囊都存在"模型空间", 每子步变换到世界空间。
	// 模型移动时只改 model_origin/model_yaw, 发根自动跟随, 且粒子继承模型速度(惯性)。
	Vector3 model_origin = Vector3();
	real_t model_yaw = 0.0f;
	Vector3 model_prev_origin = Vector3();
	Vector3 model_vel = Vector3();   // 模型线速度(拖拽时产生, 让头发有拖拽惯性)

	// ---- 粒子状态(导向发束) ----
	PackedVector3Array pos;   // strand_count * (segment_count+1)
	PackedVector3Array prev;
	PackedVector3Array vel;
	PackedVector3Array rest_dir;   // 发束局部静止方向(构建时)
	PackedFloat32Array seg_len;    // 每段原长

	// ---- 约束(CSR-free 平坦数组) ----
	// 距离约束: 相邻粒子; 弯曲约束: 隔一个粒子
	PackedInt32Array c_ia, c_ib;
	PackedFloat32Array c_rest;
	PackedInt32Array c_type; // 0=距离 1=弯曲
	PackedFloat32Array c_stiff; // 每条约束的刚性(0..1, 沿发根到发梢衰减)

	// ---- 头皮锚点(发根全局位置, 构建时算好) ----
	PackedVector3Array roots;
	PackedVector3Array root_normals;

	// ---- 真实头皮(HS2 捏人角色) ----
	// 调用方只需把头模顶点云交进来(set_scalp_mesh), 发根采样/发长分配/刘海分流
	// 全部由本插件完成 —— 与"布料测试场景"里头发自带头皮采样的口径一致。
	bool scalp_on = false;
	PackedVector3Array scalp_rest;      // 采样出的头皮发根(调用方给的静止空间)
	PackedVector3Array scalp_normals;   // 每根发根处的头皮法线
	PackedVector3Array scalp_dir;       // 每根发束的初始生长方向(刘海侧向梳开/其余沿头皮下坡)
	PackedFloat32Array strand_len;      // 每根发束长度(m): 刘海短/头顶中/颈后长
	PackedVector3Array head_verts;      // 头模顶点云(静止空间)
	PackedVector3Array skel_prev;       // 上一帧发根(求骨骼线速度, 给头发惯性)
	Vector3 skel_vel = Vector3();
	Vector3 skel_head = Vector3(0, 1.55f, 0);
	Vector3 scalp_center = Vector3(0, 1.66f, 0);   // 头心(静止空间)
	Vector3 face_axis = Vector3(0, 0, 1);          // 脸朝向(静止空间, 水平分量即可)

	// ---- 脸部挡板(不让头发垂到脸上) ----
	// 光靠"发根初始方向朝后"是不够的: 物理一稳定, 前额那些发丝照样会被重力
	// 拉直成一道帘子挂在眼前。这里加一个**位于颜面前方的竖直挡板**:
	// 眉毛高度以下、且仍在脸宽范围内的发丝, 一律不能被推到挡板前面去。
	bool face_guard_on = false;
	Vector3 face_pt = Vector3();     // 挡板上一点(静止空间世界坐标)
	Vector3 face_nrm = Vector3();    // 指向"脑后"的单位法线
	real_t face_min_y = 0.0f;        // 低于该高度才启用
	real_t face_guard_r = 0.10f;     // 只有横向仍在脸宽内的发丝受约束

	// ---- 头皮采样参数(插件内完成) ----
	int scalp_rows = 17;          // 头皮网格行(极角方向)
	int scalp_cols = 24;          // 头皮网格列(方位角方向)
	real_t len_top = 0.26f;       // 头顶/侧后发长(m)
	real_t len_bottom = 0.62f;    // 颈后发长(m)
	real_t bang_len = 0.13f;      // 刘海长度(m)
	real_t len_jitter = 0.12f;    // 发长随机扰动(±)
	real_t face_phi = 1.15f;      // 面部区判定: 方位角大于它且朝前 -> 不长头发
	real_t face_z = 0.62f;
	real_t bang_phi_lo = 0.40f;   // 刘海带(极角区间)
	real_t bang_phi_hi = 1.15f;
	real_t bang_z = 0.55f;

	// ---- 渲染顶点限位(只对渲染网格做, 不参与物理) ----
	// 真实头模不是球: 用"头心以下的发丝必须有身体把它垫住"这一约束,
	// 把悬空/穿脸的发丝顶点轻推到身体胶囊外侧, 避免长头发飘在胸前或糊在脸上。
	real_t render_min_y = -1.0f;        // <0 = 关闭
	real_t render_snap = 0.030f;        // 限位落差超过它就整体下移, 否则只做向外推

	// ---- 渲染发丝插值权重: 每渲染发丝 -> K 个(导向束索引, 权重) ----
	int blend_k = 4;
	PackedInt32Array blend_idx;    // render_strands * blend_k
	PackedFloat32Array blend_w;    // render_strands * blend_k

	// ---- 渲染 ----
	Ref<ArrayMesh> mesh;
	Ref<Material> material;

	bool built = false;

	// ---- 头部运动(甩头测试): 每子步记录头部位移, 粒子继承头速度 ----
	Vector3 head_prev = Vector3();
	bool head_moving = false;

	// ---- 可移动障碍球(碰撞测试) ----
	bool obstacle_on = false;
	Vector3 obstacle_c = Vector3(0, -0.1f, 0);
	real_t obstacle_r = 0.09f;

	// ---- 身体胶囊碰撞(头发垂到身上不穿模) ----
	bool body_on = true;
	Vector3 body_a = Vector3(0, 1.42f, 0);   // 胶囊上端(肩部)
	Vector3 body_b = Vector3(0, 0.55f, 0);   // 胶囊下端(臀部)
	real_t body_r = 0.17f;                    // 身体半径

	// ---- 内部 ----
	void _sample_scalp();          // 头模顶点 -> 头皮发根/法线/发长(插件内完成)
	void _build_strands();
	void _build_render_strands();
	void _collide_head(int i);
	void _collide_face(int i);   // 脸部挡板: 不让发丝垂到脸上
	void _collide_obstacle(int i);
	void _collide_body(int i);
	void _relax_chain(const PackedInt32Array &cand_a, const PackedInt32Array &cand_b, int passes);

protected:
	static void _bind_methods();

public:
	XPBDHairNative();
	~XPBDHairNative();

	void build();
	void step(real_t delta, int substeps);
	void update_mesh();

	PackedVector3Array get_positions() const { return pos; }
	int get_guide_count() const { return strand_count; }
	// 实际落地发丝数(采样过滤后可能少于请求值)
	int get_render_count() const { return blend_k > 0 ? (int)(blend_idx.size() / blend_k) : 0; }
	int get_render_requested() const { return render_strands; }
	real_t get_tri_r() const { return 0.0f; }
	bool get_asleep() const { return false; }

	void set_strand_count(int v) { strand_count = v; }
	int get_strand_count() const { return strand_count; }
	void set_segment_count(int v) { segment_count = v; }
	int get_segment_count() const { return segment_count; }
	void set_hair_length(real_t v) { hair_length = v; }
	real_t get_hair_length() const { return hair_length; }
	void set_head_radius(real_t v) { head_radius = v; }
	real_t get_head_radius() const { return head_radius; }
	void set_head_center(const Vector3 &v) { head_center = v; }
	Vector3 get_head_center() const { return head_center; }
	void set_head_ellipsoid(const Vector3 &r) { head_radii = r; }
	Vector3 get_head_ellipsoid() const { return head_radii; }
	void set_damping(real_t v) { damping = v; }
	real_t get_damping() const { return damping; }
	void set_stiffness(real_t v) { stiffness = v; }
	real_t get_stiffness() const { return stiffness; }
	void set_root_stiffness(real_t v) { root_stiffness = v; }
	real_t get_root_stiffness() const { return root_stiffness; }
	void set_friction(real_t v) { friction = v; }
	real_t get_friction() const { return friction; }
	void set_render_strands(int v) {
		if (v == render_strands) {
			return;
		}
		render_strands = v;
		blend_idx.clear(); // 使插值权重失效, 下次 update_mesh 重建
		blend_w.clear();
	}
	int get_render_strands() const { return render_strands; }
	void set_strand_width(real_t v) { strand_width = v; }
	real_t get_strand_width() const { return strand_width; }
	void set_tip_width(real_t v) { tip_width = v; }
	real_t get_tip_width() const { return tip_width; }
	void set_root_color(const Color &v) { root_color = v; }
	Color get_root_color() const { return root_color; }
	void set_tip_color(const Color &v) { tip_color = v; }
	Color get_tip_color() const { return tip_color; }
	void set_albedo(const Color &v) { albedo = v; }
	Color get_albedo() const { return albedo; }
	void set_material(const Ref<Material> &m) { material = m; }
	Ref<Material> get_material() const { return material; }
	Ref<ArrayMesh> get_mesh() const { return mesh; }

	void set_obstacle(bool on, const Vector3 &c, real_t r) { obstacle_on = on; obstacle_c = c; obstacle_r = r; }
	bool get_obstacle_on() const { return obstacle_on; }
	Vector3 get_obstacle_c() const { return obstacle_c; }
	real_t get_obstacle_r() const { return obstacle_r; }
	void set_head_moving(bool v) { head_moving = v; }
	void set_wind(const Vector3 &v) { wind = v; }
	Vector3 get_wind() const { return wind; }
	void set_model_transform(const Vector3 &origin, real_t yaw_deg);
	void translate_follow(const Vector3 &delta);
	Vector3 get_model_origin() const { return model_origin; }
	real_t get_model_yaw() const { return model_yaw; }
	void set_body_collision(bool on, const Vector3 &a, const Vector3 &b, real_t r) {
		body_on = on; body_a = a; body_b = b; body_r = r;
	}

	// ---- 真实头皮发根(HS2 捏人角色) ----
	// 交进头模顶点云(静止空间即可, 调用方每帧用装配矩阵把它映射到世界)。
	// 之后 build() 会在插件内部完成: 球坐标网格采样 -> 面部区剔除 -> 最远点采样
	// 选取 strand_count 根发束 -> 按所处区域分配发长(刘海/头顶/颈后)。
	void set_scalp_mesh(const PackedVector3Array &verts);
	bool get_scalp_ready() const { return scalp_on; }
	// 采样出的发根(静止空间)与头心(静止空间): 调用方拿它做每帧的骨骼跟随变换
	PackedVector3Array get_scalp_roots() const { return scalp_rest; }
	PackedVector3Array get_scalp_normals() const { return scalp_normals; }
	PackedVector3Array get_scalp_dirs() const { return scalp_dir; }
	PackedFloat32Array get_scalp_lengths() const { return strand_len; }
	Vector3 get_scalp_center() const { return scalp_center; }

	// 头皮采样参数
	void set_face_dir(const Vector3 &v) { face_axis = v; }
	Vector3 get_face_dir() const { return face_axis; }
	// 脸部挡板: pt 在颜面前方, nrm 指向脑后, min_y 以下生效, r 为脸宽半径
	void set_face_guard(bool on, const Vector3 &pt, const Vector3 &nrm, real_t min_y, real_t r) {
		face_guard_on = on;
		face_pt = pt;
		face_nrm = nrm.length_squared() > 1e-8f ? nrm.normalized() : Vector3(0, 0, -1);
		face_min_y = min_y;
		face_guard_r = r;
	}
	bool get_face_guard_on() const { return face_guard_on; }
	void set_scalp_grid(int rows, int cols) { scalp_rows = rows < 3 ? 3 : rows; scalp_cols = cols < 4 ? 4 : cols; }
	void set_scalp_lengths(real_t top, real_t bottom, real_t bang, real_t jitter) {
		len_top = top; len_bottom = bottom; bang_len = bang; len_jitter = jitter;
	}
	void set_scalp_region(real_t face_phi_v, real_t face_z_v, real_t bang_z_v) {
		face_phi = face_phi_v; face_z = face_z_v; bang_z = bang_z_v;
	}
	void set_render_clamp(real_t min_y, real_t snap) { render_min_y = min_y; render_snap = snap; }
	real_t get_render_min_y() const { return render_min_y; }
	void clear_scalp_roots() { scalp_on = false; scalp_rest.clear(); scalp_normals.clear(); skel_prev.clear(); skel_vel = Vector3(); }
	bool get_scalp_on() const { return scalp_on; }
	int get_scalp_count() const { return scalp_rest.size(); }
	// 每帧由骨架驱动: 发根贴着头皮走, 发丝按骨骼线速度继承惯性。
	void follow_skeleton(const PackedVector3Array &root_now, const Vector3 &head_now, real_t dt);
	bool get_body_on() const { return body_on; }
	Vector3 get_body_a() const { return body_a; }
	Vector3 get_body_b() const { return body_b; }
	real_t get_body_r() const { return body_r; }

	// 布间/物体接口(与布料一致的语义, 供场景统一调度)
	void reset_wind() {}
};

} // namespace godot

#endif
