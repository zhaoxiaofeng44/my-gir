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
	Vector3 head_center = Vector3(0, 0.2f, 0);
	Vector3 head_up = Vector3(0, 1, 0);

	// ---- 物理 ----
	real_t gravity_scale = 1.0f;
	real_t damping = 0.06f;
	real_t stiffness = 0.7f;     // 弯曲刚度 0..1 (映射到弯曲约束刚性)
	real_t root_stiffness = 0.9f; // 靠发根处额外刚性(发根更硬)
	real_t collide_radius = 0.008f; // 发丝碰撞半径(m)
	real_t friction = 0.4f;

	// ---- 插值渲染 ----
	int render_strands = 240; // 渲染发丝数
	real_t strand_width = 0.006f;
	real_t tip_width = 0.0015f;
	Color root_color = Color(0.16f, 0.09f, 0.05f);
	Color tip_color = Color(0.45f, 0.30f, 0.16f);
	Color albedo = Color(0.45f, 0.35f, 0.25f);

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

	// ---- 内部 ----
	void _build_strands();
	void _build_render_strands();
	void _collide_head(int i);
	void _collide_obstacle(int i);
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
	int get_render_count() const { return render_strands; }
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
	void set_damping(real_t v) { damping = v; }
	real_t get_damping() const { return damping; }
	void set_stiffness(real_t v) { stiffness = v; }
	real_t get_stiffness() const { return stiffness; }
	void set_render_strands(int v) { render_strands = v; }
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

	// 布间/物体接口(与布料一致的语义, 供场景统一调度)
	void reset_wind() {}
};

} // namespace godot

#endif
