# hs2/hair_native.gd — 把「布料/头发测试场景」的 C++ XPBD 头发接到 HS2 捏人角色
#
# 本文件**不含任何头发物理/采样算法**：求解、头皮发根采样、发长分配、
# 面部区剔除、渲染发丝插值全部在 GDExtension 里完成
# (src/cloth/xpbd_hair_native.cpp，与 scenes/cloth_hair_test_suite.tscn 用的同一个类)。
#
# 这里只做三件"胶水"的事：
#   1) 把合并骨架上的头模顶点云交给插件 (静止空间)
#   2) 每帧算出 A = S · pose(头骨) · rest(头骨)⁻¹，用 A 把发根/头心映射到世界下发给插件
#      —— 发根贴着头皮走，位移速度给粒子当惯性(甩头/走动时头发会甩)
#   3) 把插件生成的发丝网格挂到场景里
#
# 原来的 hs2/hair_system.gd(自实现 GDScript 发链)已删除。

class_name HS2HairNative
extends Node3D

const HEAD_BONE := "cf_J_Head_s"

# ---- 头皮采样参数(直接转发给插件) ----
@export var scalp_rows := 15             # 头皮网格行(极角)
@export var scalp_cols := 22             # 头皮网格列(方位角)
@export var len_top := 0.26              # 头顶/侧后发长(m)
@export var len_bottom := 0.62           # 颈后发长(m)
@export var bang_len := 0.13             # 刘海长度(m)
@export var face_dir := Vector3(0, 0, 1) # 脸朝向(装配空间): HS2 里 +Z 是脸
@export var len_jitter := 0.12           # 发长随机扰动

# ---- 物理发束 ----
@export var guide_count := 160           # 导向发束数(物理密度)
@export var segment_count := 10          # 每束段数
@export var render_strands := 4000       # 渲染发丝数(视觉密度)
@export var stiffness := 0.55
@export var root_stiffness := 0.90
@export var damping := 0.09
@export var friction := 0.4
@export var strand_width := 0.0038
@export var tip_width := 0.0009
@export var wind := Vector3.ZERO

var enabled := true

var _sk: Skeleton3D
var _native: XPBDHairNative
var _mesh: MeshInstance3D
var _head := -1
var _head_rest_inv := Transform3D()
var _head_verts := PackedVector3Array()  # 头模顶点云(静止空间)
var _root_rest := PackedVector3Array()   # 插件采样出的发根(静止空间)
var _scalp_center := Vector3.ZERO
var _ready_ok := false

func native_node() -> XPBDHairNative:
	return _native

func mesh_node() -> MeshInstance3D:
	return _mesh

func stats() -> Dictionary:
	if _native == null:
		return {}
	return {
		"guides": _native.get_guide_count(),
		"render": _native.get_render_count(),
		"particles": _native.get_positions().size(),
		"scalp": _native.get_scalp_count(),
	}

# =====================================================================
# 构建
# =====================================================================
func build(skeleton: Skeleton3D, parts: Dictionary) -> bool:
	_sk = skeleton
	_ready_ok = false
	if _sk == null or not ClassDB.class_exists("XPBDHairNative"):
		push_warning("[HairNative] 缺少骨架或 C++ 扩展未注册")
		return false
	_head = _sk.find_bone(HEAD_BONE)
	if _head < 0:
		_head = _sk.find_bone("cf_J_Head")
	if _head < 0:
		push_warning("[HairNative] 找不到头骨")
		return false
	_head_rest_inv = _sk.get_bone_global_rest(_head).affine_inverse()

	# 头模顶点云(静止空间) -> 插件自己采头皮
	var verts := _head_mesh_points(parts.get("head"))
	if verts.size() < 32:
		push_warning("[HairNative] 头模顶点不足")
		return false

	_native = XPBDHairNative.new()
	_native.name = "HairNativePhysics"
	_native.top_level = true
	add_child(_native)
	# 采样/物理参数
	_native.set_scalp_grid(scalp_rows, scalp_cols)
	_native.set_scalp_lengths(len_top, len_bottom, bang_len, len_jitter)
	_native.set_segment_count(segment_count)
	_native.set_strand_count(guide_count)
	_native.set_stiffness(stiffness)
	_native.set_root_stiffness(root_stiffness)
	_native.set_damping(damping)
	_native.set_friction(friction)
	_native.set_strand_width(strand_width)
	_native.set_tip_width(tip_width)
	_native.set_render_strands(render_strands)
	_head_verts = verts
	_native.set_scalp_mesh(verts)
	# 脸朝向(装配空间): HS2 装配空间里 +Z 就是脸的方向(见 hair_system 的 n.z<-0.9 后脑过滤)。
	# 注意**不要**用骨骼 rest 位置去推: 各部件 glb 的骨骼 rest 并不在同一个空间里
	# (那正是 hs2_rig_merge 要合并的原因), 拿它和头模顶点质心相减会把前后判反。
	_native.set_face_dir(face_dir)
	# 头部椭球碰撞: 用头模顶点的分位数半轴(真实头不是球)
	_native.set_head_ellipsoid(_head_radii(verts))
	_native.build()
	# 注意: 头心必须在 build() **之后**取 —— 采样发生在 build() 里
	var A := _assembly()
	var hc := _native.get_scalp_center()
	_native.set_head_center(A * hc)
	# 身体胶囊: 颈 -> 腰(挡住长发穿身体)
	_native.set_body_collision(true, A * Vector3(0, hc.y - 0.24, 0), A * Vector3(0, hc.y - 1.10, 0), 0.20)
	# 脸部挡板: 眉毛高度以下, 脸前方的发丝一律推回脑后(否则物理一稳定就糊脸)
	var hb: Dictionary = _head_y_range()
	var brows: float = hb["max"] - (hb["max"] - hb["min"]) * 0.42   # 约在眼睛/眉高
	var fwd: Vector3 = _native.get_face_dir()
	var ngn: Vector3 = A.basis * Vector3(fwd.x, 0.0, fwd.z)
	ngn.y = 0.0
	if ngn.length_squared() < 1e-8:
		ngn = Vector3(0, 0, -1)
	ngn = -ngn.normalized()            # 指向脑后
	var fpt: Vector3 = A * Vector3(hc.x + fwd.x * 0.055, 0.0, hc.z + fwd.z * 0.055)
	fpt.y = 0.0
	_native.set_face_guard(true, Vector3(fpt.x, brows, fpt.z), ngn, brows, 0.062)
	_native.set_material(_make_material())

	_root_rest = _native.get_scalp_roots()
	_scalp_center = _native.get_scalp_center()
	if _root_rest.is_empty():
		push_warning("[HairNative] 头皮采样为空")
		return false

	# 发根初值: 用当前装配矩阵把静止发根映射到世界
	var w := PackedVector3Array()
	w.resize(_root_rest.size())
	for i in _root_rest.size():
		w.set(i, A * _root_rest[i])
	_native.follow_skeleton(w, A * _scalp_center, 1.0 / 60.0)

	_mesh = MeshInstance3D.new()
	_mesh.name = "HairNativeMesh"
	_mesh.top_level = true
	add_child(_mesh)
	_mesh.mesh = _native.get_mesh()

	_ready_ok = true
	print("[HairNative] 导向发束 %d / 渲染发丝 %d / 段 %d" % [
		_native.get_guide_count(), _native.get_render_count(), segment_count])
	return true

# =====================================================================
# 每帧步进
# =====================================================================
func step(dt: float) -> void:
	if not _ready_ok or _native == null or not enabled:
		return
	_native.set_wind(wind)
	var A := _assembly()
	var n := _root_rest.size()
	var w := PackedVector3Array()
	w.resize(n)
	for i in n:
		w.set(i, A * _root_rest[i])
	_native.follow_skeleton(w, A * _scalp_center, dt)
	_native.step(dt, 3)
	_native.update_mesh()
	if _mesh:
		_mesh.mesh = _native.get_mesh()

# 装配矩阵: 静止空间 -> 世界
func _assembly() -> Transform3D:
	return _sk.global_transform * _sk.get_bone_global_pose(_head) * _head_rest_inv

func world_roots() -> PackedVector3Array:
	if not _ready_ok:
		return PackedVector3Array()
	var A := _assembly()
	var out := PackedVector3Array()
	out.resize(_root_rest.size())
	for i in _root_rest.size():
		out.set(i, A * _root_rest[i])
	return out

# =====================================================================
# 头模顶点(静止空间: 用 rest 骨架蒙皮, 与插件的采样口径一致)
# =====================================================================
func _head_mesh_points(mi: MeshInstance3D) -> PackedVector3Array:
	var out := PackedVector3Array()
	if mi == null or mi.mesh == null:
		return out
	var nb: int = mini(_sk.get_bone_count(), mi.skin.get_bind_count()) if mi.skin != null else 0
	var mats := []
	if nb > 0:
		mats.resize(nb)
		for b in nb:
			mats[b] = _sk.get_bone_global_rest(b) * mi.skin.get_bind_pose(b)
	for surf in mi.mesh.get_surface_count():
		var arr: Array = mi.mesh.surface_get_arrays(surf)
		var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
		for i in v.size():
			var acc := Vector3.ZERO
			var ws := 0.0
			if nb > 0 and bo.size() >= (i + 1) * 4:
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					var bi: int = bo[i * 4 + k]
					if bi >= nb:
						continue
					acc += (mats[bi] as Transform3D) * v[i] * w
					ws += w
			out.append((acc / ws) if ws > 0.0 else v[i])
	return out

## 头模顶点云的 y 范围(装配空间): 用来定"眉毛"高度
func _head_y_range() -> Dictionary:
	var lo := INF
	var hi := -INF
	for v in _head_verts:
		lo = minf(lo, v.y)
		hi = maxf(hi, v.y)
	if lo > hi:
		return {"min": 1.56, "max": 1.77}
	return {"min": lo, "max": hi}

func _head_radii(verts: PackedVector3Array) -> Vector3:
	var c := Vector3.ZERO
	for v in verts:
		c += v
	c = c / float(verts.size())
	var ax := PackedFloat32Array()
	var ay := PackedFloat32Array()
	var az := PackedFloat32Array()
	for v in verts:
		var d: Vector3 = (v - c).abs()
		ax.append(d.x)
		ay.append(d.y)
		az.append(d.z)
	return Vector3(_pct(ax, 0.97) * 0.94, _pct(ay, 0.97) * 0.94, _pct(az, 0.97) * 0.94)

func _pct(a: PackedFloat32Array, q: float) -> float:
	var s := Array(a)
	s.sort()
	if s.is_empty():
		return 0.1
	return s[clampi(int(float(s.size() - 1) * q), 0, s.size() - 1)]

# =====================================================================
# 材质 / 参数
# =====================================================================
func _make_material() -> Material:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	m.vertex_color_use_as_albedo = true
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	m.roughness = 0.34
	m.specular_mode = BaseMaterial3D.SPECULAR_SCHLICK_GGX
	m.metallic_specular = 0.42
	m.emission_enabled = true
	m.emission = Color(0.10, 0.06, 0.035)
	return m

func set_stiffness(v: float) -> void:
	stiffness = v
	if _native:
		_native.set_stiffness(v)

func set_damping(v: float) -> void:
	damping = v
	if _native:
		_native.set_damping(v)

func set_render_strands(n: int) -> void:
	render_strands = n
	if _native:
		_native.set_render_strands(n)
		_native.update_mesh()
		if _mesh:
			_mesh.mesh = _native.get_mesh()

func set_wind(v: Vector3) -> void:
	wind = v
	if _native:
		_native.set_wind(v)

func reset_physics() -> void:
	if _native:
		_native.build()
		_root_rest = _native.get_scalp_roots()
		_scalp_center = _native.get_scalp_center()

# =====================================================================
# 蒙皮几何工具(供布料定位使用)
# =====================================================================
func _skinned_points(mi: MeshInstance3D) -> PackedVector3Array:
	var out := PackedVector3Array()
	if mi == null or mi.mesh == null or mi.skin == null:
		return out
	var nb: int = mini(_sk.get_bone_count(), mi.skin.get_bind_count())
	var S: Transform3D = _sk.global_transform
	var mats := []
	mats.resize(nb)
	for b in nb:
		mats[b] = S * _sk.get_bone_global_pose(b) * mi.skin.get_bind_pose(b)
	for surf in mi.mesh.get_surface_count():
		var arr: Array = mi.mesh.surface_get_arrays(surf)
		var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		var bo: PackedInt32Array = arr[Mesh.ARRAY_BONES]
		var we: PackedFloat32Array = arr[Mesh.ARRAY_WEIGHTS]
		var i := 0
		while i < v.size():
			var acc := Vector3.ZERO
			var ws := 0.0
			if bo.size() >= (i + 1) * 4:
				for k in 4:
					var w: float = we[i * 4 + k]
					if w <= 0.0:
						continue
					var bi: int = bo[i * 4 + k]
					if bi >= nb:
						continue
					acc += (mats[bi] as Transform3D) * v[i] * w
					ws += w
			out.append((acc / ws) if ws > 0.0 else v[i])
			i += 3
	return out

## 找"腰": 在身高 45%~72% 这一段里, 水平半径最小的那一层。
func waist_info(body: MeshInstance3D) -> Dictionary:
	var pts := _skinned_points(body)
	if pts.is_empty():
		return {"y": 1.0, "r": 0.16, "hip_r": 0.18, "h": 1.7, "floor": 0.0}
	var lo := INF
	var hi := -INF
	for p in pts:
		lo = minf(lo, p.y)
		hi = maxf(hi, p.y)
	var h: float = maxf(hi - lo, 0.5)
	var y0: float = lo + h * 0.45
	var y1: float = lo + h * 0.72
	var band: float = h * 0.012
	var best_y := lo + h * 0.6
	var best_r := 1e9
	var hip_r := 0.0
	var y := y0
	while y <= y1:
		var rr := 0.0
		for p in pts:
			if absf(p.y - y) < band:
				rr = maxf(rr, sqrt(p.x * p.x + p.z * p.z))
		if rr > 0.02:
			if rr < best_r:
				best_r = rr
				best_y = y
			if y < lo + h * 0.58:
				hip_r = maxf(hip_r, rr)
		y += band * 0.5
	if best_r > 1e8:
		best_r = 0.15
	if hip_r <= 0.0:
		hip_r = best_r * 1.15
	return {"y": best_y, "r": best_r, "hip_r": hip_r, "h": h, "floor": lo}
