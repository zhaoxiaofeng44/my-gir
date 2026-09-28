class_name XPBDClothNativeAdapter
extends Node3D
## XPBD 布料 —— C++ 后端适配器
## 对外暴露与 scripts/xpbd_cloth.gd 完全相同的接口(同名属性/方法), 内部把热路径
## (约束求解/自碰/物体碰撞/布间)委托给 GDExtension 的 XPBDClothNative。
## 这样测试场景/压测工具无需修改即可 A/B 对比两套实现。

var native: XPBDClothNative = null
var _colliders: Array = []

# ---- 与 GDScript 版同名的参数(写入时同步到 C++) ----
var albedo := Color(0.85, 0.3, 0.35)
var colliders: Array = [] : set = _set_colliders_backing
var enable_object_collision: bool = true : set = _set_eoc
var enable_self_collision: bool = true : set = _set_esc
var ground_y: float = -1.6 : set = _set_gy
var gravity := Vector3(0, -9.8, 0) : set = _set_grav
var cloth_thickness: float = 0.035 : set = _set_thick
var friction: float = 0.85 : set = _set_fric
var max_stretch: float = 1.06 : set = _set_ms
var compliance_shear: float = 0.00002 : set = _set_cs
var compliance_bend: float = 0.0003 : set = _set_cb
var air_drag: float = 0.005 : set = _set_ad
var grab_stiffness: float = 0.6 : set = _set_gs

# ---- 由 C++ 转发的只读状态 ----
var cols: int = 24 : set = _set_cols
var rows: int = 24 : set = _set_rows
var cloth_size := Vector2(2.0, 2.0) : set = _set_size
var spawn_origin := Vector3.ZERO : set = _set_origin
var vertical: bool = false : set = _set_vertical
var yaw: float = 0.0 : set = _set_yaw
var iterations: int = 1 : set = _set_iter

var mesh_inst: MeshInstance3D
var material: StandardMaterial3D

func _set_colliders_backing(v: Array) -> void:
	colliders = v
	_colliders = v

func _set_eoc(v): enable_object_collision = v; if native: native.enable_object_collision = v
func _set_esc(v): enable_self_collision = v; if native: native.enable_self_collision = v
func _set_gy(v): ground_y = v; if native: native.ground_y = v
func _set_grav(v): gravity = v; if native: native.gravity = v
func _set_thick(v): cloth_thickness = v; if native: native.cloth_thickness = v
func _set_fric(v): friction = v; if native: native.friction = v
func _set_ms(v): max_stretch = v; if native: native.max_stretch = v
func _set_cs(v): compliance_shear = v; if native: native.compliance_shear = v
func _set_cb(v): compliance_bend = v; if native: native.compliance_bend = v
func _set_ad(v): air_drag = v; if native: native.air_drag = v
func _set_gs(v): grab_stiffness = v; if native: native.grab_stiffness = v
func _set_cols(v): cols = v; if native: native.cols = v
func _set_rows(v): rows = v; if native: native.rows = v
func _set_size(v): cloth_size = v; if native: native.cloth_size = v
func _set_origin(v): spawn_origin = v; if native: native.spawn_origin = v
func _set_vertical(v): vertical = v; if native: native.vertical = v
func _set_yaw(v): yaw = v; if native: native.yaw = v
func _set_iter(v): iterations = v; if native: native.iterations = v

func _ready() -> void:
	native = XPBDClothNative.new()
	native.name = "Native"
	add_child(native)

func build() -> void:
	native.cols = cols
	native.rows = rows
	native.cloth_size = cloth_size
	native.spawn_origin = spawn_origin
	native.vertical = vertical
	native.yaw = yaw
	native.iterations = iterations
	native.gravity = gravity
	native.air_drag = air_drag
	native.cloth_thickness = cloth_thickness
	native.friction = friction
	native.max_stretch = max_stretch
	native.compliance_shear = compliance_shear
	native.compliance_bend = compliance_bend
	native.enable_object_collision = enable_object_collision
	native.enable_self_collision = enable_self_collision
	native.ground_y = ground_y
	native.grab_stiffness = grab_stiffness
	native.build()
	if material == null:
		material = StandardMaterial3D.new()
		material.cull_mode = BaseMaterial3D.CULL_DISABLED
		material.roughness = 0.7
	material.albedo_color = albedo
	native.set_material(material)
	if mesh_inst == null:
		mesh_inst = MeshInstance3D.new()
		add_child(mesh_inst)
	# 注册表由 C++ 的 build() 自动维护(static 方法不可从 GDScript 调用)
	_sync_collider()

func _sync_collider() -> void:
	# 把 XPBDCollider(GDScript) 的形状与变换下发给 C++
	if colliders.is_empty() or colliders[0] == null:
		native.set_collider_shapes(PackedInt32Array(), PackedVector3Array(), PackedVector3Array(),
			PackedVector3Array(), PackedFloat32Array(), PackedVector3Array(), Transform3D())
		return
	var col = colliders[0]
	native.set_collider_shapes(col.pt, col.poff, col.pa, col.pb, col.pr, col.phalf, col.root.global_transform)

# ---- 子步阶段: 直接转发 ----
func sub_begin() -> void:
	native.sub_begin()

func sub_integrate(h: float, wind: Vector3) -> void:
	native.sub_integrate(h, wind)

func sub_solve(h: float) -> void:
	native.sub_solve(h)

func begin_contacts() -> void:
	native.begin_contacts()

func self_collision() -> void:
	native.self_collision()

func object_collision() -> void:
	_sync_collider()
	native.object_collision()

func final_push_out() -> void:
	_sync_collider()
	native.final_push_out()

func sub_recover(h: float) -> void:
	native.sub_recover(h)

# ---- 状态访问(返回 C++ 内部的 Packed 数组视图) ----
var pos: PackedVector3Array : get = _get_pos
var prev: PackedVector3Array : get = _get_prev
var vel: PackedVector3Array : get = _get_vel
var pinned: PackedByteArray : get = _get_pinned
var c_ia: PackedInt32Array : get = _get_cia
var c_ib: PackedInt32Array : get = _get_cib
var c_rest: PackedFloat32Array : get = _get_crest
var c_alive: PackedByteArray : get = _get_calive
var c_type: PackedInt32Array : get = _get_ctype
var max_vel2: float : get = _get_mv2, set = _set_mv2
var contact_deficit: float : get = _get_cd, set = _set_cd
var tri_r: float : get = _get_tr
var _asleep: bool : get = _get_asleep

func _get_pos() -> PackedVector3Array: return native.get_positions()
func _get_prev() -> PackedVector3Array: return native.get_positions()
func _get_vel() -> PackedVector3Array: return native.get_velocities()
func _get_pinned() -> PackedByteArray: return native.get_pinned_flags()
func _get_cia() -> PackedInt32Array: return native.get_constraint_ia()
func _get_cib() -> PackedInt32Array: return native.get_constraint_ib()
func _get_crest() -> PackedFloat32Array: return native.get_constraint_rest()
func _get_calive() -> PackedByteArray: return native.get_constraint_alive()
func _get_ctype() -> PackedInt32Array: return native.get_constraint_type()
func _get_mv2() -> float: return native.get_max_vel2()
func _set_mv2(v: float) -> void: native.set_max_vel2(v)
func _get_cd() -> float: return native.get_contact_deficit()
func _set_cd(v: float) -> void: native.set_contact_deficit(v)
func _get_tr() -> float: return native.get_tri_r()
func _get_asleep() -> bool: return native.get_asleep()

# ---- 抓取 ----
var grabbing: bool : get = _get_grabbing, set = _set_grabbing
var grab_id: int : get = _get_gid, set = _set_gid
var grab_target: Vector3 : get = _get_gt, set = _set_gt
var grab_depth: float = 0.0 : set = _set_gd

func _get_grabbing() -> bool: return native.grabbing
func _set_grabbing(v: bool) -> void: native.grabbing = v
func _get_gid() -> int: return native.grab_id
func _set_gid(v: int) -> void: native.grab_id = v
func _get_gt() -> Vector3: return native.grab_target
func _set_gt(v: Vector3) -> void: native.grab_target = v
func _set_gd(v: float) -> void: grab_depth = v

func start_grab(i: int, ro: Vector3, rn: Vector3) -> void:
	native.grab_id = i
	native.grabbing = true
	native.grab_target = native.get_positions()[i]
	grab_depth = (native.get_positions()[i] - ro).dot(rn)

func release_grab(substeps: int) -> void:
	native.release_grab(substeps)

func pick_vertex(ro: Vector3, rn: Vector3) -> int:
	return native.pick_vertex(ro, rn)

# ---- 审计/渲染 ----
func penetration_count() -> int:
	return native.penetration_count()

func _self_deficit() -> float:
	return native.self_deficit()

func update_mesh() -> void:
	native.update_mesh()
	if mesh_inst:
		mesh_inst.mesh = native.get_mesh()

func set_positions(p: PackedVector3Array) -> void:
	native.set_positions(p)

func set_velocities(v: PackedVector3Array) -> void:
	native.set_velocities(v)
