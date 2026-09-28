class_name XPBDCloth
extends Node3D
## XPBD 布料实例: 粒子 + 距离/剪切/弯曲约束 + 自碰撞 + 与 XPBDCollider 物体碰撞
## 由场景管理器编排子步: sub_begin -> sub_integrate -> sub_solve -> [self_collision] -> object_collision -> sub_recover
## 支持水平落体(sheet)与垂直挂帘(curtain, 顶行 pinned)两种生成方式

var cols: int = 24
var rows: int = 24
var cloth_size: Vector2 = Vector2(2.0, 2.0)
var spawn_origin: Vector3 = Vector3.ZERO
var vertical: bool = false
var yaw: float = 0.0            # 布料整体绕Y摆放
var albedo := Color(0.85, 0.3, 0.35)

# ---- 物理参数(管理器下发) ----
var iterations: int = 1
var gravity := Vector3(0, -9.8, 0)
var damping: float = 0.02
var air_drag: float = 0.005
var tear_dist_factor: float = 4.0
var max_stretch: float = 1.06
var compliance_struct := 0.0
var compliance_shear := 0.00002
var compliance_bend := 0.0003
var cloth_thickness: float = 0.035
var friction: float = 0.85
var collision_margin: float = 0.012
var enable_object_collision: bool = true
var enable_self_collision: bool = true
var ground_y: float = -1.6
var colliders: Array = []       # [XPBDCollider]

# ---- 状态 ----
var pos: PackedVector3Array
var prev: PackedVector3Array
var vel: PackedVector3Array
var pinned: PackedByteArray
var c_ia: PackedInt32Array
var c_ib: PackedInt32Array
var c_rest: PackedFloat32Array
var c_alive: PackedByteArray
var c_type: PackedInt32Array
var mesh: ArrayMesh
var mesh_inst: MeshInstance3D
var material: StandardMaterial3D
var grabbing: bool = false
var grab_id: int = -1
var grab_target: Vector3 = Vector3.ZERO
var grab_depth: float = 0.0
var grab_stiffness: float = 0.6
var grab_max_speed: float = 12.0   # 抓取点每子步最大位移(m/s): 防止瞬移式极限拉扯撕裂布料
var _hash: Dictionary = {}
var _alphas: Array[float] = [0.0, 0.0, 0.0]
# 静态拓扑: 每顶点关联的三角形(3个顶点索引/三角形) — 供布间面级碰撞复用, build() 时构建
var tri_r: float = 0.0   # 三角形外接半径上界(跨布面级间距补偿用, build() 计算)
var vtx_tri_ofs: PackedInt32Array = PackedInt32Array()
var vtx_tri_ids: PackedInt32Array = PackedInt32Array()
var tri_verts: PackedInt32Array = PackedInt32Array()
var _pre_contact: PackedVector3Array = PackedVector3Array()
var _pushed: PackedByteArray = PackedByteArray()      # 硬接触(物体/地面/网格边): 速度投影
var _pushed_soft: PackedByteArray = PackedByteArray() # 软接触(自碰/布间): 保留推出速度(展开动力)
var max_vel2: float = 0.0   # 本帧最大速度平方: 管理器据此升级自碰/布间为每子步求解
var contact_deficit: float = 0.0   # 上帧自碰最深违例欠量: 堆叠受压时据此维持每子步接触求解

func build() -> void:
	c_ia = PackedInt32Array(); c_ib = PackedInt32Array()
	c_rest = PackedFloat32Array(); c_alive = PackedByteArray()
	c_type = PackedInt32Array()
	pinned = PackedByteArray()
	pos = PackedVector3Array(); prev = PackedVector3Array(); vel = PackedVector3Array()
	var rot := Basis.from_euler(Vector3(0, yaw, 0))
	var dx: float = cloth_size.x / (cols - 1)
	var dy: float = cloth_size.y / (rows - 1)
	for r in rows:
		for c in cols:
			var x: float = (float(c) / (cols - 1) - 0.5) * cloth_size.x
			var j := float((r * 13 + c * 7) % 10) * 0.002   # 微扰动打破对称
			var p: Vector3
			if vertical:
				# 挂帘: 局部 XY 平面, 顶行(r=0)在 origin 高度, 向下垂
				p = Vector3(x, -float(r) * dy + j * 0.5, 0.0)
			else:
				var z: float = (float(r) / (rows - 1) - 0.5) * cloth_size.y
				p = Vector3(x, j, z)
			p = spawn_origin + rot * p
			pos.append(p)
			prev.append(p)
			vel.append(Vector3.ZERO)
			pinned.append((1 if (vertical and r == 0) else 0))
	var idx := func(c: int, r: int) -> int: return r * cols + c
	for r in rows:
		for c in cols:
			var i: int = idx.call(c, r)
			if c + 1 < cols: _add_constraint(i, idx.call(c + 1, r), 0)
			if r + 1 < rows: _add_constraint(i, idx.call(c, r + 1), 0)
			if c + 1 < cols and r + 1 < rows:
				_add_constraint(i, idx.call(c + 1, r + 1), 1)
				_add_constraint(idx.call(c + 1, r), idx.call(c, r + 1), 1)
			if c + 2 < cols: _add_constraint(i, idx.call(c + 2, r), 2)
			if r + 2 < rows: _add_constraint(i, idx.call(c, r + 2), 2)
			# 对角弯曲对: 抑制沿对角线方向的自由折痕(行列弯曲对只管正交方向)
			if c + 2 < cols and r + 2 < rows:
				_add_constraint(i, idx.call(c + 2, r + 2), 2)
				_add_constraint(idx.call(c + 2, r), idx.call(c, r + 2), 2)
	_build_topology()
	if material == null:
		material = StandardMaterial3D.new()
		material.cull_mode = BaseMaterial3D.CULL_DISABLED
		material.roughness = 0.7
	material.albedo_color = albedo
	if mesh_inst == null:
		mesh_inst = MeshInstance3D.new()
		add_child(mesh_inst)
	_make_mesh()

# 预计算顶点->关联三角形 CSR 表(拓扑固定, 一次构建): 布间面级碰撞按需取样,避免全量哈希
func _build_topology() -> void:
	tri_r = 0.0
	tri_verts = PackedInt32Array()
	var counts := PackedInt32Array()
	counts.resize(pos.size())
	counts.fill(0)
	for r in rows - 1:
		for c in cols - 1:
			var a0: int = r * cols + c
			for tri in [[a0, a0 + cols, a0 + 1], [a0 + 1, a0 + cols, a0 + cols + 1]]:
				tri_verts.append(tri[0])
				tri_verts.append(tri[1])
				tri_verts.append(tri[2])
				counts[tri[0]] += 1
				counts[tri[1]] += 1
				counts[tri[2]] += 1
				# 三角形外接圆半径 R = abc/(4K) — 顶点间距补偿用的紧上界
				# (用最长边会保守 2 倍, 使跨布配对量暴涨 ~13 倍, 实测拖垮性能)
				var ea: float = pos[tri[0]].distance_to(pos[tri[1]])
				var eb: float = pos[tri[1]].distance_to(pos[tri[2]])
				var ec: float = pos[tri[2]].distance_to(pos[tri[0]])
				var area2: float = (pos[tri[1]] - pos[tri[0]]).cross(pos[tri[2]] - pos[tri[0]]).length()
				if area2 > 1e-12:
					tri_r = maxf(tri_r, ea * eb * ec / (2.0 * area2))
	vtx_tri_ofs.resize(pos.size() + 1)
	var acc := 0
	for i in pos.size():
		vtx_tri_ofs[i] = acc
		acc += counts[i]
	vtx_tri_ofs[pos.size()] = acc
	vtx_tri_ids.resize(acc)
	var cursor := PackedInt32Array()
	cursor.resize(pos.size())
	for t in tri_verts.size() / 3:
		for k in 3:
			var v: int = tri_verts[t * 3 + k]
			vtx_tri_ids[vtx_tri_ofs[v] + cursor[v]] = t
			cursor[v] += 1

func _add_constraint(ia: int, ib: int, ctype: int) -> void:
	c_ia.append(ia); c_ib.append(ib)
	c_rest.append(pos[ia].distance_to(pos[ib])); c_alive.append(1)
	c_type.append(ctype)

func _make_mesh() -> void:
	mesh = ArrayMesh.new()
	var verts := PackedVector3Array()
	var uvs := PackedVector2Array()
	for r in rows:
		for c in cols:
			verts.append(pos[r * cols + c])
			uvs.append(Vector2(float(c) / (cols - 1), float(r) / (rows - 1)))
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mesh.surface_set_material(0, material)
	mesh_inst.mesh = mesh

# ---------- 子步阶段 ----------
func sub_begin() -> void:
	# 抓取: 按刚度柔顺跟随(避免硬传送把布料塞进碰撞体)
	# 限速: 单子步位移不超过 grab_max_speed*h — 手速再快也不能瞬移布料(撕裂/极限拉扯的来源)
	if grabbing and grab_id >= 0:
		var to_t := grab_target - pos[grab_id]
		var ml := to_t.length()
		if ml > 1e-9:
			var step_max := grab_max_speed / 60.0 / 4.0
			if ml > step_max:
				pos[grab_id] += to_t * (step_max / ml) * clampf(grab_stiffness, 0.05, 1.0)
			else:
				pos[grab_id] = pos[grab_id].lerp(grab_target, clampf(grab_stiffness, 0.05, 1.0))

func sub_integrate(h: float, wind_vec: Vector3) -> void:
	var n := pos.size()
	for i in n:
		if pinned[i] == 1:
			continue
		prev[i] = pos[i]
		var v := vel[i] + gravity * h + wind_vec * h
		v *= (1.0 - damping)
		v -= v * v.length() * air_drag * h
		vel[i] = v
		var v2 := v.length_squared()
		if v2 > max_vel2:
			max_vel2 = v2
		pos[i] += v * h

func sub_solve(h: float) -> void:
	var n := pos.size()
	var nc := c_ia.size()
	_alphas[0] = compliance_struct
	_alphas[1] = compliance_shear
	_alphas[2] = compliance_bend
	var inv_h2 := 1.0 / (h * h)
	for _it in iterations:
		for k in nc:
			if c_alive[k] == 0:
				continue
			var ia: int = c_ia[k]
			var ib: int = c_ib[k]
			var rest: float = c_rest[k]
			var d := pos[ia] - pos[ib]
			var ln := d.length()
			if ln < 1e-6:
				continue
			if ln > rest * tear_dist_factor:
				c_alive[k] = 0
				continue
			var wa := 0.0 if (pinned[ia] == 1 or ia == grab_id) else 1.0
			var wb := 0.0 if (pinned[ib] == 1 or ib == grab_id) else 1.0
			var w := wa + wb
			if w == 0.0:
				continue
			var lam := -(ln - rest) / (ln * (w + _alphas[c_type[k]] * inv_h2))
			pos[ia] += d * lam * wa
			pos[ib] -= d * lam * wb
	_strain_limit()

# 应变限制(仅结构/剪切: 弯曲约束本身软, 限制它无意义且占1/3开销)
# 接触推出在约束求解之后还会移动顶点 -> final_push_out 末尾必须再跑一次, 否则静置残差
# 令实际拉伸比超过 max_stretch(实测 1.11 > 1.06)
func _strain_limit() -> void:
	var nc := c_ia.size()
	for k in nc:
		if c_alive[k] == 0 or c_type[k] == 2:
			continue
		var ia: int = c_ia[k]
		var ib: int = c_ib[k]
		var limit: float = c_rest[k] * max_stretch
		var d := pos[ia] - pos[ib]
		var ln := d.length()
		if ln <= limit:
			continue
		var wa := 0.0 if (pinned[ia] == 1 or ia == grab_id) else 1.0
		var wb := 0.0 if (pinned[ib] == 1 or ib == grab_id) else 1.0
		var w := wa + wb
		if w == 0.0:
			continue
		var lam := (ln - limit) / ln / w
		pos[ia] -= d * lam * wa
		pos[ib] += d * lam * wb

# 接触阶段开始: 快照位置(碰撞修正不计入速度, 消除"推出->获得速度->又飞起"的抖动反馈环)
func begin_contacts() -> void:
	_pre_contact = pos.duplicate()
	_pushed.resize(pos.size())
	_pushed.fill(0)
	_pushed_soft.resize(pos.size())
	_pushed_soft.fill(0)

func sub_recover(h: float) -> void:
	var n := pos.size()
	for i in n:
		if pinned[i] == 1 or i == grab_id:
			continue
		var v := (pos[i] - prev[i]) / h
		if _pushed[i] == 1 and _pushed_soft[i] == 0:
			# 硬接触(物体/地面/网格边)顶点的速度: 必须用"实际位移"算 —
			# 里面含地面摩擦对切向的衰减, 若改用修正前位置就会丢掉摩擦, 布料永远贴着表面滑走
			# (实测 10 秒后仍以 ~2m/s 整体滑移)。只去掉推出造成的法向外弹分量即可消除抖动。
			var corr := pos[i] - _pre_contact[i]
			var c2 := corr.length_squared()
			if c2 > 1e-12:
				var nrm := corr / sqrt(c2)
				var vn := v.dot(nrm)
				if vn > 0.0:
					v -= nrm * vn
		vel[i] = v

# ---------- 自碰撞 ----------
# 密集堆叠(揉团/重压)下单遍求解不足以抵消每帧再压入 -> 残差深穿透;
# 扫内后续推挤可把已解对重新压入(求解时欠量失真), 故每轮求解后以实测欠量决定是否加扫(上限6)
# 休眠: 连续3次检查干净且几乎无运动 => 跳过整个哈希构建(静置布料零成本), 运动即唤醒
var _sleep_checks: int = 0
var _asleep: bool = false
var _sleep_frame: int = 0
var _has_vel: bool = true   # vel[] 是否有效(子步求解期间为真; 非物理调用时置否)

func self_collision() -> void:
	if not enable_self_collision:
		return
	# 休眠判据必须与求解器自身的收敛门限一致(0.008)。用更严的值(0.001)会导致:
	# 求解器已收敛(残差 0.004)却永远不满足休眠 -> 每帧重建 6 轮哈希(实测 self=31ms/帧, FPS 1)
	if _asleep:
		# 休眠中: 静止则定期抽检欠量(重力/求解仍可能慢慢压拢折皱), 有欠量即唤醒
		if max_vel2 < 0.04:
			_sleep_frame += 1
			if _sleep_frame < 8:
				return
			_sleep_frame = 0
			if _self_deficit() < 0.008:
				return
		_asleep = false
		_sleep_checks = 0
	elif contact_deficit < 0.008 and max_vel2 < 0.04:
		_sleep_checks += 1
		if _sleep_checks >= 3:
			_asleep = true
			_sleep_frame = 0
			return
	else:
		_sleep_checks = 0
	var n := pos.size()
	# 自碰同样需要几何补偿: 折叠层的一个顶点可以扎进对侧折层的三角形内部, 而顶点间距
	# 仍 >= 2倍布厚。不补偿会出现"推开->落下"的永久抖振(实测 17 顶点悬空 3cm 抖动)。
	# 采样半径 = S_visual + R。哈希格仍用该值, 邻域仍是 1 圈, 成本不变。
	var min_d := cloth_thickness * 0.5 + tri_r
	var min_d2 := min_d * min_d
	var inv := 1.0 / min_d
	# 注: 曾尝试用"平坦数组+计数排序"替代 Dictionary[Array], 但 GDScript 中手工归并/二分
	# 远比 C++ 实现的 Dictionary 慢(实测 39.7ms vs 13.9ms), 已回退。
	# 保留 Dictionary: 它是 GDScript 下最快的格索引。
	for sweep in 6:
		_hash.clear()
		for i in n:
			var p := pos[i]
			var key := (int(floor(p.x * inv)) + 512) << 20 | (int(floor(p.y * inv)) + 512) << 10 | (int(floor(p.z * inv)) + 512)
			if _hash.has(key):
				_hash[key].append(i)
			else:
				_hash[key] = [i]
		var worst := 0.0
		var pushed := false
		for i in n:
			var p := pos[i]
			var cx := int(floor(p.x * inv)) + 512
			var cy := int(floor(p.y * inv)) + 512
			var cz := int(floor(p.z * inv)) + 512
			var ri := i / cols
			var ci := i % cols
			for off in _OFFS:
				var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
				if _hash.has(key):
					var bucket: Array = _hash[key]
					for jj in bucket.size():
						var j: int = bucket[jj]
						if j <= i:
							continue
						# 网格邻域内的顶点对有约束相连, 跳过(替代O(n²)邻接掩码); 折叠层(dr>2)必须求解
						var dr: int = absi(j / cols - ri)
						var dc: int = absi(j % cols - ci)
						if dr <= 2 and dc <= 2:
							continue
						var d := p - pos[j]
						var d2 := d.length_squared()
						if d2 >= min_d2 or d2 < 1e-12:
							continue
						var ln := sqrt(d2)
						var deficit := min_d - ln
						if deficit > worst:
							worst = deficit
						var wa := 0.0 if (pinned[i] == 1 or i == grab_id) else 1.0
						var wb := 0.0 if (pinned[j] == 1 or j == grab_id) else 1.0
						var w := wa + wb
						if w == 0.0:
							continue
						var nrm := d / ln
						var push := deficit / w
						pos[i] += nrm * push * wa
						pos[j] -= nrm * push * wb
						# 法向速度阻尼: 折层在分离距离附近来回弹跳会永不静止(实测 11 顶点
						# 长期 0.7m/s, 导致布料无法休眠、每帧全量自碰)。对接近中的法向分量做耗散。
						if _has_vel:
							var vrel := vel[i] - vel[j]
							var vn := vrel.dot(nrm)
							if vn < 0.0:
								var dv := nrm * (vn * 0.5)
								if wa > 0.0:
									vel[i] -= dv * wa
								if wb > 0.0:
									vel[j] += dv * wb
						p = pos[i]
						pushed = true
						_pushed_soft[i] = 1
						_pushed_soft[j] = 1
		if worst < 0.005 and not pushed:
			contact_deficit = worst
			break
		# 求解后实测(重建哈希, 只测不推): 实测干净才收工
		# 收敛门限 0.008: 残余欠量 < 8mm 且本轮无新推挤才停 — 静置折皱不再滞留 1.5cm 量级的夹层
		contact_deficit = _self_deficit()
		if contact_deficit < 0.008:
			break

# 实测当前自碰最深欠量(不推顶点); 与审计同口径(不跳pinned, 仅排除网格邻接)
func _self_deficit() -> float:
	var n := pos.size()
	var min_d := cloth_thickness * 2.0
	var min_d2 := min_d * min_d
	var inv := 1.0 / min_d
	_hash.clear()
	for i in n:
		var p := pos[i]
		var key := (int(floor(p.x * inv)) + 512) << 20 | (int(floor(p.y * inv)) + 512) << 10 | (int(floor(p.z * inv)) + 512)
		if _hash.has(key):
			_hash[key].append(i)
		else:
			_hash[key] = [i]
	var worst := 0.0
	for i in n:
		var p := pos[i]
		var cx := int(floor(p.x * inv)) + 512
		var cy := int(floor(p.y * inv)) + 512
		var cz := int(floor(p.z * inv)) + 512
		var ri := i / cols
		var ci := i % cols
		for off in _OFFS:
			var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
			if _hash.has(key):
				var bucket: Array = _hash[key]
				for jj in bucket.size():
					var j: int = bucket[jj]
					if j <= i:
						continue
					var dr: int = absi(j / cols - ri)
					var dc: int = absi(j % cols - ci)
					if dr <= 2 and dc <= 2:
						continue
					var d := p - pos[j]
					var d2 := d.length_squared()
					if d2 < min_d2 and d2 > 1e-12:
						var deficit := min_d - sqrt(d2)
						if deficit > worst:
							worst = deficit
	return worst

# 2 圈邻域(125 格): 检测半径 > 格边长时使用(自碰/布间几何补偿后即此情形)
static func make_offs2() -> Array:
	var a: Array = []
	for x in range(-2, 3):
		for y in range(-2, 3):
			for z in range(-2, 3):
				a.append(Vector3i(x, y, z))
	return a


# 27 邻域偏移: 必须含 (0,0,0) 中心格 — 落入同一格的顶点对(折叠层常态)否则永远检测不到
const _OFFS: Array = [Vector3i(0,0,0),Vector3i(-1,-1,-1),Vector3i(-1,-1,0),Vector3i(-1,-1,1),Vector3i(-1,0,-1),Vector3i(-1,0,0),Vector3i(-1,0,1),Vector3i(-1,1,-1),Vector3i(-1,1,0),Vector3i(-1,1,1),Vector3i(0,-1,-1),Vector3i(0,-1,0),Vector3i(0,-1,1),Vector3i(0,0,-1),Vector3i(0,0,1),Vector3i(0,1,-1),Vector3i(0,1,0),Vector3i(0,1,1),Vector3i(1,-1,-1),Vector3i(1,-1,0),Vector3i(1,-1,1),Vector3i(1,0,-1),Vector3i(1,0,0),Vector3i(1,0,1),Vector3i(1,1,-1),Vector3i(1,1,0),Vector3i(1,1,1)]

# ---------- 物体碰撞: 局部空间TOI滑移 + 最终推出兜底(遍历所有碰撞体) ----------
func object_collision() -> void:
	if not enable_object_collision:
		return
	var n := pos.size()
	var pad: float = cloth_thickness * 0.5 + collision_margin
	var xfs: Array = []
	var xfis: Array = []
	for col in colliders:
		col.begin_step(pad)
		var xf: Transform3D = col.root.global_transform
		xfs.append(xf)
		xfis.append(xf.affine_inverse())
	for i in n:
		if pinned[i] == 1:
			continue
		var pre_w: Vector3 = prev[i]
		var p_w: Vector3 = pos[i]
		# 地面(世界空间)
		if p_w.y < ground_y + pad:
			var slide := Vector3(p_w.x - pre_w.x, 0, p_w.z - pre_w.z)
			p_w = Vector3(p_w.x, ground_y + pad, p_w.z)
			p_w -= slide * friction
		for ci in colliders.size():
			p_w = _collide_one(colliders[ci], xfs[ci], xfis[ci], pre_w, p_w, pad)
		if p_w != pos[i]:
			pos[i] = p_w
			_pushed[i] = 1

func _collide_one(col: XPBDCollider, xf: Transform3D, xfi: Transform3D, pre_w: Vector3, p_w: Vector3, pad: float) -> Vector3:
	var mv := p_w - pre_w
	var ml := mv.length()
	var to_p := p_w - xf.origin
	if to_p.length_squared() > (col.broad_r + pad) * (col.broad_r + pad) + ml * ml + 2.0 * (col.broad_r + pad) * ml:
		return p_w
	var pre: Vector3 = xfi * pre_w
	var p: Vector3 = xfi * p_w
	# 迭代: 最早TOI接触 -> 去法向分量切向滑移 -> 接触点外侧继续扫掠剩余路径
	for _pass in 3:
		var seg := p - pre
		var seg2: float = seg.length_squared()
		var best_t := 2.0
		var best_n := Vector3.ZERO
		for k in col.count:
			col.toi_k(k, pre, seg, seg2)
			if col.ct < best_t:
				best_t = col.ct
				best_n = col.cn
		if best_t > 1.0:
			break
		var cp := pre + seg * best_t
		var rest := seg * (1.0 - best_t)
		var vn: float = rest.dot(best_n)
		if vn < 0.0:
			rest -= best_n * vn
		p = cp + best_n * 0.0006 + rest * (1.0 - friction)
		pre = cp + best_n * 0.0006
	# 兜底: 任意残余穿透一律最小深度推出
	for k in col.count:
		p = col.push_k(k, p)
	return xf * p

# 帧末兜底: 保证物体/地面硬不变量(零位移语义, 只推出不扫掠), 在接触求解之后调用
func final_push_out() -> void:
	if not enable_object_collision:
		return
	var pad: float = cloth_thickness * 0.5 + collision_margin
	var xfs: Array = []
	var xfis: Array = []
	for col in colliders:
		var xf: Transform3D = col.root.global_transform
		xfs.append(xf)
		xfis.append(xf.affine_inverse())
	# 顺序关键(标准"投影约束 -> 再投影硬不变量"): 
	# 1) 先应变限制 — 它会让顶点相互靠近, 可能把顶点拉回碰撞体内
	# 2) 顶点推出
	# 3) 网格级推出(边中点) — 平摊推力时可能把端点反向压回碰撞体/地面下
	# 4) 顶点再推出 + 地面最终夹紧, 恢复"碰撞体/地面是硬不变量"
	# 反过来(推出->应变限制)会破坏硬不变量, 实测拖拽穿透 0.15m; 缺少 4) 时方箱穿透 0.016m 且顶点沉到地面下
	_strain_limit()
	_vertex_push_out(pad, xfs, xfis)
	_mesh_contact_push(pad, xfs, xfis)
	_vertex_push_out(pad, xfs, xfis)

# 逐顶点推出: 地面夹紧 + 全部碰撞体(零位移语义, 只推出)
func _vertex_push_out(pad: float, xfs: Array, xfis: Array) -> void:
	for i in pos.size():
		if pinned[i] == 1:
			continue
		var p := pos[i]
		if p.y < ground_y + pad:
			p = Vector3(p.x, ground_y + pad, p.z)
		for ci in colliders.size():
			p = _collide_one(colliders[ci], xfs[ci], xfis[ci], p, p, pad)
		if p != pos[i]:
			_pushed[i] = 1
		pos[i] = p
	# 地面是最终硬不变量: 网格推动可能越过它 -> 末尾统一夹紧。
	# 同时清零"向下速度": 该夹紧发生在 sub_recover 之后(末子步), 若不处理速度,
	# 下一子步顶点会带向下速度再次扎入 -> 永久抖振(实测 76 顶点 >0.3m/s, 最大 1.22m/s)
	for i in pos.size():
		if pinned[i] == 1:
			continue
		if pos[i].y < ground_y + pad:
			pos[i] = Vector3(pos[i].x, ground_y + pad, pos[i].z)
			_pushed[i] = 1
			if vel[i].y < 0.0:
				vel[i].y = 0.0

# 网格级兜底: 顶点全在碰撞体外时, 三角形边/面仍可能横切碰撞体(球间深谷/地面),
# 肉眼即"布料切入岩石缝隙"。对每条网格边中点与每格面心做碰撞检测, 欠量均摊到端点/顶点。
# 性能: 先按格子AABB与碰撞体包围球/地面粗筛, 绝大多数格子零成本跳过。
var _aabb_cache_bottom: float = 0.0

func _update_aabb_bottom() -> void:
	var lo := 1e9
	for i in pos.size():
		lo = minf(lo, pos[i].y)
	_aabb_cache_bottom = lo

func _mesh_contact_push(pad: float, xfs: Array, xfis: Array) -> void:
	if cols < 2 or rows < 2:
		return
	_update_aabb_bottom()
	# 每碰撞体世界空间包围球(中心+半径)
	var broad_c: Array = []
	var broad_r: Array = []
	for ci in colliders.size():
		broad_c.append(xfs[ci].origin)
		broad_r.append(colliders[ci].broad_r + pad)
	for r in rows - 1:
		for c in cols - 1:
			var a0: int = r * cols + c
			var p0: Vector3 = pos[a0]
			var p1: Vector3 = pos[a0 + cols]
			var p2: Vector3 = pos[a0 + 1]
			var p3: Vector3 = pos[a0 + cols + 1]
			# 格子AABB
			var mn := p0.min(p1).min(p2).min(p3)
			var mx := p0.max(p1).max(p2).max(p3)
			var cell_c := (mn + mx) * 0.5
			var cell_r := (mx - mn).length() * 0.5
			var near_col := false
			for ci in broad_c.size():
				if cell_c.distance_squared_to(broad_c[ci]) <= (broad_r[ci] + cell_r) * (broad_r[ci] + cell_r):
					near_col = true
					break
			if not near_col:
				# 快路径: 只处理地面(纯Y夹紧, 内联, 不进碰撞体循环/局部变换)
				var floor_y := ground_y + pad
				if mn.y <= floor_y:
					var e0 := [a0, a0 + cols, a0 + 1, a0 + cols + 1]
					for ee in [[e0[0], e0[1]], [e0[0], e0[2]], [e0[1], e0[3]], [e0[2], e0[3]], [e0[1], e0[2]]]:
						var ia: int = ee[0]
						var ib: int = ee[1]
						var mid_y := (pos[ia].y + pos[ib].y) * 0.5
						if mid_y < floor_y:
							var wa := _edge_weight(ia)
							var wb := _edge_weight(ib)
							var s := wa + wb
							if s > 0.0:
								var dy := (floor_y - mid_y) / s
								pos[ia].y += dy * wa
								pos[ib].y += dy * wb
								_pushed[ia] = 1
								_pushed[ib] = 1
				continue
			var ids := [a0, a0 + cols, a0 + 1, a0 + cols + 1]
			# 5 条网格边(水平x2 垂直x2 对角x1): 中点检测, 推力平摊两端
			# (面心省略: 顶点+网格边已覆盖桥接, 对角边兜住箱角/球谷的斜切)
			for e in [[ids[0], ids[1]], [ids[0], ids[2]], [ids[1], ids[3]], [ids[2], ids[3]], [ids[1], ids[2]]]:
				_push_edge_mid(e[0], e[1], pad, xfs, xfis)

func _edge_weight(i: int) -> float:
	return 0.0 if (pinned[i] == 1 or (grabbing and i == grab_id)) else 1.0

func _push_edge_mid(ia: int, ib: int, pad: float, xfs: Array, xfis: Array) -> void:
	var wa := _edge_weight(ia)
	var wb := _edge_weight(ib)
	if wa + wb == 0.0:
		return
	var mid_w: Vector3 = (pos[ia] + pos[ib]) * 0.5
	var out_w := mid_w
	# 地面
	if out_w.y < ground_y + pad:
		out_w = Vector3(out_w.x, ground_y + pad, out_w.z)
	for ci in colliders.size():
		var col: XPBDCollider = colliders[ci]
		var xf: Transform3D = xfs[ci]
		var to_p := out_w - xf.origin
		if to_p.length_squared() > (col.broad_r + pad) * (col.broad_r + pad):
			continue
		var p_l: Vector3 = xfis[ci] * out_w
		var inside := false
		for k in col.count:
			var ty: int = col.pt[k]
			if ty == 2:
				var l := p_l - col.poff[k]
				var h := col.php[k]
				if absf(l.x) <= h.x and absf(l.y) <= h.y and absf(l.z) <= h.z:
					inside = true
					break
			else:
				var cen: Vector3 = col.poff[k] if ty == 0 else XPBDCollider.closest_on_seg(col.pa[k], col.pb[k], p_l)
				if (p_l - cen).length_squared() < col.prp[k] * col.prp[k]:
					inside = true
					break
		if inside:
			var pushed_l: Vector3 = p_l
			for k in col.count:
				pushed_l = col.push_k(k, pushed_l)
			out_w = xf * pushed_l
	if out_w != mid_w:
		var delta := out_w - mid_w
		var s := wa + wb
		pos[ia] += delta * (wa / s)
		pos[ib] += delta * (wb / s)
		_pushed[ia] = 1
		_pushed[ib] = 1

# 穿透审计: 最终位置仍深入碰撞体(视觉面一半以上/箱内部)的采样顶点数(应恒为0)
func penetration_count() -> int:
	if not enable_object_collision:
		return 0
	var pad: float = cloth_thickness * 0.5 + collision_margin
	var count := 0
	var n := pos.size()
	var xfis: Array = []
	for col in colliders:
		xfis.append(col.root.global_transform.affine_inverse())
	for i in range(0, n, 3):
		for ci in colliders.size():
			var col: XPBDCollider = colliders[ci]
			var p: Vector3 = xfis[ci] * pos[i]
			for k in col.count:
				var ty: int = col.pt[k]
				if ty == 2:
					var l: Vector3 = p - col.poff[k]
					var h: Vector3 = col.phalf[k]
					if absf(l.x) < h.x and absf(l.y) < h.y and absf(l.z) < h.z:
						count += 1
						break
				else:
					var cen: Vector3 = col.poff[k] if ty == 0 else XPBDCollider.closest_on_seg(col.pa[k], col.pb[k], p)
					var rr: float = col.pr[k] + pad
					if (p - cen).length_squared() < rr * rr * 0.25:
						count += 1
						break
	return count

# 抓取释放: 甩出速度
func release_grab(substeps: int) -> void:
	if grab_id >= 0:
		vel[grab_id] = (grab_target - prev[grab_id]) * 60.0 / maxf(substeps, 1)
	grabbing = false
	grab_id = -1

# 射线拾取布料顶点: 返回(全局)最近命中顶点局部id, 未命中-1; out附深度
func pick_vertex(ro: Vector3, rn: Vector3) -> int:
	var best := -1
	var best_along := 1e9
	var fallback := -1
	var fallback_along := 1e9
	for i in pos.size():
		var to_p := pos[i] - ro
		var along := to_p.dot(rn)
		if along < 0.1:
			continue
		var perp := (to_p - rn * along).length()
		if perp < 0.45 and along < best_along:
			best_along = along
			best = i
		if along < fallback_along:
			fallback_along = along
			fallback = i
	# 命中半径内取最近; 否则回退抓射线上最近点(保证点哪都能拖)
	return best if best >= 0 else fallback

func start_grab(i: int, ro: Vector3, rn: Vector3) -> void:
	grab_id = i
	grabbing = true
	grab_target = pos[i]
	grab_depth = (pos[i] - ro).dot(rn)

func update_mesh() -> void:
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = pos
	var indices := PackedInt32Array()
	for r in rows - 1:
		for c in cols - 1:
			var a0 := r * cols + c
			indices.append_array([a0, a0 + cols, a0 + 1, a0 + 1, a0 + cols, a0 + cols + 1])
	arrays[Mesh.ARRAY_INDEX] = indices
	var normals := PackedVector3Array()
	normals.resize(pos.size())
	for k in range(0, indices.size(), 3):
		var a := indices[k]
		var b := indices[k + 1]
		var cc := indices[k + 2]
		var nm := (pos[b] - pos[a]).cross(pos[cc] - pos[a])
		normals[a] += nm
		normals[b] += nm
		normals[cc] += nm
	for i in pos.size():
		normals[i] = normals[i].normalized() if normals[i].length() > 1e-8 else Vector3(0, 0, 1)
	arrays[Mesh.ARRAY_NORMAL] = normals
	mesh = ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mesh.surface_set_material(0, material)
	mesh_inst.mesh = mesh
