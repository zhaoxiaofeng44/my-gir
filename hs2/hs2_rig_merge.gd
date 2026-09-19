# hs2_rig_merge.gd — 按 myHs2 的权威装配逻辑，把 HS2 七个部件并成单一骨架 + 统一蒙皮
#
# 依据 myHs2/docs/head_body_merge.md（作者本人的推导）：
#   ① 部件是分离资产：head 只含 45 根面骨顶点组、body 的 184 骨里一根面骨都没有，
#      两套骨架「命名空间互斥 + rest 空间不同」，所以 head 不能按同名顶点组 rebind。
#   ② 装配只有一个权威来源 —— Blender golden 装配矩阵（hs2core/ref_head/head_ref.json）：
#          A(t) = boneHeadWorld(t) · R_golden · (M_load)⁻¹
#      R_golden：head = 纯缩放 0.01（相对 cf_J_Head_s 无旋转无平移）
#                hair_f/hair_b = T(0, 0.05, 0) · Rx(-90°) · 0.01
#      文档同时点明两侧导入轴不同（Blender 侧 body=RotX(180°)、head=RotX(90°)），
#      这就是「不能自己拼变换、必须用 R_golden」的原因。
#   ③ 换绑到合并骨架时（three 语义）：bindMatrix_new = A(0)，Xᵢ_new = Xᵢ_load · A(0)⁻¹。
#
# Godot 侧等价实现（实测判别出 Godot 蒙皮为 v = S · Pose · bind · v）：
#   • bind 保持部件原值不动；
#   • rest 用 bind⁻¹（use_bind_inverse），于是静止渲染 = A · v_mesh，
#     与 R_golden「定义在 mesh 局部空间」的口径一致；
#   • A 的表达：A_head = bone_p（头骨 rest 在渲染空间的变换，含换基 V=Rx(+90°)），
#     A_hair = A_head · T(0,0.05,0) · Rx(-90°)（照抄 golden 的相对关系）。
#   • 再把网格顶点的骨骼序号重映射到合并骨架（对应 JS 的 remapSkinIndex）。
class_name HS2RigMerge

const BODY_FAMILY := ["body", "top", "bot", "shoe"]
const ATTACH_BONES := ["cf_J_Head_s", "cf_J_Head", "cf_J_Neck"]

## 返回统一 Skin（所有部件 mesh 都改用它）
static func merge_into_body(body_sk: Skeleton3D, entries: Array) -> Skin:
	var body_entry: Dictionary = {}
	for e in entries:
		if e.get("part", "") == "body":
			body_entry = e
			break
	if body_entry.is_empty():
		push_error("[HS2RigMerge] 缺少 body 部件")
		return Skin.new()
	var body_mi: MeshInstance3D = body_entry["mi"]
	var merged: Skin = body_mi.skin.duplicate() as Skin

	var attach_idx := -1
	for bn in ATTACH_BONES:
		attach_idx = body_sk.find_bone(bn)
		if attach_idx >= 0:
			break

	# 头骨在渲染空间（P 空间）的变换；头/发都挂在它上面（见文件头说明）
	var bone_p := _bone_p_space(body_sk, attach_idx)
	print("[HS2RigMerge] attach_bone=", ATTACH_BONES[0], " bone_p=", bone_p)

	for e in entries:
		var part: String = e.get("part", "")
		if part in BODY_FAMILY:
			continue   # body/top/bot/shoe 共用 body 骨架，不必并入
		var part_sk: Skeleton3D = e["sk"]
		var part_mi: MeshInstance3D = e["mi"]
		var use_bind_inverse := true
		var A: Transform3D
		if part == "head":
			A = bone_p                                   # R_golden: 无旋转无平移
		else:
			# golden 的 HAIR_CORR = T(0,-0.05,0)·Rx(-90°)（在 mesh 局部空间），
			# 换算到本工程坐标系的相对增量 = T(0, 0, +0.05) · Rx(-90°)。
			# 注意平移是 z=+0.05（不是 y），写错轴会让头发整体偏 5cm。
			A = bone_p * Transform3D(Basis(Vector3.RIGHT, -PI / 2.0), Vector3(0, 0, 0.05))

		var map := {}
		for i in part_sk.get_bone_count():
			map[i] = body_sk.add_bone(part_sk.get_bone_name(i))
		# 对应 JS 的 remapSkinIndex：把顶点骨骼序号从「部件骨架序号」→「合并骨架序号」
		remap_mesh_bones(part_mi, map)

		var old_skin: Skin = part_mi.skin
		var global_rests := {}
		for i in part_sk.get_bone_count():
			var old_bind: Transform3D = old_skin.get_bind_pose(i) if old_skin != null and i < old_skin.get_bind_count() else Transform3D()
			var src: Transform3D = old_bind.affine_inverse() if use_bind_inverse else part_sk.get_bone_global_rest(i)
			global_rests[i] = A * src
		for i in part_sk.get_bone_count():
			var idx: int = map[i]
			var p: int = part_sk.get_bone_parent(i)
			var global_rest: Transform3D = global_rests[i]
			var parent_global: Transform3D
			if p < 0:
				if attach_idx >= 0:
					body_sk.set_bone_parent(idx, attach_idx)
					parent_global = body_sk.get_bone_global_rest(attach_idx)
				else:
					parent_global = Transform3D()
			else:
				body_sk.set_bone_parent(idx, map[p])
				parent_global = global_rests[p]
			body_sk.set_bone_rest(idx, parent_global.affine_inverse() * global_rest)
			var old_bind2: Transform3D = old_skin.get_bind_pose(i) if old_skin != null and i < old_skin.get_bind_count() else body_sk.get_bone_global_rest(idx)
			merged.add_bind(idx, old_bind2)
	return merged

## 某骨在渲染空间（P 空间）的变换。
## 实测：骨 rest 空间 → P 空间 的换基 V = Rx(+90°)：(x,y,z) → (x, -z, y)
static func _bone_p_space(sk: Skeleton3D, bone_idx: int) -> Transform3D:
	if bone_idx < 0:
		return Transform3D()
	var V := Basis(Vector3.RIGHT, PI / 2.0)
	var r: Transform3D = sk.get_bone_global_rest(bone_idx)
	return Transform3D(V * r.basis * V.inverse(), V * r.origin)

## 重映射网格顶点的骨骼索引（old_bone → merged_bone）
static func remap_mesh_bones(mi: MeshInstance3D, map: Dictionary) -> void:
	var src: Mesh = mi.mesh
	var out := ArrayMesh.new()
	out.resource_name = src.resource_name
	for surf in src.get_surface_count():
		var arrays: Array = src.surface_get_arrays(surf)
		if arrays.size() > Mesh.ARRAY_BONES and arrays[Mesh.ARRAY_BONES] != null:
			var bones: PackedInt32Array = arrays[Mesh.ARRAY_BONES]
			for i in bones.size():
				bones[i] = int(map.get(bones[i], 0))
			arrays[Mesh.ARRAY_BONES] = bones
		out.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		out.surface_set_name(surf, src.surface_get_name(surf))
		out.surface_set_material(surf, src.surface_get_material(surf))
	mi.mesh = out

## 把部件 mesh 重挂到合并骨架下（统一蒙皮）
static func rebind_mesh(mi: MeshInstance3D, sk: Skeleton3D, skin: Skin) -> void:
	var parent := mi.get_parent()
	if parent != null:
		parent.remove_child(mi)
	sk.add_child(mi)
	mi.transform = Transform3D()
	mi.skin = skin
	mi.skeleton = NodePath("..")
