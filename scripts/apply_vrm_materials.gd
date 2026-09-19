# 全身人物验证：VRM 模型
# 皮肤/衣服保留 VRM 原生 MToon 材质；头发应用我们的 Kajiya-Kay 着色器
extends Node3D

const HAIR_SHADER := preload("res://shaders/hair.gdshader")

func _ready() -> void:
	var mesh_nodes := find_children("*", "MeshInstance3D", true, false)
	for mi in mesh_nodes:
		var lname := mi.name.to_lower()
		var parent_name: String = mi.get_parent().name.to_lower() if mi.get_parent() else ""
		var is_hair: bool = lname.contains("hair") or parent_name.contains("hair")
		if not is_hair:
			continue  # 皮肤/衣服保持 VRM 原生材质
		for i in mi.mesh.get_surface_count():
			var orig: Material = mi.get_active_material(i)
			var mat := ShaderMaterial.new()
			mat.shader = HAIR_SHADER
			if orig is BaseMaterial3D:
				var tex: Texture2D = orig.albedo_texture
				if tex:
					mat.set_shader_parameter("hair_texture", tex)
				mat.set_shader_parameter("alpha_scissor", clampf(orig.albedo_color.a, 0.3, 0.6))
			elif orig is ShaderMaterial:
				# MToon 材质：取其主贴图给我们的头发着色器
				var tex2: Texture2D = orig.get_shader_parameter("_MainTex")
				if tex2 == null:
					tex2 = orig.get_shader_parameter("main_tex")
				if tex2:
					mat.set_shader_parameter("hair_texture", tex2)
			# 保持原发色
			mat.set_shader_parameter("root_color", Color(1, 1, 1))
			mat.set_shader_parameter("tip_color", Color(1, 1, 1))
			mat.set_shader_parameter("primary_power", 90.0)
			mat.set_shader_parameter("primary_gain", 0.3)
			mat.set_shader_parameter("translucency", 0.4)
			mi.set_surface_override_material(i, mat)
	_frame_camera()

func _frame_camera() -> void:
	var cam := get_node_or_null("Camera") as Camera3D
	if cam == null:
		return
	var mesh_nodes := find_children("*", "MeshInstance3D", true, false)
	if mesh_nodes.is_empty():
		return
	var aabb: AABB = mesh_nodes[0].global_transform * mesh_nodes[0].get_aabb()
	for mi in mesh_nodes:
		aabb = aabb.merge(mi.global_transform * mi.get_aabb())
	var center := aabb.get_center()
	var dist: float = aabb.size.y * 1.05
	cam.global_position = center + Vector3(0, aabb.size.y * 0.08, dist)
	cam.look_at(center + Vector3(0, aabb.size.y * 0.02, 0), Vector3.UP)
	print("hair meshes processed")
