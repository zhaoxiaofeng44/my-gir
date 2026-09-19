# 高模验证：皮肤着色器(带法线贴图) + 程序化头发 + 自动取景
extends Node3D

const SKIN_SHADER := preload("res://shaders/skin.gdshader")
const HAIR_SHADER := preload("res://shaders/hair.gdshader")
const ALBEDO_TEX := preload("res://addons/HumanShaders/Models/HumanHead/InfiniteScan/Images/Map-COL.jpg")
const NRM_TEX := preload("res://addons/HumanShaders/Models/HumanHead/InfiniteScan/Infinite-Level_02_NRM_AO_4k.png")

func _ready() -> void:
	var mat := ShaderMaterial.new()
	mat.shader = SKIN_SHADER
	mat.set_shader_parameter("albedo_texture", ALBEDO_TEX)
	mat.set_shader_parameter("use_normal_map", true)
	mat.set_shader_parameter("normal_texture", NRM_TEX)
	mat.set_shader_parameter("normal_strength", 0.8)
	mat.set_shader_parameter("base_color", Color(1, 1, 1))
	mat.set_shader_parameter("roughness", 0.5)
	mat.set_shader_parameter("specular_amount", 0.3)
	mat.set_shader_parameter("subsurface_strength", 1.4)
	mat.set_shader_parameter("rim_intensity", 0.08)
	_apply_recursive(self, mat)
	_build_hair()
	_frame_camera()

func _apply_recursive(node: Node, mat: Material) -> void:
	if node is MeshInstance3D:
		for i in node.mesh.get_surface_count():
			node.set_surface_override_material(i, mat)
	for child in node.get_children():
		_apply_recursive(child, mat)

# 程序化生成发丝 alpha 贴图（竖向发丝 + 噪声断口）
func _make_hair_texture(size: int = 1024) -> ImageTexture:
	var img := Image.create(size, size, false, Image.FORMAT_RGBA8)
	var strand_count := 3200
	var rng := RandomNumberGenerator.new()
	rng.seed = 7
	for s in strand_count:
		var x := rng.randi_range(0, size - 1)
		var width := rng.randi_range(2, 4)
		var y := 0
		var drift := 0.0
		while y < size:
			drift += rng.randf_range(-0.6, 0.6)
			drift = clamp(drift, -3.0, 3.0)
			var xx := int(wrapf(x + drift, 0, size))
			var a := rng.randf_range(0.55, 1.0)
			for w in width:
				var px := int(wrapf(xx + w, 0, size))
				img.set_pixel(px, y, Color(1, 1, 1, a))
			y += rng.randi_range(1, 3)
	return ImageTexture.create_from_image(img)

func _build_hair() -> void:
	var mesh_nodes := find_children("*", "MeshInstance3D", true, false)
	if mesh_nodes.is_empty():
		return
	var head := mesh_nodes[0] as MeshInstance3D
	var gaabb: AABB = head.global_transform * head.mesh.get_aabb()
	var hair := MeshInstance3D.new()
	hair.name = "HairShell"
	hair.mesh = head.mesh  # 与头模同一网格，完美贴合
	add_child(hair)
	hair.global_transform = head.global_transform
	var h := gaabb.size.y
	var mat := ShaderMaterial.new()
	mat.shader = HAIR_SHADER
	mat.set_shader_parameter("hair_texture", _make_hair_texture())
	mat.set_shader_parameter("root_color", Color(0.16, 0.10, 0.07))
	mat.set_shader_parameter("tip_color", Color(0.50, 0.34, 0.20))
	mat.set_shader_parameter("alpha_scissor", 0.4)
	mat.set_shader_parameter("primary_power", 90.0)
	mat.set_shader_parameter("primary_gain", 0.22)
	mat.set_shader_parameter("secondary_power", 22.0)
	mat.set_shader_parameter("translucency", 0.6)
	# 头皮模式：沿法线膨胀 + 只保留头顶/后脑
	mat.set_shader_parameter("scalp_mode", true)
	mat.set_shader_parameter("normal_expand", h * 0.028)
	mat.set_shader_parameter("aabb_min_y", gaabb.position.y)
	mat.set_shader_parameter("aabb_inv_h", 1.0 / gaabb.size.y)
	mat.set_shader_parameter("aabb_min_z", gaabb.position.z)
	mat.set_shader_parameter("aabb_inv_d", 1.0 / gaabb.size.z)
	mat.set_shader_parameter("hairline", 0.55)
	mat.set_shader_parameter("front_limit", 0.62)
	hair.material_override = mat

func _frame_camera() -> void:
	var cam := get_node_or_null("Camera") as Camera3D
	if cam == null:
		return
	var mesh_nodes := find_children("*", "MeshInstance3D", true, false)
	if mesh_nodes.is_empty():
		return
	var mi := mesh_nodes[0] as MeshInstance3D
	var aabb: AABB = mi.global_transform * mi.get_aabb()
	var center := aabb.get_center()
	var radius: float = max(aabb.size.x, max(aabb.size.y, aabb.size.z)) * 0.5
	cam.global_position = center + Vector3(0, aabb.size.y * 0.06, radius * 2.4)
	cam.look_at(center + Vector3(0, aabb.size.y * 0.04, 0), Vector3.UP)