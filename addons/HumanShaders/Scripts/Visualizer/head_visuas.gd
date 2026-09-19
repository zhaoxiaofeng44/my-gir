extends Node3D

@export var mesh: MeshInstance3D
@export var material_index: int = 0
@export var materials: Array[int]
@export var animation_player: AnimationPlayer
@export var standard_material_3d: Material

@onready var infinite_level_02: MeshInstance3D = $"Infinite-level02"
#@onready var mesh: MeshInstance3D = $Emily21

@onready var parent: Node3D = get_parent_node_3d()
@onready var skin_color_picker: ColorPicker = %SkinColorPicker
@onready var roughness_slider: HSlider = %RoughnessSlider
@onready var subsurface_scattering: CheckButton = %"SubsurfaceScattering"
@onready var subsurf_slider: HSlider = %SubSurfSlider
@onready var sss_noise: CheckButton = %SSSNoise
@onready var tinted_shadow: CheckButton = %TintedShadow
@onready var lightwarp: CheckButton = %Lightwarp
@onready var translucency: CheckButton = %Translucency
@onready var standard_mat_toggle: CheckButton = %StandardMaterial

@onready var play_button: Button = %PlayButton
@onready var animation_slider: HSlider = %AnimationSlider

var using_player_slider: bool = false
var paused: bool = false

var default_rot : Vector3
var skin_material : Array[ShaderMaterial]

const idle := AnimationMixer.ANIMATION_CALLBACK_MODE_PROCESS_IDLE
const manual := AnimationMixer.ANIMATION_CALLBACK_MODE_PROCESS_MANUAL

func skin_parameter(p: String) -> Variant:
	var parameter : Variant
	
	for mat in skin_material:
		parameter = mat.get_shader_parameter(p)
		
	return parameter

func _ready() -> void:
	#for i in get_children():
	#	var shader_mat = i.get_surface_override_material(0)
	#	if shader_mat is ShaderMaterial:
	#		skin_material = shader_mat
	#var shader_mats: Array[ShaderMaterial]
	
	for mat in range(mesh.mesh.get_surface_count()):
		skin_material.append(mesh.mesh.surface_get_material(mat))
	#if shader_mat is ShaderMaterial:
	#	skin_material = shader_mat
	
	default_rot = parent.rotation
	skin_color_picker.visible = false
	
	roughness_slider.value_changed.connect(on_roughness_slider_changed)
	roughness_slider.value = skin_parameter("roughness")
	
	subsurface_scattering.toggled.connect(on_sss_toggle)
	subsurface_scattering.button_pressed = skin_parameter("use_subsurface_scattering")
	
	sss_noise.toggled.connect(on_sss_noise_toggle)
	sss_noise.button_pressed = skin_parameter("use_noise")
	
	subsurf_slider.value_changed.connect(on_sss_slider_changed)
	subsurf_slider.value = skin_parameter("subsurface_scattering_strength")
	
	tinted_shadow.toggled.connect(on_tinted_shadow_toggle)
	tinted_shadow.button_pressed = skin_parameter("tinted_shadow_penumbra")
	
	lightwarp.toggled.connect(on_lightwarp_toggle)
	lightwarp.button_pressed = skin_parameter("old_lightwarp_fallof")
	
	translucency.toggled.connect(on_translucency_toggle)
	translucency.button_pressed = skin_parameter("translucency")
	
	standard_mat_toggle.toggled.connect(on_standard_mat_toggle)
	standard_mat_toggle.button_pressed = false
	
	
	play_button.pressed.connect(on_play_pressed)
	if animation_player.callback_mode_process == idle:
		play_button.text = "Pause"
	else:
		play_button.text = "Play"
		
	#animation_player.assigned_animation = "ArmatureAction_002"
	animation_slider.drag_started.connect(on_anim_slider_drag)
	animation_slider.drag_ended.connect(on_anim_slider_drag_end)
	animation_slider.value_changed.connect(on_anim_slider_changed)
	
func _on_reset_model_pressed() -> void:
	parent.rotation = default_rot
	
	Autoload.log_message("Reset Model Rotation Applied")
	
func _on_skin_color_toggled(toggled_on: bool) -> void:
	skin_color_picker.visible = toggled_on

func _on_skin_color_picker_color_changed(color: Color) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("albedo", color)
	
	if color == Color.WHITE:
		Autoload.log_message("Skin color changed to Default")
	else:
		Autoload.log_message("Skin color changed to " + str(color.to_html()))

func on_roughness_slider_changed(value: float) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("roughness", value)
	
	Autoload.log_message("Roughness changed to " + str(value))
	
func on_sss_toggle(value: float) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("use_subsurface_scattering", value)
	
	if value:
		Autoload.log_message("SSS toggled ON")
	else:
		Autoload.log_message("SSS toggled OFF")
	
func on_sss_noise_toggle(value: float) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("use_noise", value)
	
	if value:
		Autoload.log_message("SSS Noise toggled ON")
	else:
		Autoload.log_message("SSS Noise toggled OFF")
	
func on_sss_slider_changed(value: float) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("subsurface_scattering_strength", value)
	
	Autoload.log_message("SSS changed to " + str(value))
	
func on_tinted_shadow_toggle(value: float) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("tinted_shadow_penumbra", value)
	
	if value:
		Autoload.log_message("Tinted Shadow Penumbra toggled ON")
	else:
		Autoload.log_message("Tinted Shadow Penumbra toggled OFF")
	
func on_lightwarp_toggle(value: float) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("old_lightwarp_fallof", value)
	
	if value:
		Autoload.log_message("Lightwarp toggled ON")
	else:
		Autoload.log_message("Lightwarp toggled OFF")
	
func on_translucency_toggle(value: float) -> void:
	for mat in skin_material:
		mat.set_shader_parameter("translucency", value)
	
	if value:
		Autoload.log_message("Translucency toggled ON")
	else:
		Autoload.log_message("Translucency toggled OFF")
		
func on_standard_mat_toggle(value: float) -> void:
	if value:
		mesh.set_surface_override_material(0, standard_material_3d)
	else:
		mesh.set_surface_override_material(0, null)


#Animation Player Logic:
func on_play_pressed() -> void:
	if animation_player.callback_mode_process == idle:
		paused = true
		animation_player.callback_mode_process = manual
		play_button.text = "Play"
	else:
		paused = false
		animation_player.callback_mode_process = idle
		play_button.text = "Pause"
	pass
		
func on_anim_slider_drag() -> void:
	using_player_slider = true
	animation_player.callback_mode_process = manual
func on_anim_slider_drag_end(_value_changed: bool) -> void:
	using_player_slider = false
	if !paused:
		animation_player.callback_mode_process = idle
	animation_slider.value = animation_player.current_animation_position
	
func on_anim_slider_changed(value: float) -> void:
	if using_player_slider:
		animation_player.seek(animation_player.current_animation_length * value, true)
	
func _process(_delta: float) -> void:
	if !using_player_slider:
		animation_slider.value = animation_player.current_animation_position / animation_player.current_animation_length
		#animation_slider.value = wrapf(animation_slider.value, 0.0, 1.0)
