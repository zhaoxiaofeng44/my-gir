extends Node3D

@onready var parent: Node3D = get_parent_node_3d()
@onready var lighting_setup_buttons: HFlowContainer = %LightingSetupButtons
@onready var cast_shadows: CheckButton = %CastShadows

var default_rot : Vector3

func _ready() -> void:
	default_rot = parent.rotation
	cast_shadows.toggled.connect(on_cast_shadows_toggled)
	for i in lighting_setup_buttons.get_child_count():
		var b = lighting_setup_buttons.get_child(i)
		b.pressed.connect(toggle_light_setup.bind(i))

func _on_reset_lights_pressed() -> void:
	parent.rotation = default_rot
	Autoload.log_message("Reset Lighting Setup Applied")
	
func toggle_light_setup(i: int) -> void:
	if i == self.get_index() and not visible:
		visible = true
		Autoload.log_message("Lighting Setup " + str(i+1) + " Enabled")
	else:
		visible = false

func on_cast_shadows_toggled(toggled_on: bool) -> void:
	for l in get_children():
		l.shadow_enabled = toggled_on
		
	if toggled_on:
		Autoload.log_message("Shadows Enabled")
	else:
		Autoload.log_message("Shadows Disabled")
