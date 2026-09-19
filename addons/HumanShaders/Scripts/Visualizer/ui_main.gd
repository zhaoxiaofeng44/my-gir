extends Control

@onready var splash_screen: TextureRect = $SplashScreen

var use_splash_screen : bool

func _ready() -> void:
	if splash_screen.visible == true:
		use_splash_screen = true

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_focus_next"):
		if visible:
			visible = false
		else:
			visible = true
	
	if event is InputEventMouseButton and use_splash_screen:
		if event.button_index == MOUSE_BUTTON_LEFT:
			var tween = get_tree().create_tween()
			tween.tween_property(splash_screen, "modulate", Color.TRANSPARENT, .5)
			tween.tween_callback(Callable(splash_screen, "hide"))
