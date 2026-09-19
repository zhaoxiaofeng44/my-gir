extends Label

@export var slider : HSlider

func _process(delta: float) -> void:
	text = str(slider.value)
