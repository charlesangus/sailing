class_name SettingsPanel
extends CanvasLayer
## Section-based settings panel opened by the settings action of the BoatInput. Sections are
## collapsible and later features add their own with register_section(). The sim keeps running
## while the panel is open so a changed setting shows its effect on the boat at once; the input
## node is also what closes the panel again, so it must keep polling.

const MAX_WIND_KN := 30.0

@export var boat_path: NodePath
@export var input_path: NodePath

var panel: PanelContainer
var sections: VBoxContainer
var tws_slider: HSlider
var direction_slider: HSlider

var _boat: Node
var _input: Node
var _tws_label: Label
var _direction_label: Label
var _section_bodies: Dictionary = {}


func _ready() -> void:
	layer = 10
	_boat = get_node_or_null(boat_path)
	_input = get_node_or_null(input_path)
	if _input != null:
		_input.settings_toggled.connect(toggle)

	panel = PanelContainer.new()
	panel.set_anchors_and_offsets_preset(Control.PRESET_CENTER_RIGHT, Control.PRESET_MODE_MINSIZE, 16)
	panel.custom_minimum_size = Vector2(360.0, 0.0)
	add_child(panel)
	var margin := MarginContainer.new()
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 10)
	panel.add_child(margin)
	var column := VBoxContainer.new()
	margin.add_child(column)
	var title := Label.new()
	title.text = "Settings"
	column.add_child(title)
	sections = VBoxContainer.new()
	column.add_child(sections)

	register_section("Wind", _build_wind_section())
	panel.visible = false


func is_open() -> bool:
	return panel.visible


func toggle() -> void:
	set_open(not panel.visible)


func set_open(open: bool) -> void:
	if open == panel.visible:
		return
	panel.visible = open
	if _input != null:
		_input.controls_enabled = not open
	if open:
		_sync_from_boat()
		_relink_focus()
		var first := _focusables()
		if not first.is_empty():
			first[0].grab_focus()
	else:
		var focused := get_viewport().gui_get_focus_owner()
		if focused != null and panel.is_ancestor_of(focused):
			focused.release_focus()


## Adds a collapsible section holding control and returns its header button. Registering a title
## twice replaces the earlier section.
func register_section(title: String, control: Control) -> Button:
	if _section_bodies.has(title):
		var old: Node = _section_bodies[title].get_parent()
		sections.remove_child(old)
		old.queue_free()
	var section := VBoxContainer.new()
	section.name = title
	var header := Button.new()
	header.text = "v " + title
	header.toggle_mode = true
	header.button_pressed = true
	header.alignment = HORIZONTAL_ALIGNMENT_LEFT
	header.toggled.connect(func(open: bool) -> void:
		control.visible = open
		header.text = ("v " if open else "> ") + title
		_relink_focus())
	section.add_child(header)
	section.add_child(control)
	sections.add_child(section)
	_section_bodies[title] = control
	_relink_focus()
	return header


func section_titles() -> Array[String]:
	var titles: Array[String] = []
	for title in _section_bodies:
		titles.append(title)
	return titles


func _build_wind_section() -> Control:
	var body := VBoxContainer.new()
	_tws_label = Label.new()
	body.add_child(_tws_label)
	tws_slider = _slider(0.0, MAX_WIND_KN, 0.5)
	tws_slider.value_changed.connect(_on_tws_changed)
	body.add_child(tws_slider)
	_direction_label = Label.new()
	body.add_child(_direction_label)
	direction_slider = _slider(0.0, 359.0, 1.0)
	direction_slider.value_changed.connect(_on_direction_changed)
	body.add_child(direction_slider)
	_sync_from_boat()
	return body


func _slider(lo: float, hi: float, step: float) -> HSlider:
	var slider := HSlider.new()
	slider.min_value = lo
	slider.max_value = hi
	slider.step = step
	slider.custom_minimum_size = Vector2(0.0, 24.0)
	slider.focus_mode = Control.FOCUS_ALL
	slider.scrollable = true
	return slider


func _sync_from_boat() -> void:
	if _boat == null:
		return
	tws_slider.set_value_no_signal(_boat.wind_speed_kn)
	direction_slider.set_value_no_signal(_boat.wind_direction_deg)
	_update_labels()


func _update_labels() -> void:
	_tws_label.text = "True wind speed: %.1f kn" % tws_slider.value
	_direction_label.text = "Wind from: %d deg" % int(direction_slider.value)


func _on_tws_changed(value: float) -> void:
	if _boat != null:
		_boat.wind_speed_kn = value
	_update_labels()


func _on_direction_changed(value: float) -> void:
	if _boat != null:
		_boat.wind_direction_deg = value
	_update_labels()


# Controls a pad can land on, top to bottom: section headers and the focusable controls of the
# bodies that are open.
func _focusables() -> Array[Control]:
	var found: Array[Control] = []
	for section in sections.get_children():
		for child in section.get_children():
			_collect(child, found)
	return found


func _collect(node: Node, found: Array[Control]) -> void:
	if node is Control:
		if not node.visible:
			return
		if node.focus_mode == Control.FOCUS_ALL:
			found.append(node)
	for child in node.get_children():
		_collect(child, found)


# Vertical neighbours form a closed ring, so a stick or d-pad never leaves the panel. Left and
# right stay free because a slider uses them to change its value.
func _relink_focus() -> void:
	var list := _focusables()
	for i in list.size():
		var above := list[(i - 1 + list.size()) % list.size()]
		var below := list[(i + 1) % list.size()]
		list[i].focus_neighbor_top = list[i].get_path_to(above)
		list[i].focus_neighbor_bottom = list[i].get_path_to(below)
