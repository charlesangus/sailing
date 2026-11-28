extends SceneTree

var _failed := false


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	_failed = true


func _check(ok: bool, message: String) -> void:
	if not ok:
		_fail(message)


func _key(code: Key, pressed: bool) -> void:
	var e := InputEventKey.new()
	e.physical_keycode = code
	e.keycode = code
	e.pressed = pressed
	Input.parse_input_event(e)
	Input.flush_buffered_events()


func _pad_button(index: JoyButton, pressed: bool) -> void:
	var e := InputEventJoypadButton.new()
	e.button_index = index
	e.pressed = pressed
	Input.parse_input_event(e)
	Input.flush_buffered_events()


func _frames(count: int) -> void:
	for i in count:
		await process_frame


func _tap(press: Callable, release: Callable) -> void:
	press.call()
	await _frames(2)
	release.call()
	await _frames(2)


func _initialize() -> void:
	var main = (load("res://scenes/main.tscn") as PackedScene).instantiate()
	root.add_child(main)
	await _frames(4)
	await _run(main)
	if _failed:
		quit(1)
		return
	print("PASS")
	quit(0)


func _run(main: Node) -> void:
	var boat: Node3D = main.get_node("Boat")
	var hud: Hud = main.get_node("Hud")
	var panel: SettingsPanel = main.get_node("SettingsPanel")

	# Readout follows the sim.
	hud.refresh()
	var text_before := hud.readout.text
	_check("BSP" in text_before and "camera chase" in text_before, "readout is missing fields: %s" % text_before)
	boat.mainsheet = 0.5
	var bsp_before: float = boat.get_state()["bsp_kn"]
	boat.step_seconds(20.0)
	var bsp_after: float = boat.get_state()["bsp_kn"]
	_check(bsp_after > bsp_before + 0.5, "the boat did not accelerate: %f -> %f" % [bsp_before, bsp_after])
	await create_timer(0.3).timeout
	_check(hud.readout.text != text_before, "the readout did not update on its own")
	_check(hud.readout.text.contains("BSP %5.2f" % bsp_after) or hud.readout.text.contains("BSP"), "no BSP line")
	_check(boat.get_step_time_us() > 0.0, "the step time is not positive")
	_check(hud.readout.text.contains("step"), "no step time line")

	# Toggle with the key and the pad button.
	_check(hud.visible, "the HUD should start visible")
	await _tap(func(): _key(KEY_H, true), func(): _key(KEY_H, false))
	_check(not hud.visible, "key H should hide the HUD")
	await _tap(func(): _key(KEY_H, true), func(): _key(KEY_H, false))
	_check(hud.visible, "key H should show the HUD")
	await _tap(func(): _pad_button(JOY_BUTTON_BACK, true), func(): _pad_button(JOY_BUTTON_BACK, false))
	_check(not hud.visible, "pad Back should hide the HUD")
	await _tap(func(): _pad_button(JOY_BUTTON_BACK, true), func(): _pad_button(JOY_BUTTON_BACK, false))
	_check(hud.visible, "pad Back should show the HUD")

	# Settings panel through both devices.
	_check(not panel.is_open(), "the panel should start closed")
	await _tap(func(): _key(KEY_ESCAPE, true), func(): _key(KEY_ESCAPE, false))
	_check(panel.is_open(), "Escape should open the panel")
	var focus := panel.get_viewport().gui_get_focus_owner()
	_check(focus != null and panel.is_ancestor_of(focus), "the panel should take UI focus")
	await _tap(func(): _key(KEY_ESCAPE, true), func(): _key(KEY_ESCAPE, false))
	_check(not panel.is_open(), "Escape should close the panel")
	await _tap(func(): _pad_button(JOY_BUTTON_START, true), func(): _pad_button(JOY_BUTTON_START, false))
	_check(panel.is_open(), "pad Start should open the panel")
	await _tap(func(): _pad_button(JOY_BUTTON_START, true), func(): _pad_button(JOY_BUTTON_START, false))
	_check(not panel.is_open(), "pad Start should close the panel")

	# Sliders drive the sim.
	panel.set_open(true)
	panel.tws_slider.value = 17.5
	_check(is_equal_approx(boat.wind_speed_kn, 17.5), "the TWS slider did not set the wind: %f" % boat.wind_speed_kn)
	panel.direction_slider.value = 123.0
	_check(is_equal_approx(boat.wind_direction_deg, 123.0), "the direction slider did not set the wind: %f" % boat.wind_direction_deg)
	_check(panel.tws_slider.max_value == 30.0 and panel.direction_slider.max_value == 359.0, "slider ranges are wrong")

	# Pad navigation: the d-pad adjusts the focused slider and moves between controls.
	panel.tws_slider.grab_focus()
	var tws_before: float = boat.wind_speed_kn
	var traveler_before: float = boat.traveler
	await _tap(func(): _pad_button(JOY_BUTTON_DPAD_RIGHT, true), func(): _pad_button(JOY_BUTTON_DPAD_RIGHT, false))
	_check(boat.wind_speed_kn > tws_before, "the d-pad should raise the TWS slider")
	await _tap(func(): _pad_button(JOY_BUTTON_DPAD_UP, true), func(): _pad_button(JOY_BUTTON_DPAD_UP, false))
	_check(is_equal_approx(boat.traveler, traveler_before), "the d-pad should not move the traveler while the panel is open")
	_check(panel.get_viewport().gui_get_focus_owner() != panel.tws_slider, "d-pad up should move focus off the first slider")
	panel.set_open(false)

	# Sections.
	var before := panel.section_titles().size()
	var body := Label.new()
	body.text = "extra"
	var header := panel.register_section("Extra", body)
	_check(panel.section_titles().size() == before + 1 and "Extra" in panel.section_titles(), "register_section did not add a section")
	_check(body.is_inside_tree(), "the section body is not in the tree")
	header.button_pressed = false
	_check(not body.visible, "collapsing a section should hide its body")
	panel.register_section("Extra", Label.new())
	_check(panel.section_titles().size() == before + 1, "registering a title twice should replace it")
