extends SceneTree

const SCENE_DIR := "res://data/scenes/bay"
const CONTROL_DT := 0.02
const RUN_SECONDS := 30.0
const MEAN_SECONDS := 5.0
const TARGET_TWA_DEG := 90.0
const POLAR_PATH := "res://data/boats/j80/sim_polar.csv"


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	quit(1)


func _polar_row(tws_kn: float, twa_deg: float) -> Dictionary:
	var file := FileAccess.open(POLAR_PATH, FileAccess.READ)
	if file == null:
		return {}
	var header := file.get_csv_line()
	while not file.eof_reached():
		var line := file.get_csv_line()
		if line.size() != header.size():
			continue
		var row := {}
		for i in header.size():
			row[header[i]] = line[i]
		if is_equal_approx(float(row["tws_kn"]), tws_kn) and is_equal_approx(float(row["twa_deg"]), twa_deg):
			return row
	return {}


func _initialize() -> void:
	var row := _polar_row(10.0, TARGET_TWA_DEG)
	if row.is_empty():
		_fail("no TWS 10, TWA 90 row in " + POLAR_PATH)
		return
	var polar_bsp_kn := float(row["bsp_kn"])
	var meta: Variant = JSON.parse_string(FileAccess.get_file_as_string(SCENE_DIR + "/meta.json"))
	if typeof(meta) != TYPE_DICTIONARY:
		_fail("cannot read " + SCENE_DIR + "/meta.json")
		return
	var start_world := Vector2(float(meta["start"]["x_m"]), float(meta["start"]["y_m"]))
	var heading_deg := float(meta["start"]["heading_deg"])

	var boat = ClassDB.instantiate("SailBoat")
	# Wind from the east over a boat heading north: TWA 90 on starboard tack.
	boat.wind_speed_kn = 10.0
	boat.wind_direction_deg = 90.0
	boat.boat_dir = "res://data/boats/j80"
	boat.scene_dir = SCENE_DIR
	if not boat.is_active():
		_fail("boat did not load: " + boat.get_last_error())
		boat.free()
		return
	boat.set_controls_from_dictionary({
		"mainsheet": float(row["mainsheet"]),
		"traveler": float(row["traveler"]),
		"backstay": float(row["backstay"]),
		"jib_sheet": float(row["jib_sheet"]),
		"jib_car": float(row["jib_car"]),
		"reef": float(row["reef"]),
	})
	# Core world (x east, y north) is Godot (x, -z).
	if not boat.reset_boat(Vector3(start_world.x, 0.0, -start_world.y), heading_deg, 3.0):
		_fail("reset_boat failed: " + boat.get_last_error())
		boat.free()
		return
	var first: Dictionary = boat.get_state()
	if not bool(first.get("has_seabed", false)) or not first.has("seabed_z_m"):
		_fail("the bay's seabed is not loaded")
		boat.free()
		return
	var start: Vector3 = boat.transform.origin

	var kp := 1.6
	var kd := 1.6
	var bsp_sum := 0.0
	var samples := 0
	var max_contacts := 0
	var t := 0.0
	while t < RUN_SECONDS - 1e-9:
		var s: Dictionary = boat.get_state()
		var error := deg_to_rad(TARGET_TWA_DEG - float(s["course_twa_deg"]))
		var yaw_rate := deg_to_rad(float(s["yaw_rate_dps"]))
		boat.tiller = clampf(-(kp * error - kd * yaw_rate), -1.0, 1.0)
		boat.step_seconds(CONTROL_DT)
		t += CONTROL_DT
		var after: Dictionary = boat.get_state()
		max_contacts = maxi(max_contacts, int(after["seabed_contacts"]))
		if t > RUN_SECONDS - MEAN_SECONDS:
			bsp_sum += float(after["bsp_kn"])
			samples += 1

	if not boat.is_active() or boat.get_last_error() != "":
		_fail("boat went inert: " + boat.get_last_error())
		boat.free()
		return
	var state: Dictionary = boat.get_state()
	var bsp_kn := bsp_sum / samples
	var moved: Vector3 = boat.transform.origin - start
	print("seabed z %.2f m at the start, %.2f m at the end, BSP %.3f kn, moved %s m, max contacts %d" % [
		float(first["seabed_z_m"]), float(state["seabed_z_m"]), bsp_kn, moved, max_contacts])
	boat.free()

	if absf(float(first["seabed_z_m"]) - (-31.62)) > 0.01:
		_fail("seabed z %.3f m at the start, expected -31.62" % float(first["seabed_z_m"]))
		return
	if max_contacts != 0:
		_fail("%d seabed contacts in open water" % max_contacts)
		return
	if absf(bsp_kn - polar_bsp_kn) > 0.1 * polar_bsp_kn:
		_fail("BSP %.3f kn is not near the polar's %.3f kn" % [bsp_kn, polar_bsp_kn])
		return
	# North is Godot -Z.
	if -moved.z < 0.5 * polar_bsp_kn * 1852.0 / 3600.0 * RUN_SECONDS:
		_fail("the boat moved %s, expected mostly north (-Z)" % moved)
		return

	print("PASS")
	quit(0)
