extends SceneTree

const POLAR_PATH := "res://data/boats/j80/sim_polar.csv"
const KNOT := 1852.0 / 3600.0
const CONTROL_DT := 0.02
const RUN_SECONDS := 30.0
const MEAN_SECONDS := 5.0
const TARGET_TWA_DEG := 90.0


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	quit(1)


func _frame_helper_error() -> String:
	var point := SailBoat.core_to_godot_point(1.0, 2.0, 3.0)
	if not point.is_equal_approx(Vector3(1.0, 3.0, -2.0)):
		return "core (1, 2, 3) maps to %s, want (1, 3, -2)" % point
	var back := SailBoat.godot_to_core_point(point)
	if not back.is_equal_approx(Vector3(1.0, 2.0, 3.0)):
		return "the round trip of core (1, 2, 3) gives %s" % back
	if not (SailBoat.core_to_godot_basis() * Vector3(1.0, 2.0, 3.0)).is_equal_approx(point):
		return "core_to_godot_basis disagrees with core_to_godot_point"
	if not SailBoat.compass_dir(0.0).is_equal_approx(Vector3(0.0, 0.0, -1.0)):
		return "north is %s, want -Z" % SailBoat.compass_dir(0.0)
	if not SailBoat.compass_dir(90.0).is_equal_approx(Vector3(1.0, 0.0, 0.0)):
		return "east is %s, want +X" % SailBoat.compass_dir(90.0)
	var hull := SailBoat.hull_to_local_basis()
	if not (hull * Vector3(1.0, 0.0, 0.0)).is_equal_approx(Vector3(0.0, 0.0, -1.0)) \
			or not (hull * Vector3(0.0, 1.0, 0.0)).is_equal_approx(Vector3(-1.0, 0.0, 0.0)):
		return "hull forward and port are local %s and %s, want -Z and -X" % [hull.x, hull.y]
	return ""


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
	if not ClassDB.class_exists("SailBoat"):
		_fail("class SailBoat is not registered")
		return

	var row := _polar_row(10.0, TARGET_TWA_DEG)
	if row.is_empty():
		_fail("no TWS 10, TWA 90 row in " + POLAR_PATH)
		return
	var polar_bsp_kn := float(row["bsp_kn"])

	var boat = ClassDB.instantiate("SailBoat")
	# Wind from the south over a boat heading east: TWA 90 on starboard tack.
	boat.wind_speed_kn = 10.0
	boat.wind_direction_deg = 180.0
	boat.boat_dir = "res://data/boats/j80"
	if not boat.is_active():
		_fail("boat did not load: " + boat.get_last_error())
		boat.free()
		return
	var trimmed: bool = boat.set_controls_from_dictionary({
		"mainsheet": float(row["mainsheet"]),
		"traveler": float(row["traveler"]),
		"backstay": float(row["backstay"]),
		"jib_sheet": float(row["jib_sheet"]),
		"jib_car": float(row["jib_car"]),
		"reef": float(row["reef"]),
	})
	if not trimmed:
		_fail("set_controls_from_dictionary rejected the polar trim")
		boat.free()
		return
	if not boat.reset_boat(Vector3.ZERO, 90.0, 3.0):
		_fail("reset_boat failed: " + boat.get_last_error())
		boat.free()
		return
	var start: Vector3 = boat.transform.origin

	# Gains as the core autopilot's, in tiller units per radian: the signed TWA rises as the boat
	# turns to port, and the tiller to starboard (negative) turns it to port.
	var kp := 1.6
	var kd := 1.6
	var bsp_sum := 0.0
	var heel_sum := 0.0
	var samples := 0
	var steps := 0
	var t := 0.0
	while t < RUN_SECONDS - 1e-9:
		var s: Dictionary = boat.get_state()
		var error := deg_to_rad(TARGET_TWA_DEG - float(s["course_twa_deg"]))
		var yaw_rate := deg_to_rad(float(s["yaw_rate_dps"]))
		boat.tiller = clampf(-(kp * error - kd * yaw_rate), -1.0, 1.0)
		steps += boat.step_seconds(CONTROL_DT)
		t += CONTROL_DT
		if t > RUN_SECONDS - MEAN_SECONDS:
			var after: Dictionary = boat.get_state()
			bsp_sum += float(after["bsp_kn"])
			heel_sum += float(after["heel_deg"])
			samples += 1

	if not boat.is_active() or boat.get_last_error() != "":
		_fail("boat went inert: " + boat.get_last_error())
		boat.free()
		return
	var state: Dictionary = boat.get_state()
	var bsp_kn := bsp_sum / samples
	var heel_deg := heel_sum / samples
	var moved: Vector3 = boat.transform.origin - start
	var basis: Basis = boat.transform.basis
	print("steps %d, sim time %.2f s" % [steps, float(state["time_s"])])
	print("BSP %.3f kn vs polar %.3f kn (%.1f%%), heel %.2f deg vs polar %s deg, course TWA %.2f deg" % [
		bsp_kn, polar_bsp_kn, 100.0 * (bsp_kn - polar_bsp_kn) / polar_bsp_kn, heel_deg,
		row["heel_deg"], float(state["course_twa_deg"])])
	print("moved %s m, roll %.2f deg, crew %.2f m to port, rudder %.2f deg, hinge %.1f N*m" % [
		moved, rad_to_deg(basis.get_euler().z), float(state["crew_to_port_m"]),
		float(state["rudder_angle_deg"]), float(state["rudder_hinge_moment_nm"])])
	boat.free()

	if steps != int(round(RUN_SECONDS * 200.0)):
		_fail("expected %d fixed steps, ran %d" % [int(round(RUN_SECONDS * 200.0)), steps])
		return
	if absf(bsp_kn - polar_bsp_kn) > 0.1 * polar_bsp_kn:
		_fail("BSP %.3f kn is not within 10%% of the polar's %.3f kn" % [bsp_kn, polar_bsp_kn])
		return
	# East is Godot +X; the boat should have gone mostly that way, a little downwind (north, -Z).
	var expected_distance := polar_bsp_kn * KNOT * (RUN_SECONDS - 10.0)
	if moved.x < expected_distance or absf(moved.z) > 0.25 * moved.x or absf(moved.y) > 0.5:
		_fail("the boat moved %s, expected mostly along +X" % moved)
		return
	# Starboard tack heels to port: a negative core heel, so the starboard side (local +X) rises
	# and the roll about local +Z is positive.
	if heel_deg >= -2.0:
		_fail("heel %.2f deg, expected to port (negative)" % heel_deg)
		return
	if basis.x.y <= 0.0 or basis.get_euler().z <= 0.0:
		_fail("the node's roll %s does not show the heel to port" % basis.get_euler())
		return
	if absf(rad_to_deg(basis.get_euler().z) + heel_deg) > 0.5:
		_fail("roll %.2f deg does not mirror heel %.2f deg" % [rad_to_deg(basis.get_euler().z), heel_deg])
		return
	if not state.is_read_only():
		_fail("get_state should return a read-only Dictionary")
		return
	var frame_error := _frame_helper_error()
	if frame_error != "":
		_fail(frame_error)
		return

	print("PASS")
	quit(0)
