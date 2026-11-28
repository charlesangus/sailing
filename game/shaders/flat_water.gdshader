shader_type spatial;
render_mode cull_back, diffuse_burley, specular_schlick_ggx;

uniform sampler2D normal_a : source_color, filter_linear_mipmap, repeat_enable;
uniform sampler2D normal_b : source_color, filter_linear_mipmap, repeat_enable;
uniform float scale_a = 0.07;
uniform float scale_b = 0.31;
uniform vec2 scroll_a = vec2(0.012, 0.007);
uniform vec2 scroll_b = vec2(-0.02, 0.015);
uniform float normal_strength = 0.35;
uniform float ripple_fade_m = 900.0;

uniform sampler2D depth_tex : filter_linear, repeat_disable;
// xy = world x, z of the texture's min corner; zw = its extent in metres.
uniform vec4 depth_rect = vec4(-2048.0, -2048.0, 4096.0, 4096.0);
uniform float deep_depth_m = 25.0;
uniform float open_sea_depth_m = 60.0;

uniform vec3 shallow_color : source_color = vec3(0.10, 0.62, 0.60);
uniform vec3 deep_color : source_color = vec3(0.01, 0.07, 0.15);
uniform vec3 sky_horizon : source_color = vec3(0.64, 0.75, 0.86);
uniform vec3 sky_zenith : source_color = vec3(0.22, 0.43, 0.75);

varying vec2 world_xz;

void vertex() {
	world_xz = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xz;
}

void fragment() {
	vec2 uv_a = world_xz * scale_a + TIME * scroll_a;
	vec2 uv_b = world_xz * scale_b + TIME * scroll_b;
	vec2 slope = (texture(normal_a, uv_a).xy + texture(normal_b, uv_b).xy) * 2.0 - 2.0;
	float fade = 1.0 - clamp(length(VERTEX) / ripple_fade_m, 0.0, 1.0);
	slope *= normal_strength * fade;
	NORMAL_MAP = normalize(vec3(slope, 1.0)) * 0.5 + 0.5;

	// Plane normal is +Y in world space, so the perturbed world normal is built directly.
	vec3 world_normal = normalize(vec3(slope.x, 1.0, slope.y));
	vec3 view_world = normalize((INV_VIEW_MATRIX * vec4(VIEW, 0.0)).xyz);
	float n_dot_v = clamp(dot(world_normal, view_world), 0.0, 1.0);
	float fresnel = 0.02 + 0.98 * pow(1.0 - n_dot_v, 5.0);

	vec2 depth_uv = (world_xz - depth_rect.xy) / depth_rect.zw;
	bool inside = all(greaterThanEqual(depth_uv, vec2(0.0))) && all(lessThanEqual(depth_uv, vec2(1.0)));
	float depth_m = inside ? texture(depth_tex, depth_uv).r : open_sea_depth_m;
	float deepness = smoothstep(0.0, deep_depth_m, depth_m);
	vec3 body = mix(shallow_color, deep_color, sqrt(deepness));

	vec3 refl = reflect(-view_world, world_normal);
	vec3 sky = mix(sky_horizon, sky_zenith, pow(clamp(refl.y, 0.0, 1.0), 0.5));

	ALBEDO = body * (1.0 - fresnel);
	EMISSION = sky * fresnel;
	ROUGHNESS = 0.06;
	SPECULAR = 0.6;
	METALLIC = 0.0;
}
