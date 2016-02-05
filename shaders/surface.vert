attribute vec2 uv;

uniform mat4 u_KT_cw;
uniform mat4 u_T_wc;
uniform mat4 u_T_cw;
uniform vec2 u_dim;

// intrinsics
uniform vec2 u_ff;
uniform vec2 u_pp;

uniform sampler2D u_heightmap;
uniform sampler2D u_normmap;

varying vec4 Pz0_w;
//varying vec3 n_c;
varying vec2 tex_uv;

void main()
{
    vec2 p_c = uv * u_dim;

    // Back-project p_c
    vec4 P_c = vec4( (p_c - u_pp) / u_ff, 1.0, 1.0);

    // Rotate into world coords
    vec4 P_w = u_T_wc * P_c;

    // Camera origin in world coords
    vec4 C_w = u_T_wc * vec4(0.0, 0.0, 0.0, 1.0);

    // Intersect with z=0 plane
    float lambda = -C_w.z / (P_w.z - C_w.z);
    vec2 z0 = C_w.xy + lambda * (P_w.xy - C_w.xy);

    tex_uv = z0 / 1.0;

    float h = texture2D(u_heightmap, tex_uv ).r;
//    n_c = (u_T_cw * texture2D(u_normmap, uv ).rgba).xyz;
//    n_c = texture2D(u_normmap, tex_uv ).rgb;
//    n_c /= length(n_c);

    // Coords of point in world frame
    Pz0_w = vec4(z0, h, 1.0);

    if(tex_uv.x > 0.0 && tex_uv.x < 1.0 && tex_uv.y > 0.0 && tex_uv.y < 1.0) {
        // Convert back to camera coords and project into camera
        gl_Position = u_KT_cw * Pz0_w;
    }else{
        gl_Position.w = 0.0 / 0.0;
    }

}
