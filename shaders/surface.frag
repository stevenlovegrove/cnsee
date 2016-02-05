uniform sampler2D u_normmap;

varying vec4 Pz0_w;
//varying vec3 n_c;
varying vec2 tex_uv;

const vec3 lightdir = vec3(0.34, 0.34, 0.87);

const float ambient = 0.4;
const float diffuse = 0.4;
const float specular = 0.2;

void main()
{
    vec3 n_c = texture2D(u_normmap, tex_uv ).rgb;
    n_c /= length(n_c);

    vec3 p_c = vec3( 0.0, 0.0, 1.0);
    vec3 eyedir = -1.0 * p_c / length(p_c);

    float ldotn = dot(lightdir,n_c);
    vec3 lightreflect = 2.0*ldotn*n_c + (-1.0) * lightdir;
    float edotr = max(0.0, dot(eyedir,lightreflect));
    float edotr2 = edotr*edotr;
    float edotr4 = edotr2*edotr2;
    float spec = edotr4*edotr4*edotr2;
    float I = ambient + diffuse * ldotn  + specular * spec;
    gl_FragColor = vec4(I,I,I,1.0);

//    gl_FragColor = vec4(n_c, 1.0);
}
