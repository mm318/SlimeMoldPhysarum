#version 440
in vec4 agentColor;
flat in int activity;
out vec4 outputColor;
void main() {
    vec2 point = gl_PointCoord * 2.0 - 1.0;
    float radius = activity == 3 ? abs(point.x) + abs(point.y) : length(point);
    if (radius > 1.0) discard;
    if (radius > 0.64) {
        outputColor = vec4(0.0, 0.0, 0.0, agentColor.a * 0.85);
    } else {
        outputColor = agentColor;
    }
}
