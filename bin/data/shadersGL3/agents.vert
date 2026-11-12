#version 440
#pragma include "common.glsl"
layout(std430, binding = 0) readonly buffer Agents { Agent agents[]; };
uniform mat4 modelViewProjectionMatrix;
uniform ivec2 gridSize;
uniform vec2 drawSize;
uniform vec4 teamColors[TEAM_COUNT];
in vec4 position;
out vec4 agentColor;
void main() {
    Agent agent = agents[gl_VertexID];
    vec2 screenPosition = agent.position.xy / vec2(gridSize) * drawSize + position.xy;
    gl_Position = modelViewProjectionMatrix * vec4(screenPosition, 0.0, 1.0);
    int activity = int(agent.state.z);
    vec3 color = mix(teamColors[agent.identity.x].rgb, vec3(1.0), 0.2);
    agentColor = vec4(color, activity == 0 ? 0.45 : 1.0);
}
