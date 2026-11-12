#version 440
#pragma include "common.glsl"

layout(std430, binding = 1) readonly buffer Trails { vec4 trails[]; };
uniform ivec2 gridSize;
uniform vec2 drawSize;
uniform float routeThreshold;
uniform vec4 teamColors[TEAM_COUNT];
uniform bool showPheromones;
uniform bool showRoutes;
out vec4 outputColor;

float routeAt(ivec2 cell, int team) {
    cell = clamp(cell, ivec2(0), gridSize - 1);
    return trails[(cell.x + cell.y * gridSize.x) * TEAM_COUNT + team].w;
}

void main() {
    vec2 uv = vec2(gl_FragCoord.x / drawSize.x, 1.0 - gl_FragCoord.y / drawSize.y);
    ivec2 cell = clamp(ivec2(uv * vec2(gridSize)), ivec2(0), gridSize - 1);
    int field = (cell.x + cell.y * gridSize.x) * TEAM_COUNT;
    vec3 fogColor = vec3(0.0);
    float fogTotal = 0.0;
    float strongestScent = 0.0;
    float strongestRoute = 0.0;
    int owner = 0;
    for (int team = 0; team < TEAM_COUNT; ++team) {
        vec4 density = trails[field + team];
        float scent = max(max(density.x, density.y), density.z);
        float weight = log(1.0 + scent);
        fogColor += teamColors[team].rgb * weight;
        fogTotal += weight;
        strongestScent = max(strongestScent, scent);
        if (density.w > strongestRoute) {
            strongestRoute = density.w;
            owner = team;
        }
    }
    vec3 color = vec3(0.018, 0.023, 0.033);
    if (showPheromones) {
        // Scent is a translucent, subdued cloud, separate from route strength.
        float opacity = 0.38 * (1.0 - exp(-strongestScent * 0.6));
        vec3 tint = mix(vec3(0.16), fogColor / max(fogTotal, 0.0001), 0.7);
        color = mix(color, tint, opacity);
    }
    if (showRoutes) {
        // Show developing branches too; crossing the threshold only changes
        // the emphasis and combat cost, rather than creating a route object.
        float opacity = 0.6 * clamp(strongestRoute / routeThreshold, 0.0, 1.0);
        color = mix(color, teamColors[owner].rgb * 0.65, opacity);
    }
    if (showRoutes && strongestRoute >= routeThreshold) {
        // Solid team-colored routes have a crisp dark border; they never turn
        // into white blobs that can be confused with the agent sprites.
        vec2 withinCell = fract(uv * vec2(gridSize));
        vec2 border = clamp(0.75 * vec2(gridSize) / drawSize, vec2(0.08), vec2(0.3));
        bool edge = (withinCell.x < border.x && routeAt(cell - ivec2(1, 0), owner) < routeThreshold) ||
                    (withinCell.x > 1.0 - border.x && routeAt(cell + ivec2(1, 0), owner) < routeThreshold) ||
                    (withinCell.y < border.y && routeAt(cell - ivec2(0, 1), owner) < routeThreshold) ||
                    (withinCell.y > 1.0 - border.y && routeAt(cell + ivec2(0, 1), owner) < routeThreshold);
        color = teamColors[owner].rgb * (edge ? 0.28 : 0.82);
    }
    outputColor = vec4(color, 1.0);
}
