const int TEAM_COUNT = 25;
const float MAX_TRAIL = 48.0;
const float MAX_SCENT = 16.0;
const uint HISTORY_CAPACITY = 256u;

struct Agent {
    vec4 position;
    uvec4 identity;
    vec4 state;
    uvec4 journey;
};

struct Plan {
    vec4 step; // proposed x, y, heading, unused
    uvec4 action; // kind (0 wait, 1 explore, 2 encroach, 3 attack), paid, return waypoint, unused
};

struct TeamStats {
    uint spent;
    uint exploring;
    uint encroaching;
    uint attacking;
    uint wins;
    uint arrivals;
    uint defended;
    uint defenseFailed;
    uint defenseDeclined;
    uint outbound;
    uint returning;
    uint homeReturns;
    uint shortcuts;
    uint padding1;
    uint padding2;
    uint padding3;
};

uint hash(uint value) {
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    value *= 0x846ca68bu;
    return value ^ (value >> 16);
}

float random01(uint seed) {
    return float(hash(seed) >> 8) * (1.0 / 16777216.0);
}
