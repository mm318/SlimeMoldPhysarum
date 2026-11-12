const int TEAM_COUNT = 25;
const float MAX_TRAIL = 48.0;
const float MAX_SCENT = 16.0;

struct Agent {
    vec4 position;
    uvec4 identity;
    vec4 state;
};

struct TeamStats {
    uint spent;
    uint exploring;
    uint encroaching;
    uint attacking;
    uint wins;
    uint arrivals;
    uint padding0;
    uint padding1;
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
