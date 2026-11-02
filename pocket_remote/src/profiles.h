// Named camera setups stored in NVS. Values are stored as option labels
// (not indexes) because option lists depend on the camera's mode.
#pragma once
#include "camera.h"

struct Profile {
    char name[16];
    char values[S_COUNT][16]; // "" = not part of this profile
};

namespace Profiles {

static const int MAX = 6;

void begin();
int count();
const Profile &get(int i);
bool addFromCamera(const Camera &cam); // false when full
void remove(int i);
void apply(Camera &cam, const Profile &p);
String summary(const Profile &p);

} // namespace Profiles
