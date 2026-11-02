#include "profiles.h"
#include <Preferences.h>

namespace Profiles {

static Profile list[MAX];
static int n = 0;

static void store() {
    Preferences prefs;
    prefs.begin("profiles", false);
    prefs.putInt("n", n);
    if (n > 0) prefs.putBytes("list", list, sizeof(Profile) * n);
    prefs.end();
}

struct Entry {
    SettingId id;
    const char *label;
};

static void addDefault(const char *name, std::initializer_list<Entry> entries) {
    Profile &p = list[n++];
    memset(&p, 0, sizeof(p));
    strlcpy(p.name, name, sizeof(p.name));
    for (const Entry &e : entries) strlcpy(p.values[e.id], e.label, sizeof(p.values[0]));
}

void begin() {
    Preferences prefs;
    prefs.begin("profiles", false);
    n = min((int)prefs.getInt("n", -1), MAX);
    if (n > 0) prefs.getBytes("list", list, sizeof(Profile) * n);
    prefs.end();
    if (n >= 0) return;

    n = 0;
    addDefault("Vlog", {{S_MODE, "Video"}, {S_EXPOSURE, "Auto"}, {S_FORMAT, "4K 30"},
                        {S_EV, "0"}, {S_WB, "Auto"}, {S_COLOR, "Normal"}, {S_FOCUS, "AF-C"},
                        {S_AUDIO, "Stereo"}, {S_VOCAL, "On"}});
    addDefault("Cinematic", {{S_MODE, "Video"}, {S_EXPOSURE, "Manual"}, {S_FORMAT, "4K 24"},
                             {S_ISO, "100"}, {S_SHUTTER, "1/50"}, {S_WB, "5600K"},
                             {S_COLOR, "D-Log2"}, {S_FOCUS, "AF-C"}, {S_AUDIO, "Stereo"},
                             {S_VOCAL, "Off"}});
    addDefault("Low light", {{S_MODE, "Video"}, {S_EXPOSURE, "Auto"}, {S_FORMAT, "4K 25"},
                             {S_EV, "+0.5"}, {S_WB, "Auto"}, {S_COLOR, "Normal"},
                             {S_FOCUS, "AF-C"}, {S_AUDIO, "Stereo"}, {S_VOCAL, "On"}});
    addDefault("Slow-mo", {{S_MODE, "Slow Motion"}, {S_EXPOSURE, "Auto"}, {S_FORMAT, "4K 120"},
                           {S_EV, "0"}, {S_WB, "Auto"}, {S_COLOR, "Normal"}, {S_FOCUS, "AF-C"}});
    store();
}

int count() { return n; }

const Profile &get(int i) { return list[i]; }

bool addFromCamera(const Camera &cam) {
    if (n == MAX) return false;
    Profile &p = list[n];
    memset(&p, 0, sizeof(p));
    // First unused "Custom N" name
    for (int num = 1;; num++) {
        snprintf(p.name, sizeof(p.name), "Custom %d", num);
        bool used = false;
        for (int i = 0; i < n; i++) used |= strcmp(list[i].name, p.name) == 0;
        if (!used) break;
    }
    for (int s = 0; s < S_COUNT; s++)
        if (cam.available((SettingId)s)) strlcpy(p.values[s], cam.label((SettingId)s), sizeof(p.values[0]));
    n++;
    store();
    return true;
}

void remove(int i) {
    if (i < 0 || i >= n) return;
    for (int j = i; j < n - 1; j++) list[j] = list[j + 1];
    n--;
    store();
}

void apply(Camera &cam, const Profile &p) {
    // SettingId order puts mode and exposure first
    for (int s = 0; s < S_COUNT; s++)
        if (p.values[s][0]) cam.setByLabel((SettingId)s, p.values[s]);
}

String summary(const Profile &p) {
    String out = p.values[S_MODE];
    for (SettingId s : {S_FORMAT, S_COLOR, S_EXPOSURE}) {
        if (!p.values[s][0]) continue;
        out += "  ";
        out += p.values[s];
    }
    return out;
}

} // namespace Profiles
